/* ================================================================
   FSAE Launch Timer — known-good build
   ONE binary for all three nodes. Role selected by MAC.
   Luna UART: RX=GPIO18 (<-Luna TXD), TX=GPIO17 (->Luna RXD), 250 Hz
   PRETTY=1 human console | PRETTY=0 CSV for the logger
   ================================================================ */
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <HardwareSerial.h>
#include <math.h>

#define PRETTY 0

static const uint8_t MAC_BASE[6]   = {0x14,0xC1,0x9F,0x46,0x9A,0x4C};
static const uint8_t MAC_GATE_A[6] = {0xE0,0x72,0xA1,0xD3,0x07,0xC0};
static const uint8_t MAC_GATE_B[6] = {0xE0,0x72,0xA1,0xD3,0x66,0xA0};

enum { ROLE_BASE = 0, ROLE_GATE_A = 1, ROLE_GATE_B = 2 };
uint8_t role = ROLE_BASE;

#define CHANNEL     1
#define BEACON_MS   2000
#define TX_REPEATS  3
static uint8_t BCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
esp_now_peer_info_t peer = {};

#define LUNA_RX 18
#define LUNA_TX 17
HardwareSerial LunaSerial(1);

#define TRIGGER_RATIO   3.0f
#define RELEASE_RATIO   1.5f
#define LOCKOUT_MS      1000
#define WARMUP_FRAMES   250
#define BASE_ALPHA      0.002f
#define VALIDATE_FRAMES 10
#define DIST_DROP_CM    100

#define MSG_TRIGGER     1
#define MSG_STATUS      2
#define MSG_ACK         3
#define MSG_BEACON      4

#define FLAG_NO_SYNC    0x01
#define FLAG_REPLAY     0x02
#define FLAG_WARMUP     0x04
#define FLAG_UNCONF     0x08

#define HEARTBEAT_MS    1000
#define BASE_GRACE_MS   3000
#define GATE_TIMEOUT_MS 30000
#define LINK_LOST_MS    10000
#define SENSOR_FAULT_MS 2000
#define RETRY_FAST_MS   400
#define RETRY_SLOW_MS   4000
#define RETRY_FAST_N    8
#define MAX_RUN_MS      120000.0
#define ORPHAN_MS       180000

#define PENDING_MAX 32
#define EV_MAX      24
#define SEEN_N      32

typedef struct __attribute__((packed)) {
  uint8_t  ver, node_id, msg_type, flags;
  uint32_t sequence;
  uint64_t timestamp_us;
  uint16_t amplitude, reserved16;
  uint8_t  reserved[8];
} packet_t;

typedef struct __attribute__((packed)) {
  uint8_t  ver, node_id, msg_type, flags;
  uint32_t sequence;
  uint64_t tx_us;
} beacon_t;

typedef struct {
  bool     used;
  uint32_t sequence;
  uint64_t ts;
  uint16_t amp;
  uint8_t  flags;
  uint8_t  tries;
  uint32_t last_tx;
} pending_t;

typedef struct {
  bool     used;
  uint8_t  gate;
  uint32_t sequence;
  uint64_t ts;
  uint16_t amp;
  uint8_t  flags;
  uint32_t rx_ms;
} event_t;

/* ---------------- LED ---------------- */
enum {
  L_BOOT = 0, L_WARMUP, L_ARMED, L_TRIGGER,
  L_BUFFERED, L_NO_LINK, L_SENSOR_FAULT,
  L_BASE_NONE, L_BASE_PARTIAL, L_BASE_OK
};
uint8_t  led_state = L_BOOT;
uint32_t led_trigger_until = 0;

void ledWrite(uint8_t r, uint8_t g, uint8_t b) {
#ifdef RGB_BUILTIN
  rgbLedWrite(RGB_BUILTIN, r, g, b);
#endif
}

void ledService() {
  uint32_t t = millis();
  bool slow = (t % 2000) < 1000;
  bool fast = (t % 500)  < 250;
  uint8_t pulse = 3 + (uint8_t)(12.0 * (0.5 + 0.5 * sin(t / 500.0)));
  switch (led_state) {
    case L_BOOT:         ledWrite(0, 0, 20); break;
    case L_WARMUP:       ledWrite(20, 12, 0); break;
    case L_ARMED:        ledWrite(0, pulse, 0); break;
    case L_TRIGGER:      ledWrite(0, 80, 0); break;
    case L_BUFFERED:     ledWrite(fast ? 30 : 0, fast ? 18 : 0, 0); break;
    case L_NO_LINK:      ledWrite(fast ? 40 : 0, 0, 0); break;
    case L_SENSOR_FAULT: ledWrite(40, 0, 0); break;
    case L_BASE_NONE:    ledWrite(slow ? 30 : 0, 0, 0); break;
    case L_BASE_PARTIAL: ledWrite(25, 15, 0); break;
    case L_BASE_OK:      ledWrite(0, pulse, 0); break;
  }
}

/* ---------------- sync (gate side) ---------------- */
#define SYNC_N 64
double   so_t[SYNC_N], so_o[SYNC_N];
int      so_cnt = 0, so_head = 0;
double   so_l0 = 0;
bool     so_origin = false;
uint32_t last_beacon_seq = 0, last_beacon_ms = 0;
double   off_a = 0, off_b = 0;
bool     off_valid = false;

void resetSync() {
  so_cnt = 0; so_head = 0; so_origin = false;
  off_valid = false; off_a = 0; off_b = 0; last_beacon_seq = 0;
  Serial.println("  ! sync reset (base restarted)");
}

void fitOffset() {
  if (so_cnt < 6) return;
  bool use[SYNC_N];
  for (int i = 0; i < so_cnt; i++) use[i] = true;
  double a = 0, b = 0;
  for (int pass = 0; pass < 2; pass++) {
    double sx=0, sy=0, sxx=0, sxy=0, n=0;
    for (int i = 0; i < so_cnt; i++) {
      if (!use[i]) continue;
      sx += so_t[i];          sy += so_o[i];
      sxx += so_t[i]*so_t[i]; sxy += so_t[i]*so_o[i];
      n++;
    }
    if (n < 4) return;
    double den = n*sxx - sx*sx;
    if (fabs(den) < 1e-9) { a = sy/n; b = 0; break; }
    b = (n*sxy - sx*sy) / den;
    a = (sy - b*sx) / n;
    double ss = 0; int kept = 0;
    for (int i = 0; i < so_cnt; i++) {
      if (!use[i]) continue;
      double r = so_o[i] - (a + b*so_t[i]);
      ss += r*r; kept++;
    }
    double rms = sqrt(ss/kept);
    if (pass == 0 && rms > 1.0)
      for (int i = 0; i < so_cnt; i++) {
        double r = so_o[i] - (a + b*so_t[i]);
        if (fabs(r) > 3.0*rms) use[i] = false;
      }
  }
  if (fabs(b) > 200.0) return;
  off_a = a; off_b = b; off_valid = true;
}

uint64_t toBaseTime(uint64_t local_us) {
  if (!off_valid) return local_us;
  double t   = ((double)local_us - so_l0) / 1e6;
  double out = (double)local_us + (off_a + off_b * t);
  return (out < 0) ? local_us : (uint64_t)out;
}

/* ---------------- gate ---------------- */
pending_t pend[PENDING_MAX];
int       pend_count = 0;

float    base_amp = 0, base_dist = 0;
uint32_t frames = 0, gate_seq = 0, lockout_until = 0;
bool     armed = false, in_event = false;
uint32_t last_frame_ms = 0, bad_frames = 0;

bool     hold_active = false;
uint64_t hold_ts = 0;
uint16_t hold_amp = 0;
int      hold_left = 0;
bool     hold_conf = false;

void txPending(pending_t *e) {
  packet_t p = {};
  p.ver = 1; p.node_id = role; p.msg_type = MSG_TRIGGER;
  p.flags = e->flags | (e->tries > 0 ? FLAG_REPLAY : 0);
  p.sequence = e->sequence;
  p.timestamp_us = e->ts;
  p.amplitude = e->amp;
  for (int i = 0; i < TX_REPEATS; i++) {
    esp_now_send(BCAST, (uint8_t*)&p, sizeof(p));
    delayMicroseconds(800);
  }
  e->tries++;
  e->last_tx = millis();
}

void queueTrigger(uint64_t ts, uint16_t amp, bool confirmed) {
  int slot = -1;
  for (int i = 0; i < PENDING_MAX; i++) if (!pend[i].used) { slot = i; break; }
  if (slot < 0) {
    uint64_t oldest = UINT64_MAX; int oi = 0;
    for (int i = 0; i < PENDING_MAX; i++)
      if (pend[i].ts < oldest) { oldest = pend[i].ts; oi = i; }
    Serial.printf("  ! buffer full - dropped seq %lu\n", pend[oi].sequence);
    slot = oi; pend_count--;
  }
  pend[slot].used     = true;
  pend[slot].sequence = ++gate_seq;
  pend[slot].ts       = ts;
  pend[slot].amp      = amp;
  pend[slot].flags    = (off_valid ? 0 : FLAG_NO_SYNC) | (confirmed ? 0 : FLAG_UNCONF);
  pend[slot].tries    = 0;
  pend_count++;
  txPending(&pend[slot]);
  Serial.printf("  > trigger   seq %-4lu amp %-6u %s%s\n",
                pend[slot].sequence, amp,
                off_valid ? "sync ok" : "NO SYNC",
                confirmed ? "" : "  (unconfirmed)");
}

void ackPending(uint32_t seq) {
  for (int i = 0; i < PENDING_MAX; i++)
    if (pend[i].used && pend[i].sequence == seq) {
      pend[i].used = false; pend_count--; return;
    }
}

void serviceRetries() {
  uint32_t now = millis();
  for (int i = 0; i < PENDING_MAX; i++) {
    if (!pend[i].used) continue;
    uint32_t iv = (pend[i].tries < RETRY_FAST_N) ? RETRY_FAST_MS : RETRY_SLOW_MS;
    if (now - pend[i].last_tx >= iv) txPending(&pend[i]);
  }
}

void gateLoop() {
  static uint8_t  buf[9];
  static uint8_t  idx = 0;
  static uint64_t frame_us = 0;

  while (LunaSerial.available()) {
    uint8_t b = LunaSerial.read();
    if (idx == 0) {
      if (b != 0x59) continue;
      frame_us = esp_timer_get_time();
    }
    if (idx == 1 && b != 0x59) { idx = 0; continue; }
    buf[idx++] = b;
    if (idx < 9) continue;
    idx = 0;

    uint8_t sum = 0;
    for (int i = 0; i < 8; i++) sum += buf[i];
    if (sum != buf[8]) continue;

    uint16_t dist = buf[2] | (buf[3] << 8);
    uint16_t amp  = buf[4] | (buf[5] << 8);

    if (amp >= 65535 || dist == 0 || dist == 65535) {
      if (++bad_frames % 250 == 0)
        Serial.printf("  ! %lu unreliable frames\n", bad_frames);
      continue;
    }

    last_frame_ms = millis();
    frames++;

    if (frames <= WARMUP_FRAMES) {
      base_amp  = (base_amp  == 0) ? amp  : base_amp *0.98f + amp *0.02f;
      base_dist = (base_dist == 0) ? dist : base_dist*0.98f + dist*0.02f;
      if (frames == WARMUP_FRAMES) {
        armed = true;
        Serial.printf("  armed:  baseline amp %.0f | dist %.0f cm\n", base_amp, base_dist);
        Serial.printf("  trigger threshold: amp > %.0f\n\n", base_amp * TRIGGER_RATIO);
      }
      continue;
    }

    if (hold_active) {
      if (dist < base_dist - DIST_DROP_CM) hold_conf = true;
      if (--hold_left <= 0) {
        hold_active = false;
        queueTrigger(hold_ts, hold_amp, hold_conf);
      }
      continue;
    }

    if (in_event) {
      if (amp < base_amp * RELEASE_RATIO) in_event = false;
      continue;
    }
    if (millis() < lockout_until) continue;

    if (amp > base_amp * TRIGGER_RATIO) {
      in_event      = true;
      lockout_until = millis() + LOCKOUT_MS;
      hold_active   = true;
      hold_ts       = toBaseTime(frame_us);
      hold_amp      = amp;
      hold_left     = VALIDATE_FRAMES;
      hold_conf     = false;
      led_state         = L_TRIGGER;
      led_trigger_until = millis() + 250;
      continue;
    }

    base_amp  += BASE_ALPHA * ((float)amp  - base_amp);
    base_dist += BASE_ALPHA * ((float)dist - base_dist);
  }
}

void gateLed() {
  uint32_t now = millis();
  bool sensor_dead = armed && last_frame_ms && (now - last_frame_ms > SENSOR_FAULT_MS);
  bool link_dead   = last_beacon_ms && (now - last_beacon_ms > LINK_LOST_MS);
  if      (sensor_dead)             led_state = L_SENSOR_FAULT;
  else if (link_dead)               led_state = L_NO_LINK;
  else if (pend_count > 0)          led_state = L_BUFFERED;
  else if (now < led_trigger_until) led_state = L_TRIGGER;
  else if (armed)                   led_state = L_ARMED;
  else                              led_state = L_WARMUP;
}

/* ---------------- base ---------------- */
event_t  ev[EV_MAX];
uint32_t seen[3][SEEN_N];
int      seen_head[3] = {0,0,0};
uint32_t last_heard[3] = {0,0,0};
bool     was_live[3]   = {false,false,false};
uint32_t run_no = 0;
double   best_ms = 0;
volatile uint8_t  ack_gate = 0;
volatile uint32_t ack_seq  = 0;

bool alreadySeen(uint8_t g, uint32_t s) {
  for (int i = 0; i < SEEN_N; i++) if (seen[g][i] == s) return true;
  seen[g][seen_head[g]] = s;
  seen_head[g] = (seen_head[g] + 1) % SEEN_N;
  return false;
}

void header() {
#if PRETTY
  Serial.println();
  Serial.println("  run    elapsed     mode    amp in -> out    notes");
  Serial.println("  ---   ---------    ----    -------------    -----");
#else
  Serial.println("# RUN,run,t_start_us,t_stop_us,elapsed_ms,amp_start,amp_stop,mode,flags");
#endif
}

void emitRun(event_t *a, event_t *b, const char *mode) {
  double ms = ((double)b->ts - (double)a->ts) / 1000.0;
  if (ms <= 0 || ms > MAX_RUN_MS) {
    Serial.printf("  ! FAULT   implausible elapsed %.1f ms - discarded\n", ms);
    return;
  }
  run_no++;
  bool best = (best_ms == 0 || ms < best_ms);
  if (best) best_ms = ms;

  uint8_t f = a->flags | b->flags;
  char note[48] = "";
  if (best)             strcat(note, "BEST ");
  if (f & FLAG_REPLAY)  strcat(note, "delayed ");
  if (f & FLAG_UNCONF)  strcat(note, "unconfirmed ");
  if (f & FLAG_NO_SYNC) strcat(note, "NOSYNC ");

#if PRETTY
  Serial.printf("  %3lu   %8.4f s   %-5s   %6u -> %-6u   %s\n",
                run_no, ms/1000.0, mode, a->amp, b->amp, note);
#else
  Serial.printf("RUN,%lu,%llu,%llu,%.3f,%u,%u,%s,0x%02X\n",
                run_no, a->ts, b->ts, ms, a->amp, b->amp, mode, f);
#endif
}

int earliestUnpaired(uint8_t gate, uint64_t after, bool use_after) {
  int best = -1;
  for (int i = 0; i < EV_MAX; i++) {
    if (!ev[i].used || ev[i].gate != gate) continue;
    if (use_after && ev[i].ts <= after) continue;
    if (best < 0 || ev[i].ts < ev[best].ts) best = i;
  }
  return best;
}

bool anyBetween(uint8_t gate, uint64_t t0, uint64_t t1) {
  for (int i = 0; i < EV_MAX; i++)
    if (ev[i].used && ev[i].gate == gate && ev[i].ts > t0 && ev[i].ts < t1) return true;
  return false;
}

void tryPair() {
  // every crossing closes the previous lap and opens the next
  static uint64_t prev_ts = 0;
  static uint16_t prev_amp = 0;
  static uint8_t  prev_flags = 0;
  static bool     have_prev = false;

  while (true) {
    int i = -1;
    for (int j = 0; j < EV_MAX; j++) {
      if (!ev[j].used) continue;
      if (i < 0 || ev[j].ts < ev[i].ts) i = j;
    }
    if (i < 0) return;                       // nothing left

    if (!have_prev) {
      prev_ts = ev[i].ts; prev_amp = ev[i].amp; prev_flags = ev[i].flags;
      have_prev = true;
      ev[i].used = false;
      continue;
    }

    event_t a = { true, 0, 0, prev_ts, prev_amp, prev_flags, 0 };
    emitRun(&a, &ev[i], "lap");

    prev_ts = ev[i].ts; prev_amp = ev[i].amp; prev_flags = ev[i].flags;
    ev[i].used = false;
  }
}

void expireEvents() {
  uint32_t now = millis();
  for (int i = 0; i < EV_MAX; i++) {
    if (!ev[i].used) continue;
    if (now - ev[i].rx_ms > ORPHAN_MS) {
      Serial.printf("  ! FAULT   orphan trigger gate %c seq %lu discarded\n",
                    ev[i].gate == ROLE_GATE_A ? 'A' : 'B', ev[i].sequence);
      ev[i].used = false;
    }
  }
}

void checkLiveness() {
  bool a = last_heard[1] && (millis() - last_heard[1] < GATE_TIMEOUT_MS);
  bool b = last_heard[2] && (millis() - last_heard[2] < GATE_TIMEOUT_MS);
  if (a == was_live[1] && b == was_live[2]) {
    led_state = (a && b) ? L_BASE_OK : (a || b) ? L_BASE_PARTIAL : L_BASE_NONE;
    return;
  }
  was_live[1] = a; was_live[2] = b;
  led_state = (a && b) ? L_BASE_OK : (a || b) ? L_BASE_PARTIAL : L_BASE_NONE;
  Serial.printf("\n  gates: A %-4s | B %-4s        mode: %s\n",
                a ? "ok" : "--", b ? "ok" : "--",
                (a && b) ? "A -> B" : (a || b) ? "single gate" : "none");
  header();
}

void baseHandle(const packet_t *p, int rssi) {
  if (p->node_id < 1 || p->node_id > 2) return;
  last_heard[p->node_id] = millis();

  if (p->msg_type == MSG_STATUS) { checkLiveness(); return; }
  if (p->msg_type != MSG_TRIGGER) return;

  ack_gate = p->node_id;
  ack_seq  = p->sequence;

  if (millis() < BASE_GRACE_MS) return;
  if (alreadySeen(p->node_id, p->sequence)) return;
  if (p->amplitude >= 65535) {
    Serial.printf("  ! FAULT   saturated amplitude on gate %c\n",
                  p->node_id == ROLE_GATE_A ? 'A' : 'B');
    return;
  }

  int slot = -1;
  for (int i = 0; i < EV_MAX; i++) if (!ev[i].used) { slot = i; break; }
  if (slot < 0) { Serial.println("  ! FAULT   event pool full"); return; }

  ev[slot] = { true, p->node_id, p->sequence, p->timestamp_us,
               p->amplitude, p->flags, millis() };

  Serial.printf("   .  gate %c  seq %-4lu rssi %-4d%s\n",
                p->node_id == ROLE_GATE_A ? 'A' : 'B', p->sequence, rssi,
                (p->flags & FLAG_REPLAY) ? "  (delayed)" : "");

  checkLiveness();
  tryPair();
}

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  uint64_t rx_us = esp_timer_get_time();

  if (len == sizeof(beacon_t)) {
    const beacon_t *bc = (const beacon_t*)data;
    if (bc->msg_type != MSG_BEACON || role == ROLE_BASE) return;
    if (bc->sequence < last_beacon_seq) resetSync();
    last_beacon_seq = bc->sequence;
    last_beacon_ms  = millis();
    if (!so_origin) { so_l0 = (double)rx_us; so_origin = true; }
    so_t[so_head] = ((double)rx_us - so_l0) / 1e6;
    so_o[so_head] = (double)bc->tx_us - (double)rx_us;
    so_head = (so_head + 1) % SYNC_N;
    if (so_cnt < SYNC_N) so_cnt++;
    fitOffset();
    return;
  }

  if (len != sizeof(packet_t)) return;
  const packet_t *p = (const packet_t*)data;
  if (role == ROLE_BASE) { baseHandle(p, info->rx_ctrl->rssi); return; }
  if (p->msg_type == MSG_ACK && p->node_id == role) ackPending(p->sequence);
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  ledWrite(0, 0, 20);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);

  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  if      (!memcmp(mac, MAC_BASE,   6)) role = ROLE_BASE;
  else if (!memcmp(mac, MAC_GATE_A, 6)) role = ROLE_GATE_A;
  else if (!memcmp(mac, MAC_GATE_B, 6)) role = ROLE_GATE_B;
  else { Serial.println("  !! MAC not in table - defaulting to GATE_A"); role = ROLE_GATE_A; }

  const char *rn = role == ROLE_BASE   ? "BASE"
                 : role == ROLE_GATE_A ? "GATE A" : "GATE B";

  Serial.println();
  Serial.println("============================================================");
  Serial.printf ("  FSAE LAUNCH TIMER                            %10s\n", rn);
  Serial.printf ("  %02X:%02X:%02X:%02X:%02X:%02X                      ch %d | 250 Hz\n",
                 mac[0],mac[1],mac[2],mac[3],mac[4],mac[5], CHANNEL);
  Serial.println("============================================================");

  esp_now_init();
  esp_now_register_recv_cb(onRecv);
  memcpy(peer.peer_addr, BCAST, 6);
  peer.channel = CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  if (role != ROLE_BASE) {
    LunaSerial.begin(115200, SERIAL_8N1, LUNA_RX, LUNA_TX);
    led_state = L_WARMUP;
    Serial.println("  warming up baseline...");
  } else {
    led_state = L_BASE_NONE;
    Serial.println("  waiting for gates...");
  }
}

void loop() {
  if (role == ROLE_BASE) {
    static uint32_t last_beacon = 0, bseq = 0, last_check = 0;

    if (millis() - last_beacon >= BEACON_MS) {
      last_beacon = millis();
      beacon_t b = {1, 0, MSG_BEACON, 0, ++bseq, (uint64_t)esp_timer_get_time()};
      esp_now_send(BCAST, (uint8_t*)&b, sizeof(b));
    }

    if (ack_gate) {
      packet_t a = {};
      a.ver = 1; a.node_id = ack_gate; a.msg_type = MSG_ACK;
      a.sequence = ack_seq;
      ack_gate = 0;
      for (int i = 0; i < 2; i++) {
        esp_now_send(BCAST, (uint8_t*)&a, sizeof(a));
        delayMicroseconds(600);
      }
    }

    if (millis() - last_check >= 1000) {
      last_check = millis();
      checkLiveness();
      expireEvents();
      tryPair();
    }

  } else {
    gateLoop();
    serviceRetries();
    gateLed();

    static uint32_t last_hb = 0;
    if (millis() - last_hb >= HEARTBEAT_MS) {
      last_hb = millis();
      packet_t p = {};
      p.ver = 1; p.node_id = role; p.msg_type = MSG_STATUS;
      p.flags = armed ? 0 : FLAG_WARMUP;
      p.timestamp_us = esp_timer_get_time();
      p.amplitude = (uint16_t)base_amp;
      esp_now_send(BCAST, (uint8_t*)&p, sizeof(p));
    }
  }
  ledService();
}