"""
FSAE Lap Timer - session logger
Keys:  c = cone hit    x = undo cone    v = void last lap    q = quit
"""
import csv, os, sys, time
from datetime import datetime
import serial
from serial.tools import list_ports

try:
    import msvcrt
    HAVE_KEYS = True
except ImportError:
    HAVE_KEYS = False

LIVE_CSV = "laptimer_current.csv"
BAUD     = 115200

DISCIPLINES = {
    "1": "accel",
    "2": "skidpad",
    "3": "obstacle",
    "4": "circuit",
    "5": "testing",
}

HEADER = ["timestamp","lap","elapsed_s","distance_ft","avg_speed_mph",
          "driver","config","session","discipline","cones_session",
          "amp_in","amp_out","flags","status","notes"]

rows     = []
day_csv  = ""
cones    = 0


def find_port():
    ports = list(list_ports.comports())
    if not ports:
        return None
    for p in ports:
        d = (p.description or "").lower()
        if any(k in d for k in ("cp210", "ch340", "usb serial", "silicon labs")):
            return p.device
    return ports[0].device


def ask(prompt, default=""):
    v = input(f"  {prompt}" + (f" [{default}]" if default else "") + ": ").strip()
    return v or default


def write_all():
    for path in (day_csv, LIVE_CSV):
        try:
            with open(path, "w", newline="", encoding="utf-8") as fh:
                w = csv.writer(fh)
                w.writerow(HEADER)
                w.writerows(rows)
                fh.flush()
                os.fsync(fh.fileno())
        except PermissionError:
            print(f"  ! {path} is locked - close it in Excel")


def handle_key(k):
    global cones
    if k == "c":
        cones += 1
        if rows:
            rows[-1][9] = cones
        print(f"  CONE   session total: {cones}")
        write_all()
    elif k == "x":
        if cones > 0:
            cones -= 1
            if rows:
                rows[-1][9] = cones
            print(f"  cone removed   session total: {cones}")
            write_all()
        else:
            print("  (no cones to remove)")
    elif k == "v":
        if not rows:
            print("  (no laps yet)")
            return
        rows[-1][13] = "void" if rows[-1][13] != "ok" else "ok"
        print(f"  -> lap {rows[-1][1]} marked {rows[-1][13]}")
        write_all()


def main():
    global day_csv, cones

    print("=" * 60)
    print("  FSAE LAP TIMER - session logger")
    print("=" * 60)

    port = find_port()
    if not port:
        print("\n  No serial port found. Plug in the base node and rerun.")
        input("\n  Press Enter to close...")
        return
    print(f"\n  Base node on {port}\n")

    for k, v in DISCIPLINES.items():
        print(f"    {k}. {v}")
    while True:
        sel = ask("Discipline", "2")
        if sel in DISCIPLINES:
            break
        print("  Pick a number from the list.")
    discipline = DISCIPLINES[sel]

    # gate spacing only matters for accel
    distance_ft = 0.0
    if discipline == "accel":
        while True:
            try:
                distance_ft = float(ask("Gate spacing in FEET", "200"))
                if distance_ft > 0:
                    break
            except ValueError:
                pass
            print("  Enter a positive number.")

    driver  = ask("Driver name", "unknown")
    config  = ask("Car config", "baseline")
    session = ask("Session label", "S1")

    day = datetime.now().strftime("%Y-%m-%d")
    day_csv = f"laptimer_{day}_{session}_{discipline}.csv"

    # resume an existing session file
    if os.path.exists(day_csv):
        try:
            with open(day_csv, newline="", encoding="utf-8") as fh:
                r = list(csv.reader(fh))
                if len(r) > 1:
                    rows.extend(r[1:])
                    try:
                        cones = int(rows[-1][9])
                    except (ValueError, IndexError):
                        cones = 0
                    print(f"  resuming - {len(rows)} laps, {cones} cones")
        except Exception as e:
            print(f"  ! could not read existing session ({e})")

    write_all()

    try:
        ser = serial.Serial(port, BAUD, timeout=0.2)
    except Exception as e:
        print(f"\n  Could not open {port}: {e}")
        print("  Is the Arduino IDE serial monitor still open?")
        input("\n  Press Enter to close...")
        return

    print(f"\n  Discipline   : {discipline}")
    print(f"  Session file : {day_csv}")
    if HAVE_KEYS:
        print("  Keys: [c] cone hit   [x] undo cone   [v] void lap   [q] quit\n")
    else:
        print("  Ctrl-C to stop.\n")

    try:
        while True:
            if HAVE_KEYS and msvcrt.kbhit():
                k = msvcrt.getch().decode("utf-8", errors="ignore").lower()
                if k == "q":
                    break
                handle_key(k)

            raw = ser.readline().decode("utf-8", errors="replace").strip()
            if not raw:
                continue

            if raw.startswith("RUN,"):
                p = raw.split(",")
                if len(p) < 9:
                    continue
                try:
                    lap_no  = int(p[1])
                    elapsed = float(p[4]) / 1000.0
                    amp_in  = int(p[5])
                    amp_out = int(p[6])
                except ValueError as e:
                    print(f"  ! parse error: {e}")
                    continue

                flags = p[8] if len(p) > 8 else "0x00"
                mph = (distance_ft / elapsed) * 0.681818 if (distance_ft and elapsed > 0) else 0

                notes = []
                try:
                    fv = int(flags, 16)
                except ValueError:
                    fv = 0
                if fv & 0x02: notes.append("delayed")
                if fv & 0x08: notes.append("unconfirmed")
                if fv & 0x01: notes.append("no-sync")

                rows.append([datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                             lap_no, f"{elapsed:.4f}",
                             distance_ft if distance_ft else "",
                             f"{mph:.2f}" if distance_ft else "",
                             driver, config, session, discipline, cones,
                             amp_in, amp_out, flags, "ok", " ".join(notes)])
                write_all()

                speed = f"{mph:7.2f} mph" if distance_ft else " " * 11
                print(f"  lap {lap_no:3d}   {elapsed:8.4f} s  {speed}"
                      f"   cones {cones}"
                      + (f"   [{' '.join(notes)}]" if notes else ""))

            elif "gate" in raw and "seq" in raw:
                print(f"    {raw.lstrip('. ')}")
            elif "FAULT" in raw or raw.startswith("!"):
                print(f"  ! {raw.lstrip('! ')}")

    except KeyboardInterrupt:
        pass
    finally:
        print(f"\n  Session ended - {len(rows)} laps, {cones} cones - {day_csv}")
        ser.close()
        input("\n  Press Enter to close...")


if __name__ == "__main__":
    main()