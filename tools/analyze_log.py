#!/usr/bin/env python3
"""
PARKDRONE flight-log analyzer.

Reads an ArduPilot dataflash log (.bin) and reports the things that explain a
bad first flight: flight mode timeline, GPS/EKF health, compass health,
vibration, and — most importantly — commanded vs. actual attitude, which tells
us whether the drone ignored the sticks (setup fault) or faithfully obeyed them
(practice/wind).

Usage:
    pip install pymavlink
    python analyze_log.py ../logs/your_flight.bin

Drop the .bin in D:\\drone_project\\logs\\ first (Mission Planner ->
DataFlash Logs -> Download via Mavlink).
"""
import sys

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("pymavlink not installed. Run:  pip install pymavlink")

# ArduPilot Copter flight-mode numbers -> names
COPTER_MODES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
    5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT",
    13: "SPORT", 14: "FLIP", 15: "AUTOTUNE", 16: "POSHOLD",
    17: "BRAKE", 18: "THROW", 20: "GUIDED_NOGPS", 21: "SMART_RTL",
}


def fmt_t(us):
    return f"{us / 1e6:8.1f}s"


def main(path):
    print(f"Reading {path} ...\n")
    mlog = mavutil.mavlink_connection(path)

    modes = []                      # (time_us, mode_name)
    gps = {"min_sats": None, "max_hdop": 0.0, "best_status": 0}
    vibe = {"max": [0.0, 0.0, 0.0], "clip": [0, 0, 0]}
    mag_seen = False
    att_err = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "n": 0}
    bat = {"min_v": None}
    errors = []                     # (time_us, subsystem, code)
    rc_seen = False

    while True:
        m = mlog.recv_match()
        if m is None:
            break
        t = m.get_type()
        d = m.to_dict()

        if t == "MODE":
            num = d.get("Mode", d.get("ModeNum"))
            name = COPTER_MODES.get(num, f"MODE_{num}")
            ts = d.get("TimeUS", 0)
            if not modes or modes[-1][1] != name:
                modes.append((ts, name))

        elif t == "GPS":
            status = d.get("Status", 0)
            nsats = d.get("NSats", 0)
            hdop = d.get("HDop", 0.0)
            gps["best_status"] = max(gps["best_status"], status)
            if gps["min_sats"] is None or nsats < gps["min_sats"]:
                gps["min_sats"] = nsats
            gps["max_hdop"] = max(gps["max_hdop"], hdop)

        elif t == "VIBE":
            for i, k in enumerate(("VibeX", "VibeY", "VibeZ")):
                vibe["max"][i] = max(vibe["max"][i], d.get(k, 0.0))
            for i, k in enumerate(("Clip0", "Clip1", "Clip2")):
                vibe["clip"][i] = max(vibe["clip"][i], d.get(k, 0))

        elif t == "MAG":
            mag_seen = True

        elif t == "ATT":
            # commanded (Des*) vs actual — large gap = drone fighting/ignoring you
            dr = abs(d.get("DesRoll", 0.0) - d.get("Roll", 0.0))
            dp = abs(d.get("DesPitch", 0.0) - d.get("Pitch", 0.0))
            att_err["roll"] = max(att_err["roll"], dr)
            att_err["pitch"] = max(att_err["pitch"], dp)
            att_err["n"] += 1

        elif t == "RCIN":
            rc_seen = True

        elif t in ("BAT", "BATT", "CURR"):
            v = d.get("Volt", d.get("Voltage"))
            if v is not None and (bat["min_v"] is None or v < bat["min_v"]):
                bat["min_v"] = v

        elif t == "ERR":
            errors.append((d.get("TimeUS", 0), d.get("Subsys"), d.get("ECode")))

    # ---- report ----
    print("=" * 60)
    print("FLIGHT MODE TIMELINE")
    print("=" * 60)
    if modes:
        for ts, name in modes:
            print(f"  {fmt_t(ts)}  ->  {name}")
        if any(n in ("STABILIZE", "ACRO") for _, n in modes):
            print("  NOTE: flew in a MANUAL mode (Stabilize/Acro) — no position")
            print("        hold. Steady drift here is expected; try AltHold/Loiter.")
    else:
        print("  no MODE messages found")

    print("\n" + "=" * 60)
    print("GPS")
    print("=" * 60)
    st = {0: "NO_GPS", 1: "NO_FIX", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK_F", 6: "RTK_FX"}
    print(f"  best fix: {st.get(gps['best_status'], gps['best_status'])}"
          f"   min sats: {gps['min_sats']}   max HDOP: {gps['max_hdop']:.2f}")
    if gps["best_status"] < 3:
        print("  WARN: never got a 3D fix — GPS modes (Loiter/RTL) unsafe.")
    if gps["max_hdop"] > 2.0:
        print("  WARN: HDOP > 2.0 — weak GPS geometry.")

    print("\n" + "=" * 60)
    print("COMPASS")
    print("=" * 60)
    print(f"  MAG data present: {mag_seen}")
    print("  (toilet-bowling / circling flyaways come from bad compass cal.)")

    print("\n" + "=" * 60)
    print("VIBRATION")
    print("=" * 60)
    print(f"  max VibeXYZ: {vibe['max'][0]:.1f} {vibe['max'][1]:.1f} "
          f"{vibe['max'][2]:.1f}  (keep < 30, ideally < 15)")
    print(f"  clip counts: {vibe['clip']}  (should stay 0)")
    if max(vibe["max"]) > 30 or any(c > 0 for c in vibe["clip"]):
        print("  WARN: high vibration — soft-mount the FC, balance props.")

    print("\n" + "=" * 60)
    print("ATTITUDE TRACKING  (commanded vs actual)")
    print("=" * 60)
    if att_err["n"]:
        print(f"  max roll error:  {att_err['roll']:.1f} deg")
        print(f"  max pitch error: {att_err['pitch']:.1f} deg")
        if max(att_err["roll"], att_err["pitch"]) < 5:
            print("  -> Drone tracked your commands well. The drift was likely")
            print("     wind + manual mode + piloting, NOT the flight controller.")
        else:
            print("  -> Big gap between commanded and actual attitude. The FC")
            print("     was fighting itself — suspect tuning / level cal / mechanics.")
    else:
        print("  no ATT messages found")

    print("\n" + "=" * 60)
    print("BATTERY / ERRORS")
    print("=" * 60)
    print(f"  min voltage: {bat['min_v']}")
    print(f"  RC input logged: {rc_seen}")
    if errors:
        print("  ERR events:")
        for ts, sub, code in errors:
            print(f"    {fmt_t(ts)}  subsys={sub} code={code}")
    else:
        print("  no ERR events")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("usage: python analyze_log.py <log.bin>")
    main(sys.argv[1])
