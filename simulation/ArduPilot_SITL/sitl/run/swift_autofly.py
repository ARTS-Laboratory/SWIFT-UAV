#!/usr/bin/env python3
"""
swift_autofly.py - fly one SWIFT SITL session with no MAVProxy typing.

Started by swift_run.m inside Ubuntu (WSL), next to sim_vehicle.py. It talks to
ArduPlane through an extra MAVProxy output (udp 127.0.0.1:14551) and does what
the session README used to have you type:

    1  wait for ArduPlane (up to 3 min while sim_vehicle.py boots)
    2  upload the .waypoints file                  (was: wp load ...)
    3  read back every parameter in the .param files and compare
                                                   (was: param show ...)
    4  wait for GPS 3D fix + EKF absolute position (was: "EKF3 IMU0 is using GPS")
    5  mode FBWA -> arm -> mode AUTO               (was: typed in MAVProxy)
    6  follow the flight: mission item, status text, a progress line every 30 s
    7  after touchdown and auto-disarm, write the "done" file that tells
       MATLAB to stop the model

Everything it prints also goes to the log file, which MATLAB echoes into the
Command Window while the flight runs.

Commands from MATLAB (swift_run rtl / swift_run stop) arrive as a one-line
text file (--cmd): RTL switches ArduPlane to RTL (RTL_AUTOLAND lands it),
STOP ends this script and writes the done file.

Ryan Bilock, ARTS Lab | v0.1 (25 Nov 2026)
"""
import argparse
import json
import math
import os
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    print("pymavlink is missing. In Ubuntu:  pip3 install --user pymavlink", flush=True)
    sys.exit(2)

ML = mavutil.mavlink

# ArduPlane custom modes
MODES = {"MANUAL": 0, "CIRCLE": 1, "STABILIZE": 2, "FBWA": 5, "FBWB": 6,
         "CRUISE": 7, "AUTO": 10, "RTL": 11, "LOITER": 12, "TAKEOFF": 13}
MODE_NAME = {v: k for k, v in MODES.items()}

GLOBAL_FRAMES = {0, 3, 5, 6, 10, 11}   # x, y are lat, lon -> scaled by 1e7 for MISSION_ITEM_INT

MY_SYSID = 250                          # not 255, so replies to MAVProxy / Mission Planner are ignored
MY_COMPID = 191

LOG = None


def say(msg):
    line = "%s  %s" % (time.strftime("%H:%M:%S"), msg)
    print(line, flush=True)
    if LOG:
        LOG.write(line + "\n")
        LOG.flush()


# --------------------------------------------------------------------------- files
def read_params(files):
    """name -> value, later files win (same rule as --add-param-file)."""
    vals, src = {}, {}
    for f in files:
        with open(f) as fh:
            for raw in fh:
                line = raw.split("#", 1)[0].strip()
                if not line:
                    continue
                parts = line.replace(",", " ").split()
                if len(parts) < 2:
                    continue
                try:
                    vals[parts[0].upper()] = float(parts[1])
                    src[parts[0].upper()] = os.path.basename(f)
                except ValueError:
                    pass
    return vals, src


def read_waypoints(path):
    with open(path) as fh:
        lines = [l.strip() for l in fh if l.strip()]
    if not lines or not lines[0].startswith("QGC WPL"):
        raise SystemExit("%s is not a QGC WPL waypoint file" % path)
    items = []
    for l in lines[1:]:
        f = l.split()
        if len(f) < 12:
            raise SystemExit("bad waypoint line: %s" % l)
        items.append(dict(seq=int(f[0]), current=int(f[1]), frame=int(f[2]), cmd=int(f[3]),
                          p=[float(v) for v in f[4:8]],
                          x=float(f[8]), y=float(f[9]), z=float(f[10]),
                          auto=int(float(f[11]))))
    for k, it in enumerate(items):
        if it["seq"] != k:
            raise SystemExit("waypoint file numbering jumps at line %d" % (k + 2))
    return items


# --------------------------------------------------------------------------- link
class Vehicle:
    def __init__(self, conn):
        self.m = mavutil.mavlink_connection(conn, source_system=MY_SYSID,
                                            source_component=MY_COMPID, dialect="ardupilotmega")
        self.ts = 1
        self.tc = 1
        self.armed = False
        self.mode = None
        self.last_hb = 0.0
        self.mission_seq = -1
        self.alt = 0.0           # m above home (GLOBAL_POSITION_INT.relative_alt)
        self.aspd = 0.0          # m/s
        self.thr = 0
        self.fix = 0
        self.ekf_flags = 0
        self.batt_v = float("nan")
        self.batt_mah = float("nan")
        self.texts = []          # STATUSTEXT seen since last pump

    def wait_vehicle(self, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if msg is None:
                continue
            if msg.type == ML.MAV_TYPE_GCS or msg.autopilot != ML.MAV_AUTOPILOT_ARDUPILOTMEGA:
                continue
            self.ts, self.tc = msg.get_srcSystem(), msg.get_srcComponent()
            self._hb(msg)
            return True
        return False

    def _hb(self, msg):
        self.last_hb = time.time()
        self.armed = bool(msg.base_mode & ML.MAV_MODE_FLAG_SAFETY_ARMED)
        self.mode = msg.custom_mode

    def handle(self, msg):
        t = msg.get_type()
        if msg.get_srcSystem() != self.ts:
            return
        if t == "HEARTBEAT":
            if msg.type != ML.MAV_TYPE_GCS:
                self._hb(msg)
        elif t == "MISSION_CURRENT":
            self.mission_seq = msg.seq
        elif t == "GLOBAL_POSITION_INT":
            self.alt = msg.relative_alt / 1000.0
        elif t == "VFR_HUD":
            self.aspd = msg.airspeed
            self.thr = msg.throttle
        elif t == "GPS_RAW_INT":
            self.fix = msg.fix_type
        elif t == "EKF_STATUS_REPORT":
            self.ekf_flags = msg.flags
        elif t == "SYS_STATUS":
            if msg.voltage_battery != 65535:
                self.batt_v = msg.voltage_battery / 1000.0
        elif t == "BATTERY_STATUS":
            if msg.current_consumed >= 0:
                self.batt_mah = float(msg.current_consumed)
        elif t == "STATUSTEXT":
            self.texts.append(msg.text)

    def pump(self, dt=0.2):
        """Read everything for dt seconds."""
        t_end = time.time() + dt
        while True:
            left = t_end - time.time()
            if left <= 0:
                break
            msg = self.m.recv_match(blocking=True, timeout=left)
            if msg is not None:
                self.handle(msg)

    def recv(self, types, timeout, cond=None):
        """Wait for one of `types` from the vehicle (other messages still update state)."""
        t_end = time.time() + timeout
        while time.time() < t_end:
            msg = self.m.recv_match(blocking=True, timeout=max(0.05, t_end - time.time()))
            if msg is None:
                continue
            self.handle(msg)
            if msg.get_type() in types and msg.get_srcSystem() == self.ts:
                if cond is None or cond(msg):
                    return msg
        return None

    # ---- mission
    def upload(self, items, timeout=60):
        n = len(items)
        mine = lambda msg: getattr(msg, "target_system", MY_SYSID) == MY_SYSID
        for attempt in range(3):
            self.m.mav.mission_count_send(self.ts, self.tc, n)
            sent = set()
            t_last = time.time()
            while time.time() - t_last < 8:
                msg = self.recv(("MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"), 1.0, mine)
                if msg is None:
                    continue
                t_last = time.time()
                if msg.get_type() == "MISSION_ACK":
                    if msg.type == ML.MAV_MISSION_ACCEPTED and len(sent) == n:
                        return True
                    if msg.type != ML.MAV_MISSION_ACCEPTED:
                        say("  ArduPlane rejected the mission (MAV_MISSION_RESULT %d)" % msg.type)
                        break
                    continue
                s = msg.seq
                if s >= n:
                    continue
                it = items[s]
                if it["frame"] in GLOBAL_FRAMES:
                    x, y = int(round(it["x"] * 1e7)), int(round(it["y"] * 1e7))
                else:
                    x, y = int(round(it["x"])), int(round(it["y"]))
                self.m.mav.mission_item_int_send(self.ts, self.tc, s, it["frame"], it["cmd"],
                                                 it["current"], it["auto"], *it["p"], x, y, it["z"])
                sent.add(s)
            say("  upload attempt %d did not finish, retrying" % (attempt + 1))
        return False

    def mission_count(self, timeout=5):
        mine = lambda msg: getattr(msg, "target_system", MY_SYSID) == MY_SYSID
        self.m.mav.mission_request_list_send(self.ts, self.tc)
        msg = self.recv(("MISSION_COUNT",), timeout, mine)
        if msg is not None:
            # end the download politely so ArduPlane does not keep waiting
            self.m.mav.mission_ack_send(self.ts, self.tc, ML.MAV_MISSION_ACCEPTED)
            return msg.count
        return None

    # ---- parameters
    def param(self, name, timeout=2.0):
        for _ in range(3):
            self.m.mav.param_request_read_send(self.ts, self.tc, name.encode(), -1)
            msg = self.recv(("PARAM_VALUE",), timeout,
                            lambda p: p.param_id.rstrip("\x00").upper() == name)
            if msg is not None:
                return msg.param_value
        return None

    # ---- modes and arming
    def set_mode(self, name, timeout=10):
        want = MODES[name]
        t_end = time.time() + timeout
        while time.time() < t_end:
            self.m.mav.set_mode_send(self.ts, ML.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, want)
            self.pump(1.0)
            if self.mode == want:
                return True
        return False

    def arm(self, timeout):
        t_end = time.time() + timeout
        last_reason = ""
        while time.time() < t_end:
            self.texts = []
            self.m.mav.command_long_send(self.ts, self.tc, ML.MAV_CMD_COMPONENT_ARM_DISARM,
                                         0, 1, 0, 0, 0, 0, 0, 0)
            self.pump(2.0)
            if self.armed:
                return True
            why = [t for t in self.texts if "arm" in t.lower() or "prearm" in t.lower()]
            if why and why[-1] != last_reason:
                last_reason = why[-1]
                say("  waiting to arm: %s" % last_reason)
            self.pump(1.0)
        return False


# --------------------------------------------------------------------------- main
def write_done(path, info):
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(info, fh, indent=1)
    os.replace(tmp, path)


def read_cmd(path):
    if not path or not os.path.exists(path):
        return None
    try:
        with open(path) as fh:
            c = fh.read().strip().upper()
        os.remove(path)
        return c
    except OSError:
        return None


def main():
    global LOG
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--conn", default="udpin:127.0.0.1:14551")
    ap.add_argument("--wp", required=True, help="QGC WPL .waypoints file")
    ap.add_argument("--params", nargs="*", default=[], help=".param files, in load order")
    ap.add_argument("--done", required=True, help="file to write when the flight is over")
    ap.add_argument("--log", default="", help="copy of everything printed")
    ap.add_argument("--cmd", default="", help="command file from MATLAB (RTL / STOP)")
    ap.add_argument("--boot-timeout", type=float, default=180)
    ap.add_argument("--arm-timeout", type=float, default=180)
    ap.add_argument("--max-min", type=float, default=90, help="minutes in the air before a warning")
    ap.add_argument("--no-fly", action="store_true", help="upload + check only, do not arm")
    a = ap.parse_args()

    if a.log:
        LOG = open(a.log, "a")
    info = dict(status="started", t_start=time.time(), mission_items=0, param_checked=0,
                param_mismatch=[], param_missing=[], max_item=-1, max_alt_m=0.0,
                t_armed=None, t_takeoff=None, t_disarmed=None, batt_mAh=None)

    def finish(status, code=0):
        info["status"] = status
        info["t_end"] = time.time()
        write_done(a.done, info)
        say("DONE: %s" % status)
        sys.exit(code)

    items = read_waypoints(a.wp)
    pvals, psrc = read_params(a.params) if a.params else ({}, {})
    say("SWIFT autofly  |  %d mission items  |  %d parameters to check" % (len(items), len(pvals)))

    v = Vehicle(a.conn)
    say("1/6  waiting for ArduPlane (sim_vehicle.py is booting) ...")
    if not v.wait_vehicle(a.boot_timeout):
        finish("no ArduPlane heartbeat in %.0f s - did sim_vehicle.py start?" % a.boot_timeout, 1)
    say("     ArduPlane is up (system %d), mode %s" % (v.ts, MODE_NAME.get(v.mode, v.mode)))
    v.pump(3.0)   # let parameters settle after boot

    # ---- mission
    say("2/6  uploading %s" % os.path.basename(a.wp))
    if not v.upload(items):
        finish("mission upload failed", 1)
    n = v.mission_count()
    info["mission_items"] = n if n is not None else len(items)
    if n is not None and n != len(items):
        say("     [WARN] ArduPlane reports %d items, file has %d" % (n, len(items)))
    else:
        say("     [ OK ] %d items on board (incl. home)" % len(items))

    # ---- parameters
    say("3/6  checking %d parameters against the .param files" % len(pvals))
    for name, want in pvals.items():
        got = v.param(name)
        info["param_checked"] += 1
        if got is None:
            info["param_missing"].append(name)
            say("     [WARN] %-18s not found in this ArduPlane build (%s)" % (name, psrc[name]))
        elif abs(got - want) > 1e-3 * max(1.0, abs(want)):
            info["param_mismatch"].append([name, want, got])
            say("     [FAIL] %-18s is %g, %s wants %g" % (name, got, psrc[name], want))
    bad = len(info["param_mismatch"])
    if bad == 0:
        say("     [ OK ] all %d match (%d not in this build)" % (len(pvals) - len(info["param_missing"]),
                                                              len(info["param_missing"])))
    else:
        finish("%d parameter(s) do not match - not flying. Was -w (wipe) off?" % bad, 1)

    if a.no_fly:
        finish("checked only (--no-fly)")

    # ---- ready to arm
    say("4/6  waiting for GPS 3D fix and EKF absolute position ...")
    t0 = time.time()
    while time.time() - t0 < a.arm_timeout:
        v.pump(1.0)
        for t in v.texts:
            if "using gps" in t.lower():
                say("     %s" % t)
        v.texts = []
        if v.fix >= 3 and (v.ekf_flags & ML.EKF_POS_HORIZ_ABS):
            break
    else:
        say("     [WARN] no EKF position report - trying to arm anyway")
    say("     GPS fix %d, EKF flags 0x%03x" % (v.fix, v.ekf_flags))

    # ---- FBWA, arm, AUTO
    say("5/6  mode FBWA -> arm -> mode AUTO")
    if not v.set_mode("FBWA"):
        finish("ArduPlane would not go to FBWA", 1)
    if not v.arm(a.arm_timeout):
        finish("ArduPlane would not arm in %.0f s (see the waiting-to-arm lines)" % a.arm_timeout, 1)
    info["t_armed"] = time.time()
    say("     ARMED")
    if not v.set_mode("AUTO"):
        finish("ArduPlane would not go to AUTO", 1)
    say("     AUTO - taking off")

    # ---- follow the flight
    say("6/6  flying. Abort from MATLAB:  swift_run rtl   (lands)   |   swift_run stop")
    last_seq, last_line, warned = -1, time.time(), False
    airborne = False
    while True:
        v.pump(0.5)
        for t in v.texts:
            say("     AP: %s" % t)
        v.texts = []
        now = time.time()

        if v.mission_seq != last_seq and v.mission_seq >= 0:
            last_seq = v.mission_seq
            info["max_item"] = max(info["max_item"], last_seq)
            say("     item %d / %d" % (last_seq, len(items) - 1))
        info["max_alt_m"] = max(info["max_alt_m"], v.alt)
        if not airborne and v.alt > 5 and v.aspd > 10:
            airborne = True
            info["t_takeoff"] = now
            say("     airborne")
        if now - last_line > 30:
            last_line = now
            mins = (now - info["t_armed"]) / 60
            say("     %5.1f min  alt %5.0f m  airspeed %4.1f m/s  thr %3d %%  batt %s V  %s" % (
                mins, v.alt, v.aspd, v.thr,
                "%.1f" % v.batt_v if not math.isnan(v.batt_v) else "-",
                MODE_NAME.get(v.mode, v.mode)))
            if not warned and mins > a.max_min:
                warned = True
                say("     [WARN] in the air %.0f min, longer than this session should take" % mins)
        if not math.isnan(v.batt_mah):
            info["batt_mAh"] = v.batt_mah

        c = read_cmd(a.cmd)
        if c == "RTL":
            say("     MATLAB asked for RTL")
            v.set_mode("RTL")
        elif c == "STOP":
            finish("stopped from MATLAB")

        if now - v.last_hb > 20:
            finish("lost ArduPlane heartbeat for 20 s (did the Simulink model stop?)", 1)
        if not v.armed:
            info["t_disarmed"] = now
            if airborne:
                fl = (now - info["t_takeoff"]) / 60
                say("     DISARMED after landing (%.1f min in the air, top item %d)" % (fl, info["max_item"]))
                finish("landed")
            finish("disarmed before takeoff", 1)


if __name__ == "__main__":
    main()
