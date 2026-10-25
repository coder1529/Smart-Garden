import math
import time

import serial
import firebase_admin
from firebase_admin import credentials, db

# ---------- Settings ----------
PORT = "COM6"          # hub HC-06 "Outgoing" port
GLOVE_PORT = "COM11"    # glove HC-06 "Outgoing" port -- CHANGE to yours
BAUD = 9600
DB_URL = "https://smart-garden-20afa-default-rtdb.asia-southeast1.firebasedatabase.app/"
KEY_FILE = "smart-garden-20afa-firebase-adminsdk-fbsvc-e4e179570b.json"
HISTORY_EVERY_S = 60     # one history point per pot per minute
POLL_EVERY_S = 2         # how often to check the website for changes
CMD_TIMEOUT_S = 3        # resend a command if the hub hasn't confirmed it
ACTION_MAX_AGE_S = 120   # ignore Water now / Stop presses older than this
GLOVE_RETRY_S = 30       # how often to retry connecting to the glove

WATER = {0: "idle", 1: "watering", 2: "soaking"}
LIGHT_MODES = {"auto": 0, "on": 1, "off": 2}


def ms(v):
    return int(float(v) * 1000)


# website name -> (key number on the pot, converter)
SETTINGS = {
    "autoWater":      (1, lambda v: 1 if v else 0),
    "dryPct":         (2, int),
    "wetPct":         (3, int),
    "pulseSec":       (4, ms),
    "soakSec":        (5, ms),
    "checkSec":       (6, ms),
    "maxPulses":      (7, int),
    "lightMode":      (8, lambda v: LIGHT_MODES.get(v, 0)),
    "lightOnPct":     (9, int),
    "lightOffPct":    (10, int),
    "brightness":     (11, int),
    "color":          (12, lambda v: int(str(v).lstrip("#"), 16)),
    "soilRawDry":     (13, int),
    "soilRawWet":     (14, int),
    "lightRawDark":   (15, int),
    "lightRawBright": (16, int),
}
ACTIONS = {"waterNow": 100, "stopWater": 101}

firebase_admin.initialize_app(credentials.Certificate(KEY_FILE), {"databaseURL": DB_URL})


def as_dict(v):
    # Firebase returns a list instead of a dict when keys are 1, 2, 3...
    if v is None:
        return {}
    if isinstance(v, list):
        return {str(i): x for i, x in enumerate(v) if x is not None}
    return {str(k): x for k, x in v.items()}


def num(s):
    v = float(s)   # raises on garbage like "ovf"
    return None if math.isnan(v) else v


def parse_status(parts):
    """Turn a D line into a dict, or return None if it looks corrupted."""
    try:
        f = {
            "pot": int(parts[1]), "temp": num(parts[2]), "hum": num(parts[3]),
            "soil": int(parts[4]), "light": int(parts[5]), "water": int(parts[6]),
            "grow": int(parts[7]), "seq": int(parts[8]), "rawSoil": int(parts[9]),
            "rawLight": int(parts[10]), "pulses": int(parts[11]), "autoWater": int(parts[12]),
        }
    except ValueError:
        return None
    ok = (1 <= f["pot"] <= 99
          and 0 <= f["soil"] <= 100 and 0 <= f["light"] <= 100
          and f["water"] in (0, 1, 2) and f["grow"] in (0, 1) and f["autoWater"] in (0, 1)
          and 0 <= f["rawSoil"] <= 1023 and 0 <= f["rawLight"] <= 1023
          and 0 <= f["pulses"] <= 50
          and (f["temp"] is None or -20 <= f["temp"] <= 60)
          and (f["hum"] is None or 0 <= f["hum"] <= 100))
    return f if ok else None


last_history = {}
last_seq = {}       # pot -> last status seq (to detect pot restarts)
last_grow = {}      # pot -> grow light on/off (for the glove's toggle)
desired = {}        # pot -> settings from the website
actions = {}        # pot -> queued actions from the website
inflight = {}       # pot -> command sent to the hub, waiting for confirmation
glove_jobs = {}     # pot -> list of (name, key, value) from the glove
applied = {p: as_dict(v) for p, v in as_dict(db.reference("applied").get()).items()}
next_seq = int(time.time()) & 0xFFFF


# ---------- Glove (local only, no Firebase) ----------
glove = None
glove_buf = b""
glove_next_try = 0


def open_glove():
    global glove, glove_next_try
    if glove or time.time() < glove_next_try:
        return
    glove_next_try = time.time() + GLOVE_RETRY_S
    try:
        glove = serial.Serial(GLOVE_PORT, BAUD, timeout=0)
        print("Glove connected on", GLOVE_PORT)
    except serial.SerialException as e:
        print("Glove not available:", e)


def glove_command(kind, pot):
    pot = str(pot)
    if pot not in last_seq:
        print(f"[Glove] pot {pot} isn't online, ignoring")
        return
    if kind == "P":
        print(f"[Glove] selected pot {pot}")
        return
    q = glove_jobs.setdefault(pot, [])
    if kind == "W":
        q.append(("waterNow", 100, 0, time.time()))
        print(f"[Glove] water pot {pot}")
    elif kind == "X":
        q[:] = [j for j in q if j[0] != "waterNow"]   # Stop cancels a queued water
        q.insert(0, ("stopWater", 101, 0, time.time()))
        print(f"[Glove] stop pot {pot}")
    elif kind == "T":
        on = last_grow.get(pot, False)
        q.append(("lightOff" if on else "lightOn", 8, 2 if on else 1, time.time()))
        print(f"[Glove] light {'OFF' if on else 'ON'} for pot {pot}")


def read_glove():
    global glove, glove_buf
    if not glove:
        return
    try:
        data = glove.read(glove.in_waiting or 0)
    except serial.SerialException as e:
        print("[Glove] disconnected:", e)
        try:
            glove.close()
        except Exception:
            pass
        glove = None
        return
    if not data:
        return
    glove_buf += data
    while b"\n" in glove_buf:
        raw, glove_buf = glove_buf.split(b"\n", 1)
        line = raw.decode(errors="ignore").strip()
        if not line:
            continue
        print(f"[Glove] received: {line}")           # every line the glove sends
        parts = line.split(",")
        if len(parts) != 2 or parts[0] not in ("P", "W", "X", "T") or not parts[1].isdigit():
            print(f"[Glove] not a valid command, ignoring")
            continue
        glove_command(parts[0], int(parts[1]))
    if len(glove_buf) > 100:                         # junk with no line ending
        print(f"[Glove] dropping garbage: {glove_buf[:30]!r}...")
        glove_buf = b""


# ---------- Hub ----------
def handle(line):
    """Process one line from the hub. Returns the pot id for status lines."""
    parts = line.split(",")
    now_ms = int(time.time() * 1000)

    if parts[0] == "D" and len(parts) == 13:
        f = parse_status(parts)
        if f is None:
            print("Ignored corrupted line:", line)
            return None
        pot, seq = str(f["pot"]), f["seq"]

        # A real restart starts counting from 0 again
        if pot in last_seq and seq < last_seq[pot] and seq < 10:
            print(f"Pot {pot} restarted, re-sending settings")
            applied[pot] = {}
            db.reference(f"applied/{pot}").delete()
        last_seq[pot] = seq
        last_grow[pot] = f["grow"] == 1

        live = {
            "temp": f["temp"],
            "hum": f["hum"],
            "soil": f["soil"],
            "light": f["light"],
            "water": WATER[f["water"]],
            "grow": f["grow"] == 1,
            "seq": seq,
            "rawSoil": f["rawSoil"],
            "rawLight": f["rawLight"],
            "pulses": f["pulses"],
            "autoWater": f["autoWater"] == 1,
            "online": True,
            "updatedAt": {".sv": "timestamp"},   # Firebase server time, not the PC clock
        }
        db.reference(f"live/{pot}").set(live)

        if time.time() - last_history.get(pot, 0) >= HISTORY_EVERY_S:
            last_history[pot] = time.time()
            db.reference(f"history/{pot}").push({
                "t": now_ms,
                "temp": live["temp"],
                "hum": live["hum"],
                "soil": live["soil"],
                "light": live["light"],
            })
        return pot

    elif parts[0] == "A" and len(parts) == 3:
        pot, seq = str(int(parts[1])), int(parts[2])
        job = inflight.get(pot)
        if job and job["seq"] == seq:
            del inflight[pot]
            if job["kind"] == "setting":
                applied.setdefault(pot, {})[job["name"]] = job["value"]
                db.reference(f"applied/{pot}/{job['name']}").set(job["value"])
                print(f"Pot {pot}: {job['name']} = {job['value']} applied")
            elif job["kind"] == "glove":
                q = glove_jobs.get(pot, [])
                for i, j in enumerate(q):
                    if j[0] == job["name"]:
                        q.pop(i)
                        break
                print(f"Pot {pot}: glove {job['name']} done")
            else:
                actions.get(pot, {}).pop(job["push_id"], None)
                db.reference(f"commands/{pot}/{job['push_id']}").delete()
                print(f"Pot {pot}: {job['name']} done")

    elif parts[0] == "L" and len(parts) == 2:
        db.reference(f"live/{int(parts[1])}/online").set(False)

    return None


def poll():
    global desired, actions
    desired = {p: as_dict(v) for p, v in as_dict(db.reference("config").get()).items()}
    actions = {p: as_dict(v) for p, v in as_dict(db.reference("commands").get()).items()}


def make_job(pot, key, arg, **extra):
    global next_seq
    next_seq = (next_seq + 1) & 0xFFFF
    return {"seq": next_seq, "line": f"C,{pot},{next_seq},{key},{arg}\n",
            "sent_at": time.time(), **extra}


def drop_action(pot, push_id, why):
    name = (actions.get(pot, {}).get(push_id) or {}).get("action")
    print(f"Pot {pot}: dropping {name} ({why})")
    actions.get(pot, {}).pop(push_id, None)
    db.reference(f"commands/{pot}/{push_id}").delete()


def has_stop(pot):
    return any((a or {}).get("action") == "stopWater" for a in actions.get(pot, {}).values())


def next_job(pot):
    now_ms = int(time.time() * 1000)
    pend = actions.get(pot, {})

    # A Stop cancels any Water now pressed before it
    stops = [k for k, a in pend.items() if (a or {}).get("action") == "stopWater"]
    if stops:
        last_stop = max(stops)   # push ids sort in time order
        for k in [k for k, a in pend.items()
                  if (a or {}).get("action") == "waterNow" and k < last_stop]:
            drop_action(pot, k, "cancelled by Stop")

    # Actions: Stop first, then the rest oldest first
    order = sorted(pend, key=lambda k: ((pend[k] or {}).get("action") != "stopWater", k))
    for push_id in order:
        item = pend.get(push_id) or {}
        name, t = item.get("action"), item.get("t")
        if name not in ACTIONS:
            drop_action(pot, push_id, "unknown")
            continue
        if not isinstance(t, (int, float)) or now_ms - t > ACTION_MAX_AGE_S * 1000:
            drop_action(pot, push_id, "too old")
            continue
        return make_job(pot, ACTIONS[name], 0, kind="action", name=name, push_id=push_id)

    # Then any setting that differs from what the pot has confirmed
    done = applied.setdefault(pot, {})
    for name, value in desired.get(pot, {}).items():
        if name not in SETTINGS or done.get(name) == value:
            continue
        key, conv = SETTINGS[name]
        try:
            v = conv(value)
        except (ValueError, TypeError):
            print(f"Pot {pot}: bad value for {name}: {value!r}, skipping")
            done[name] = value
            continue
        return make_job(pot, key, v, kind="setting", name=name, value=value)
    return None


def send_commands(ser, pot):
    """Called right after the pot reports, while the hub is quiet and can listen."""
    now = time.time()
    job = inflight.get(pot)

    # Glove commands go first; a glove Stop beats everything
    q = glove_jobs.get(pot)
    if q:
        q[:] = [j for j in q if time.time() - j[3] < 30]   # drop glove commands older than 30 s
    if q:
        name, key, val, _ = q[0]
        if not job or job["kind"] != "glove" or (name == "stopWater" and job["name"] != "stopWater"):
            job = make_job(pot, key, val, kind="glove", name=name)
            ser.write(job["line"].encode())
            inflight[pot] = job
            print("Sent (glove):", job["line"].strip())
            return

    # Website Stop jumps the queue
    if job and job.get("name") != "stopWater" and has_stop(pot):
        print(f"Pot {pot}: Stop pressed, sending it first")
        inflight.pop(pot)
        job = None

    if job:
        if now - job["sent_at"] >= CMD_TIMEOUT_S:
            ser.write(job["line"].encode())
            job["sent_at"] = now
            print("Resent:", job["line"].strip())
        return

    job = next_job(pot)
    if job:
        ser.write(job["line"].encode())
        inflight[pot] = job
        print("Sent:", job["line"].strip())


def main():
    last_poll = 0
    while True:
        try:
            with serial.Serial(PORT, BAUD, timeout=0.2) as ser:
                print("Connected to", PORT)
                inflight.clear()
                buf = b""
                while True:
                    open_glove()
                    read_glove()

                    if time.time() - last_poll >= POLL_EVERY_S:
                        last_poll = time.time()
                        try:
                            poll()
                        except Exception as e:
                            print("Poll failed:", e)

                    chunk = ser.read(ser.in_waiting or 1)
                    if not chunk:
                        continue
                    buf += chunk
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        line = raw.decode(errors="ignore").strip()
                        if not line:
                            continue
                        print(line)
                        try:
                            pot = handle(line)
                        except Exception as e:
                            print("Skipped line:", line, "-", e)
                            continue
                        if pot:
                            try:
                                send_commands(ser, pot)
                            except serial.SerialException:
                                raise
                            except Exception as e:
                                print("Send failed:", e)
        except serial.SerialException as e:
            print("Serial error:", e, "- retrying in 5 s")
            time.sleep(5)


if __name__ == "__main__":
    main()