#!/usr/bin/env python3
"""Sentec DO BlueOS extension backend.

Reads a Sentec OXYnor dissolved-oxygen sensor over Modbus RTU through a
BLUART USB-to-RS485 adapter, exposes a Flask dashboard / widget / data API,
logs measurements to CSV, and forwards DO + TDO to MAVLink2Rest so they end
up in the autopilot's .BIN log.
"""
import csv
import glob
import json
import os
import struct
import threading
import time
from datetime import datetime, timedelta
from pathlib import Path
from threading import Lock

import requests
import serial
from flask import Flask, jsonify, request, send_file, send_from_directory

app = Flask(__name__)

# ---------------------------------------------------------------------------
# Serial / Modbus configuration
# ---------------------------------------------------------------------------
DEFAULT_SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 19200            # OXYnor Modbus default
STOP_BITS = serial.STOPBITS_TWO  # 8N2 per OXYnor default
PARITY = serial.PARITY_NONE
MODBUS_SLAVE_ID = 1
POLL_INTERVAL_S = 5.0

# ---------------------------------------------------------------------------
# MAVLink2Rest / NAMED_VALUE_FLOAT addressing
# ---------------------------------------------------------------------------
# mavlink2rest keys its in-memory store by system_id/component_id/message_type,
# so every NAMED_VALUE_FLOAT sent from the SAME (system, component) pair lands in
# one slot and the last write wins (only one metric survives for the inspector
# and the autopilot .BIN log). Each metric therefore needs its OWN component_id.
# component_id 0 is MAV_COMP_ID_ALL and is an invalid *source* component, so we
# also avoid it. Base 70 stays clear of the BlueOS PH/TEMP/SAL/COND extension
# (25-28) and the Mikrotik-Monitor range (60-66).
MAVLINK_SYSTEM_ID = 255
MAVLINK_COMPONENT_ID_BASE = 70
NAMED_VALUE_COMPONENTS = {
    "DO": MAVLINK_COMPONENT_ID_BASE + 0,   # 70  primary unit (ppm / mg/L)
    "TDO": MAVLINK_COMPONENT_ID_BASE + 1,  # 71  DO temperature
    "DOS": MAVLINK_COMPONENT_ID_BASE + 2,  # 72  secondary unit (% air saturation)
}

SERIAL_PORT = DEFAULT_SERIAL_PORT
SERIAL_CONFIG_FILE = "/app/logs/serial_config.json"

serial_connection = None
SERIAL_LOCK = threading.Lock()

# OXYnor oxygen unit codes (register 2089) -> human label
UNIT_CODES = {
    0x10: "% vol O2",
    0x20: "% air saturation",
    0x40: "ppb (ug/L)",
    0x80: "ppm (mg/L)",
    0x80000000: "Torr",
    0x04000000: "umol/L",
    0x20000000: "hPa",
    0x40000000: "ppm gas",
}

# Units we want the sensor to report. The OXYnor can compute two units at once:
# the primary unit lives in register 2089 (value in the 4895 block), and a
# secondary unit lives in register 6063 (value in register 6065). We log both
# so we always have:
#   - ppm (mg/L): concentration, but salinity-dependent (sensor assumes the
#     salinity in register 3115, default 0 = freshwater).
#   - % air saturation: partial-pressure based and salinity-independent, so it
#     can be re-converted to a correct mg/L in post-processing for any salinity.
# mg/L (ppm) is only valid while the sensor is in "humid" measurement mode; if
# the sensor rejects a unit it reverts, so we cap how many times we try to set
# each one (every write touches the OXYnor's flash, which has a limited cycle
# count).
DESIRED_OXYGEN_UNIT_CODE = 0x80   # primary:   ppm (mg/L)
DESIRED_SECOND_UNIT_CODE = 0x20   # secondary: % air saturation
MAX_UNIT_WRITE_ATTEMPTS = 3
# Per-slot write-attempt counters so a persistently rejected unit doesn't keep
# burning flash write cycles.
_unit_write_attempts = {"primary": 0, "secondary": 0}

# ---------------------------------------------------------------------------
# Salinity compensation (register 3115, float, PSU)
# ---------------------------------------------------------------------------
# The OXYnor applies salinity ONLY to concentration units (mg/L, ppm, ug/L,
# umol/L); % air saturation is salinity-independent. The register is stored in
# flash, so it is NEVER written automatically -- only on an explicit, confirmed
# user request via /api/salinity, and only when the value actually differs --
# to preserve the limited (~10k) flash write cycles. Whatever the user sets
# simply persists on the sensor.
SALINITY_REGISTER = 3115
SALINITY_PRESETS = {"fresh": 0.0, "salt": 35.0}
SALINITY_MIN = 0.0
SALINITY_MAX = 45.0
# A requested write within this tolerance (PSU) of the current value is a no-op,
# so re-applying the same setting can't churn the flash.
SALINITY_WRITE_EPSILON = 0.05

# ---------------------------------------------------------------------------
# Storage / logging
# ---------------------------------------------------------------------------
LOG_DIR = Path("/app/logs")
LOG_FILE = LOG_DIR / "sensor_data.csv"
CSV_HEADERS = [
    "timestamp", "temperature", "do", "do_unit",
    "do_air_saturation", "do_air_saturation_unit",
    "pressure", "phase", "error",
    "vehicle_temperature", "latitude", "longitude",
]
MAX_CSV_SIZE_MB = 10

data = []  # in-memory ring buffer (last 60)
DATA_LOCK = Lock()

# Cached oxygen units so the API/UI can label axes without a round-trip every
# read. Defaults to the units we ask the sensor to use; the real values are read
# back live.
oxygen_unit_code = DESIRED_OXYGEN_UNIT_CODE
oxygen_unit_label = UNIT_CODES[DESIRED_OXYGEN_UNIT_CODE]
second_unit_code = DESIRED_SECOND_UNIT_CODE
second_unit_label = UNIT_CODES[DESIRED_SECOND_UNIT_CODE]


# ---------------------------------------------------------------------------
# Bootstrap log directory + CSV
# ---------------------------------------------------------------------------
def ensure_csv_headers():
    """Make sure the CSV exists and has the expected columns."""
    try:
        if not LOG_FILE.exists():
            print(f"CSV does not exist yet; will be created with {CSV_HEADERS}")
            return True
        with open(LOG_FILE, "r") as f:
            existing = next(csv.reader(f), [])
        missing = [h for h in CSV_HEADERS if h not in existing]
        if missing:
            print(f"Adding missing CSV columns: {missing}")
            with open(LOG_FILE, "r") as f:
                rows = list(csv.DictReader(f))
            with open(LOG_FILE, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=CSV_HEADERS)
                writer.writeheader()
                for row in rows:
                    for h in missing:
                        row[h] = None
                    writer.writerow(row)
        return True
    except Exception as e:
        print(f"Error ensuring CSV headers: {e}")
        return False


try:
    os.makedirs(str(LOG_DIR), exist_ok=True)
    print(f"Using log directory: {LOG_DIR}")
    print(f"  exists: {LOG_DIR.exists()}, writable: {os.access(str(LOG_DIR), os.W_OK)}")
    ensure_csv_headers()
except Exception as e:
    print(f"Error preparing log directory: {e}")


# ---------------------------------------------------------------------------
# Serial port discovery + config persistence
# ---------------------------------------------------------------------------
def load_serial_config():
    global SERIAL_PORT
    try:
        if os.path.exists(SERIAL_CONFIG_FILE):
            with open(SERIAL_CONFIG_FILE, "r") as f:
                cfg = json.load(f)
            saved = cfg.get("port")
            if saved and os.path.exists(saved):
                SERIAL_PORT = saved
                print(f"Loaded serial port from config: {SERIAL_PORT}")
            else:
                print(f"Saved port {saved!r} not present; using default {DEFAULT_SERIAL_PORT}")
    except Exception as e:
        print(f"Error loading serial config: {e}")


def save_serial_config(port):
    try:
        with open(SERIAL_CONFIG_FILE, "w") as f:
            json.dump({"port": port}, f)
        print(f"Saved serial port configuration: {port}")
        return True
    except Exception as e:
        print(f"Error saving serial config: {e}")
        return False


def find_serial_ports():
    ports = []
    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        for device in glob.glob(pattern):
            ports.append({"path": device, "name": os.path.basename(device)})
    ports.sort(key=lambda p: p["path"])
    return ports


def initialize_serial_connection():
    global serial_connection
    if serial_connection and serial_connection.is_open:
        try:
            serial_connection.close()
            print("Closed existing serial connection")
        except Exception as e:
            print(f"Error closing serial connection: {e}")
    try:
        serial_connection = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=PARITY,
            stopbits=STOP_BITS,
            timeout=0.8,
        )
        print(f"Opened {SERIAL_PORT} @ {BAUD_RATE} 8N2")
        return True
    except serial.SerialException as e:
        print(f"Error opening serial port {SERIAL_PORT}: {e}")
        serial_connection = None
        return False


# ---------------------------------------------------------------------------
# Modbus RTU helpers
# ---------------------------------------------------------------------------
def crc16_modbus(data_bytes: bytes) -> bytes:
    """Modbus CRC-16 (poly 0xA001, init 0xFFFF). Returns 2 bytes LSB first."""
    crc = 0xFFFF
    for b in data_bytes:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def _read_holding(ser, address, count):
    """Issue function-3 read; return payload bytes (without slave/func/len/CRC)."""
    pdu = bytes([
        MODBUS_SLAVE_ID, 0x03,
        (address >> 8) & 0xFF, address & 0xFF,
        (count >> 8) & 0xFF, count & 0xFF,
    ])
    frame = pdu + crc16_modbus(pdu)
    ser.reset_input_buffer()
    ser.write(frame)
    # Expected response length: 1 slave + 1 func + 1 byte-count + 2*count + 2 CRC
    expected = 5 + 2 * count
    deadline = time.time() + 1.0
    buf = bytearray()
    while len(buf) < expected and time.time() < deadline:
        chunk = ser.read(expected - len(buf))
        if not chunk:
            time.sleep(0.02)
            continue
        buf.extend(chunk)
    if len(buf) < 5 or buf[0] != MODBUS_SLAVE_ID:
        raise IOError(f"short or stray Modbus response: {bytes(buf).hex()}")
    if buf[1] & 0x80:
        raise IOError(f"Modbus exception, code {buf[2] if len(buf) > 2 else '?'}")
    if buf[1] != 0x03:
        raise IOError(f"unexpected function code {buf[1]:#x}")
    byte_count = buf[2]
    if len(buf) < 3 + byte_count + 2:
        raise IOError(f"truncated payload: {bytes(buf).hex()}")
    # Validate CRC
    body = bytes(buf[: 3 + byte_count])
    crc_rx = bytes(buf[3 + byte_count: 3 + byte_count + 2])
    if crc16_modbus(body) != crc_rx:
        raise IOError("Modbus CRC mismatch")
    return body[3:]


def _to_float(payload: bytes, idx: int) -> float:
    """OXYnor stores floats as two registers, low word first, big-endian within word."""
    low = payload[idx: idx + 2]
    high = payload[idx + 2: idx + 4]
    return struct.unpack(">f", high + low)[0]


def _to_uint32(payload: bytes, idx: int) -> int:
    low = payload[idx: idx + 2]
    high = payload[idx + 2: idx + 4]
    return struct.unpack(">I", high + low)[0]


def _uint32_to_regs(value: int) -> bytes:
    """Inverse of _to_uint32: pack a 32-bit value as two registers, low word
    first, big-endian within each word (the OXYnor word order)."""
    packed = struct.pack(">I", value & 0xFFFFFFFF)
    return packed[2:4] + packed[0:2]


def _float_to_regs(value: float) -> bytes:
    """Inverse of _to_float: pack a float as two registers, low word first,
    big-endian within each word (the OXYnor word order)."""
    packed = struct.pack(">f", float(value))
    return packed[2:4] + packed[0:2]


def _write_2reg(ser, address, data4: bytes):
    """Write a 4-byte (2-register) block via Modbus function 16. ``data4`` must
    already be in OXYnor word order (low word first, big-endian within word)."""
    pdu = bytes([
        MODBUS_SLAVE_ID, 0x10,
        (address >> 8) & 0xFF, address & 0xFF,
        0x00, 0x02,  # quantity of registers
        0x04,        # byte count
    ]) + data4
    frame = pdu + crc16_modbus(pdu)
    ser.reset_input_buffer()
    ser.write(frame)
    # Echo response: slave, func, addr(2), qty(2), CRC(2) = 8 bytes
    expected = 8
    deadline = time.time() + 1.0
    buf = bytearray()
    while len(buf) < expected and time.time() < deadline:
        chunk = ser.read(expected - len(buf))
        if not chunk:
            time.sleep(0.02)
            continue
        buf.extend(chunk)
    if len(buf) < expected or buf[0] != MODBUS_SLAVE_ID:
        raise IOError(f"short or stray write response: {bytes(buf).hex()}")
    if buf[1] & 0x80:
        raise IOError(f"Modbus exception on write, code {buf[2] if len(buf) > 2 else '?'}")
    if buf[1] != 0x10:
        raise IOError(f"unexpected function code on write {buf[1]:#x}")
    body = bytes(buf[:6])
    crc_rx = bytes(buf[6:8])
    if crc16_modbus(body) != crc_rx:
        raise IOError("Modbus CRC mismatch on write response")
    return True


def _write_uint32(ser, address, value):
    """Write a 32-bit value to a 2-register holding block (Modbus function 16)."""
    return _write_2reg(ser, address, _uint32_to_regs(value))


def _write_float(ser, address, value):
    """Write an IEEE-754 float to a 2-register holding block (Modbus fn 16)."""
    return _write_2reg(ser, address, _float_to_regs(value))


# ---------------------------------------------------------------------------
# MAVLink2Rest integration (same approach as the PME extension)
# ---------------------------------------------------------------------------
M2R_ENDPOINTS = [
    "http://host.docker.internal:6040/v1/mavlink",
    "http://localhost:6040/v1/mavlink",
    "http://127.0.0.1:6040/v1/mavlink",
    "http://192.168.2.2:6040/v1/mavlink",
    "http://blueos.local:6040/v1/mavlink",
]


def send_to_mavlink(name, value, component_id=None):
    """Send a NAMED_VALUE_FLOAT to any reachable MAVLink2Rest endpoint.

    Each metric is published from its own component_id so it survives in the
    mavlink2rest store (which keys by system/component/type) and is logged by
    the autopilot. Falls back to the per-name map, then the base id.
    """
    if component_id is None:
        component_id = NAMED_VALUE_COMPONENTS.get(name, MAVLINK_COMPONENT_ID_BASE)
    name_array = []
    for i in range(10):
        name_array.append(name[i] if i < len(name) else "\u0000")
    payload = {
        "header": {
            "system_id": MAVLINK_SYSTEM_ID,
            "component_id": component_id,
            "sequence": 0,
        },
        "message": {
            "type": "NAMED_VALUE_FLOAT",
            "time_boot_ms": 0,
            "value": float(value),
            "name": name_array,
        },
    }
    for endpoint in M2R_ENDPOINTS:
        try:
            response = requests.post(endpoint, json=payload, timeout=2.0)
            if response.status_code == 200:
                print(f"Sent {name}={value} (comp {component_id}) via {endpoint}")
                return True
        except Exception:
            continue
    print(f"Could not send {name}={value} to any MAVLink2Rest endpoint")
    return False


def get_vehicle_temperature():
    """Pull SCALED_PRESSURE2.temperature (centi-deg C) from MAVLink2Rest."""
    endpoints = [
        f"{base}/vehicles/1/components/1/messages/SCALED_PRESSURE2"
        for base in (
            "http://host.docker.internal:6040/v1/mavlink",
            "http://localhost:6040/v1/mavlink",
            "http://127.0.0.1:6040/v1/mavlink",
            "http://192.168.2.2:6040/v1/mavlink",
            "http://blueos.local:6040/v1/mavlink",
        )
    ]
    for ep in endpoints:
        try:
            r = requests.get(ep, timeout=2.0)
            if r.status_code == 200:
                data_json = r.json()
                msg = data_json.get("message", {})
                if "temperature" in msg:
                    return msg["temperature"] / 100.0
        except Exception:
            continue
    return None


def get_gps_position():
    """Pull GLOBAL_POSITION_INT.lat/lon from MAVLink2Rest."""
    try:
        system_id = 1
        try:
            r = requests.get(
                "http://host.docker.internal:6040/v1/mavlink/vehicles",
                timeout=2.0,
            )
            if r.status_code == 200:
                vehicles = r.json()
                if vehicles:
                    system_id = vehicles[0]
        except Exception:
            pass
        ep = (
            f"http://host.docker.internal:6040/v1/mavlink/vehicles/{system_id}"
            f"/components/1/messages/GLOBAL_POSITION_INT"
        )
        r = requests.get(ep, timeout=2.0)
        if r.status_code == 200:
            msg = r.json().get("message", {})
            if "lat" in msg and "lon" in msg:
                return {"lat": msg["lat"] / 1e7, "lon": msg["lon"] / 1e7}
    except Exception as e:
        print(f"Error getting GPS position: {e}")
    return None


# ---------------------------------------------------------------------------
# CSV writer with rotation
# ---------------------------------------------------------------------------
def write_to_csv(measurement):
    try:
        log_path = str(LOG_FILE)
        log_dir = os.path.dirname(log_path)
        os.makedirs(log_dir, exist_ok=True)
        file_exists = os.path.exists(log_path)
        if file_exists:
            size_mb = os.path.getsize(log_path) / (1024 * 1024)
            if size_mb >= MAX_CSV_SIZE_MB:
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                backup = os.path.join(log_dir, f"sensor_data_backup_{ts}.csv")
                os.rename(log_path, backup)
                print(f"Rotated log to {backup} ({size_mb:.1f} MB)")
                file_exists = False
        with open(log_path, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=CSV_HEADERS)
            if not file_exists:
                writer.writeheader()
            writer.writerow(measurement)
            f.flush()
            os.fsync(f.fileno())
    except Exception as e:
        print(f"Error writing CSV: {e}")


# ---------------------------------------------------------------------------
# Sensor poll loop
# ---------------------------------------------------------------------------
def _read_measurement_mode(ser):
    """Best-effort read of the measurement-mode register (5703) for diagnostics."""
    try:
        payload = _read_holding(ser, 5703, 2)
        return f"0x{_to_uint32(payload, 0):X}"
    except Exception as e:
        return f"unread ({e})"


def _ensure_unit(ser, register, desired_code, slot):
    """Ensure an oxygen-unit register holds ``desired_code``.

    Reads the unit register; if it isn't the desired code and we haven't
    exhausted our per-slot attempts, writes the desired code and reads it back
    to confirm. The OXYnor stores these units in flash, so we only write when
    the value differs and cap the number of attempts. mg/L is only accepted in
    humid measurement mode; if rejected the unit reverts, in which case we
    surface the measurement mode so the cause is obvious.

    Returns ``(code, label)`` or ``(None, None)`` if the register couldn't be
    read.
    """
    try:
        payload = _read_holding(ser, register, 2)
        code = _to_uint32(payload, 0)
    except Exception as e:
        print(f"Could not read oxygen unit (reg {register}): {e}")
        return None, None

    if code != desired_code and _unit_write_attempts[slot] < MAX_UNIT_WRITE_ATTEMPTS:
        _unit_write_attempts[slot] += 1
        target = UNIT_CODES.get(desired_code, f"0x{desired_code:X}")
        print(f"{slot.capitalize()} oxygen unit (reg {register}) is 0x{code:X}; "
              f"setting to {target} "
              f"(attempt {_unit_write_attempts[slot]}/{MAX_UNIT_WRITE_ATTEMPTS})")
        try:
            _write_uint32(ser, register, desired_code)
            time.sleep(0.2)  # OXYnor needs a brief settle slot after a write
            payload = _read_holding(ser, register, 2)
            code = _to_uint32(payload, 0)
        except Exception as e:
            print(f"Failed to set {slot} oxygen unit: {e}")
        if code != desired_code:
            mode = _read_measurement_mode(ser)
            print(f"Sensor did not accept {target} for the {slot} unit; it "
                  f"reports 0x{code:X}. mg/L is only valid in humid measurement "
                  f"mode (measurement-mode reg 5703 = {mode}).")

    return code, UNIT_CODES.get(code, f"unit 0x{code:X}")


def ensure_oxygen_units(ser):
    """Ensure both the primary (reg 2089, ppm/mg/L) and secondary (reg 6063,
    % air saturation) oxygen units are configured, caching their codes/labels
    for the API/UI and the CSV writer."""
    global oxygen_unit_code, oxygen_unit_label, second_unit_code, second_unit_label

    code, label = _ensure_unit(ser, 2089, DESIRED_OXYGEN_UNIT_CODE, "primary")
    if code is not None:
        oxygen_unit_code, oxygen_unit_label = code, label

    code2, label2 = _ensure_unit(ser, 6063, DESIRED_SECOND_UNIT_CODE, "secondary")
    if code2 is not None:
        second_unit_code, second_unit_label = code2, label2

    print(f"Oxygen units: primary={oxygen_unit_label} (0x{oxygen_unit_code:X}), "
          f"secondary={second_unit_label} (0x{second_unit_code:X})")


def read_salinity(ser):
    """Read the salinity-compensation value (register 3115, float PSU)."""
    payload = _read_holding(ser, SALINITY_REGISTER, 2)
    return _to_float(payload, 0)


def classify_salinity(value):
    """Map a salinity value to a UI mode label."""
    if value is None:
        return "unknown"
    if abs(value - SALINITY_PRESETS["fresh"]) < SALINITY_WRITE_EPSILON:
        return "fresh"
    if abs(value - SALINITY_PRESETS["salt"]) < SALINITY_WRITE_EPSILON:
        return "salt"
    return "custom"


def read_sensor_loop():
    """Poll the OXYnor every POLL_INTERVAL_S seconds; push to MAVLink + CSV."""
    global data, serial_connection

    load_serial_config()
    if not initialize_serial_connection():
        print("Serial init failed; will retry.")

    last_unit_refresh = 0.0
    while True:
        with SERIAL_LOCK:
            if not serial_connection or not serial_connection.is_open:
                if not initialize_serial_connection():
                    print("Serial unavailable; retrying in 10 s")
                    time.sleep(10)
                    continue

            start = time.time()

            # Ensure/refresh oxygen-unit codes occasionally (1/min is plenty).
            # On startup this also sets the sensor to the desired units
            # (primary mg/L + secondary % air saturation).
            if start - last_unit_refresh > 60:
                ensure_oxygen_units(serial_connection)
                last_unit_refresh = start

            # Read the 14-register measurement block at 4895
            try:
                blk = _read_holding(serial_connection, 4895, 14)
            except Exception as e:
                print(f"Modbus read failed: {e}")
                initialize_serial_connection()
                time.sleep(2)
                continue

            try:
                pressure = _to_float(blk, 0)
                # ref_amp at 4, oxy_amp at 8 - not logged separately by default
                phase = _to_float(blk, 12)
                temperature = _to_float(blk, 16)
                do_value = _to_float(blk, 20)
                error = _to_uint32(blk, 24)
            except Exception as e:
                print(f"Decode failed: {e} raw={blk.hex()}")
                continue

            # Secondary oxygen value (% air saturation) lives in register 6065,
            # outside the 4895 block, so it needs its own read. Failure here must
            # not drop the primary measurement.
            do_air_sat = None
            try:
                blk2 = _read_holding(serial_connection, 6065, 2)
                do_air_sat = _to_float(blk2, 0)
            except Exception as e:
                print(f"Secondary oxygen (reg 6065) read failed: {e}")

            gps = get_gps_position()
            v_temp = get_vehicle_temperature()

            measurement = {
                "timestamp": datetime.now().isoformat(),
                "temperature": round(temperature, 3),
                "do": round(do_value, 3),
                "do_unit": oxygen_unit_label,
                "do_air_saturation": round(do_air_sat, 3) if do_air_sat is not None else None,
                "do_air_saturation_unit": second_unit_label,
                "pressure": round(pressure, 2),
                "phase": round(phase, 3),
                "error": error,
                "vehicle_temperature": v_temp,
                "latitude": gps["lat"] if gps else None,
                "longitude": gps["lon"] if gps else None,
            }

            if not (-10 <= temperature <= 60 and 0 <= do_value <= 500):
                print(f"Measurement out of range; skipping: {measurement}")
            else:
                with DATA_LOCK:
                    data.append(measurement)
                    if len(data) > 60:
                        data = data[-60:]
                print(f"Stored: {measurement}")
                write_to_csv(measurement)

                send_to_mavlink("DO", do_value, NAMED_VALUE_COMPONENTS["DO"])
                send_to_mavlink("TDO", temperature, NAMED_VALUE_COMPONENTS["TDO"])
                if do_air_sat is not None:
                    send_to_mavlink("DOS", do_air_sat, NAMED_VALUE_COMPONENTS["DOS"])

            elapsed = time.time() - start
            time.sleep(max(0.0, POLL_INTERVAL_S - elapsed))


sensor_thread = threading.Thread(target=read_sensor_loop, daemon=True)
sensor_thread.start()


# ---------------------------------------------------------------------------
# HTTP API
# ---------------------------------------------------------------------------
@app.route("/api/data")
def get_data():
    try:
        duration = int(request.args.get("duration", 0))
        max_points = int(request.args.get("max_points", 1000))
    except (TypeError, ValueError):
        duration = 0
        max_points = 1000

    all_data_requested = duration <= 0
    cutoff_time = datetime.now() - timedelta(minutes=duration) if not all_data_requested else None

    log_path = str(LOG_FILE)
    if not os.path.exists(log_path):
        return jsonify([])

    try:
        filtered = []
        with open(log_path, "r") as fp:
            reader = csv.DictReader(fp)
            if not reader.fieldnames or not all(h in reader.fieldnames for h in CSV_HEADERS):
                print(f"CSV header mismatch; expected {CSV_HEADERS}, got {reader.fieldnames}")
                return jsonify([])
            for row in reader:
                try:
                    ts = datetime.fromisoformat(row["timestamp"])
                    if not all_data_requested and ts <= cutoff_time:
                        continue
                    filtered.append({
                        "timestamp": row["timestamp"],
                        "temperature": float(row["temperature"]) if row.get("temperature") else None,
                        "do": float(row["do"]) if row.get("do") else None,
                        "do_unit": row.get("do_unit") or oxygen_unit_label,
                        "do_air_saturation": float(row["do_air_saturation"]) if row.get("do_air_saturation") else None,
                        "do_air_saturation_unit": row.get("do_air_saturation_unit") or second_unit_label,
                        "pressure": float(row["pressure"]) if row.get("pressure") else None,
                        "phase": float(row["phase"]) if row.get("phase") else None,
                        "error": int(row["error"]) if row.get("error") not in (None, "") else None,
                        "vehicle_temperature": float(row["vehicle_temperature"]) if row.get("vehicle_temperature") else None,
                        "latitude": float(row["latitude"]) if row.get("latitude") else None,
                        "longitude": float(row["longitude"]) if row.get("longitude") else None,
                    })
                except (ValueError, KeyError):
                    continue
        filtered.sort(key=lambda r: r["timestamp"])
        total = len(filtered)
        if total > max_points and max_points > 0:
            step = max(1, total // max_points)
            filtered = filtered[::step][:max_points]
        return jsonify(filtered)
    except Exception as e:
        print(f"Error reading CSV: {e}")
        return jsonify([])


@app.route("/api/serial")
def get_serial():
    return jsonify({
        "serial_port": SERIAL_PORT,
        "baud_rate": BAUD_RATE,
        "framing": "8N2",
        "modbus_slave_id": MODBUS_SLAVE_ID,
        "poll_interval_s": POLL_INTERVAL_S,
        "oxygen_unit": oxygen_unit_label,
        "oxygen_unit_code": oxygen_unit_code,
        "secondary_oxygen_unit": second_unit_label,
        "secondary_oxygen_unit_code": second_unit_code,
        "mavlink_system_id": MAVLINK_SYSTEM_ID,
        "mavlink_components": NAMED_VALUE_COMPONENTS,
    })


@app.route("/api/serial/ports")
def get_serial_ports():
    return jsonify({"ports": find_serial_ports()})


@app.route("/api/serial/select", methods=["POST"])
def select_serial_port():
    global SERIAL_PORT
    body = request.json or {}
    new_port = body.get("port")
    if not new_port:
        return jsonify({"success": False, "message": "No port specified"}), 400
    if not os.path.exists(new_port):
        return jsonify({"success": False, "message": f"Port {new_port} does not exist"}), 400
    with SERIAL_LOCK:
        old_port = SERIAL_PORT
        SERIAL_PORT = new_port
        if initialize_serial_connection():
            save_serial_config(new_port)
            return jsonify({"success": True, "message": f"Switched from {old_port} to {new_port}"})
        SERIAL_PORT = old_port
        initialize_serial_connection()
        return jsonify({"success": False, "message": f"Failed to connect to {new_port}, reverted to {old_port}"}), 500


@app.route("/api/salinity")
def get_salinity():
    """Read the salinity the sensor is currently using for mg/L compensation."""
    with SERIAL_LOCK:
        if not serial_connection or not serial_connection.is_open:
            if not initialize_serial_connection():
                return jsonify({"success": False, "message": "Serial port unavailable"}), 503
        try:
            value = read_salinity(serial_connection)
        except Exception as e:
            return jsonify({"success": False, "message": f"Could not read salinity: {e}"}), 500
    return jsonify({
        "success": True,
        "salinity": round(value, 3),
        "mode": classify_salinity(value),
        "presets": SALINITY_PRESETS,
        "min": SALINITY_MIN,
        "max": SALINITY_MAX,
        "unit": "PSU",
    })


@app.route("/api/salinity", methods=["POST"])
def set_salinity():
    """Write the salinity-compensation value to the sensor.

    Requires an explicit ``confirm: true`` in the body (the UI surfaces a
    confirmation prompt) because this persists to the OXYnor's limited-cycle
    flash. Never called automatically. If the requested value matches what the
    sensor already holds, no write is performed.
    """
    body = request.json or {}
    if not body.get("confirm"):
        return jsonify({
            "success": False,
            "message": "Confirmation required before writing salinity to sensor flash",
        }), 400
    try:
        value = float(body.get("value"))
    except (TypeError, ValueError):
        return jsonify({"success": False, "message": "Invalid salinity value"}), 400
    if not (SALINITY_MIN <= value <= SALINITY_MAX):
        return jsonify({
            "success": False,
            "message": f"Salinity must be between {SALINITY_MIN} and {SALINITY_MAX} PSU",
        }), 400

    with SERIAL_LOCK:
        if not serial_connection or not serial_connection.is_open:
            if not initialize_serial_connection():
                return jsonify({"success": False, "message": "Serial port unavailable"}), 503
        try:
            current = read_salinity(serial_connection)
        except Exception as e:
            return jsonify({"success": False, "message": f"Could not read current salinity: {e}"}), 500

        if abs(current - value) < SALINITY_WRITE_EPSILON:
            return jsonify({
                "success": True,
                "written": False,
                "salinity": round(current, 3),
                "mode": classify_salinity(current),
                "message": f"Salinity already {current:.2f} PSU; no write performed.",
            })

        try:
            _write_float(serial_connection, SALINITY_REGISTER, value)
            time.sleep(0.2)  # OXYnor settle slot after a flash write
            new_value = read_salinity(serial_connection)
        except Exception as e:
            return jsonify({"success": False, "message": f"Failed to write salinity: {e}"}), 500

    ok = abs(new_value - value) < SALINITY_WRITE_EPSILON
    print(f"Salinity write requested={value:.2f} PSU, sensor now {new_value:.2f} PSU "
          f"({'ok' if ok else 'MISMATCH'})")
    return jsonify({
        "success": ok,
        "written": True,
        "salinity": round(new_value, 3),
        "mode": classify_salinity(new_value),
        "message": (f"Salinity set to {new_value:.2f} PSU"
                    if ok else
                    f"Wrote {value:.2f} PSU but sensor reports {new_value:.2f} PSU"),
    })


@app.route("/register_service")
def register_service():
    return send_from_directory("static", "register_service")


@app.route("/")
def index():
    return send_from_directory("static", "index.html")


@app.route("/widget")
def widget():
    response = send_from_directory("static", "widget.html")
    response.headers["X-Frame-Options"] = "ALLOWALL"
    response.headers["Content-Security-Policy"] = "frame-ancestors *"
    return response


@app.route("/api/logs")
def download_logs():
    log_path = str(LOG_FILE)
    if not os.path.exists(log_path):
        return "No log file found", 404
    try:
        return send_file(
            log_path,
            mimetype="text/csv",
            as_attachment=True,
            download_name="sentec_do_logs.csv",
        )
    except Exception as e:
        return f"Error accessing log file: {e}", 500


@app.route("/api/logs/delete", methods=["POST"])
def delete_logs():
    log_path = str(LOG_FILE)
    try:
        log_dir = os.path.dirname(log_path)
        deleted = 0
        if os.path.exists(log_path):
            os.remove(log_path)
            deleted += 1
        if os.path.isdir(log_dir):
            for f in os.listdir(log_dir):
                if f.startswith("sensor_data_backup_"):
                    os.remove(os.path.join(log_dir, f))
                    deleted += 1
        return jsonify({"success": True, "message": f"Deleted {deleted} log file(s)"})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500


@app.route("/<path:filename>")
def serve_static(filename):
    return send_from_directory("static", filename)


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=6438)
