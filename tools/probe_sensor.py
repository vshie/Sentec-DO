#!/usr/bin/env python3
"""
Probe a Sentec OXYnor DO sensor over an RS485 (BLUART/FTDI) adapter.

Tries both protocols described in the Sentec datasheets:
  1) Modbus RTU  (default: 19200 baud, 8 data bits, 2 stop bits, no parity, ID 1)
  2) ASCII / command-line protocol (19200 baud, 8N1)

Goal: verify we can receive valid data from the device.
"""
import sys
import time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"


def crc16_modbus(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])  # low byte first


def build_read(dev_id: int, func: int, addr: int, count: int) -> bytes:
    pdu = bytes([dev_id, func,
                 (addr >> 8) & 0xFF, addr & 0xFF,
                 (count >> 8) & 0xFF, count & 0xFF])
    return pdu + crc16_modbus(pdu)


def hexs(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


def try_modbus():
    print("\n" + "=" * 60)
    print("MODBUS RTU PROBE (19200 8N2)")
    print("=" * 60)
    # Common test reads: firmware date (1023,8), oxygen value (2091,2),
    # raw measurement block (4895,14), device id (4095,2)
    test_reads = [
        ("Firmware Date (1023,8)", 1023, 8),
        ("Device ID (4095,2)", 4095, 2),
        ("Oxygen Value (2091,2)", 2091, 2),
        ("Raw block (4895,14)", 4895, 14),
    ]
    # Try the configured-default framing first, then common alternatives.
    serial_cfgs = [
        ("8N2", serial.PARITY_NONE, serial.STOPBITS_TWO),
        ("8N1", serial.PARITY_NONE, serial.STOPBITS_ONE),
        ("8E1", serial.PARITY_EVEN, serial.STOPBITS_ONE),
    ]
    baudrates = [19200, 9600, 38400, 115200]
    found = False
    for baud in baudrates:
        for cfg_name, parity, stop in serial_cfgs:
            try:
                ser = serial.Serial(PORT, baudrate=baud, bytesize=serial.EIGHTBITS,
                                     parity=parity, stopbits=stop, timeout=0.6)
            except Exception as e:
                print(f"  open failed {baud}/{cfg_name}: {e}")
                continue
            for dev_id in (1, 0):
                for label, addr, count in test_reads:
                    req = build_read(dev_id, 3, addr, count)
                    ser.reset_input_buffer()
                    ser.write(req)
                    time.sleep(0.3)
                    resp = ser.read(256)
                    if resp:
                        print(f"  [{baud} {cfg_name} id={dev_id}] {label}")
                        print(f"      TX: {hexs(req)}")
                        print(f"      RX: {hexs(resp)}")
                        # Validate it's a sane modbus response
                        if len(resp) >= 5 and resp[0] == dev_id and resp[1] in (3, 4, 0x83, 0x84):
                            print("      -> Looks like a valid Modbus response!")
                            found = True
                    # if anything found at this config, keep probing this config only
                if found:
                    ser.close()
                    return True
            ser.close()
    if not found:
        print("  No Modbus response on any tested config.")
    return found


def try_ascii():
    print("\n" + "=" * 60)
    print("ASCII / COMMAND-LINE PROBE (19200 8N1)")
    print("=" * 60)
    cmds = ["data\r", "srno?\r", "code?\r", "repo\r", "post\r", "idno?\r"]
    found = False
    for baud in (19200, 9600, 38400):
        try:
            ser = serial.Serial(PORT, baudrate=baud, bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                 timeout=1.0)
        except Exception as e:
            print(f"  open failed {baud}: {e}")
            continue
        # First, passively listen in case device is in continuous mode
        ser.reset_input_buffer()
        time.sleep(2.0)
        passive = ser.read(512)
        if passive:
            print(f"  [{baud}] passive listen RX: {passive!r}")
            found = True
        for c in cmds:
            ser.reset_input_buffer()
            ser.write(c.encode())
            time.sleep(0.5)
            resp = ser.read(512)
            if resp:
                print(f"  [{baud}] '{c.strip()}' -> {resp!r}")
                found = True
            else:
                print(f"  [{baud}] '{c.strip()}' -> (no response)")
        ser.close()
        if found:
            break
    if not found:
        print("  No ASCII response on any tested config.")
    return found


if __name__ == "__main__":
    print(f"Probing sensor on {PORT}")
    m = try_modbus()
    a = try_ascii()
    print("\n" + "=" * 60)
    print(f"RESULT: modbus={'OK' if m else 'no'}, ascii={'OK' if a else 'no'}")
    print("=" * 60)
