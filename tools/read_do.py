#!/usr/bin/env python3
"""Read and decode live values from the Sentec OXYnor DO sensor (Modbus RTU)."""
import sys
import time
import struct
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
DEV_ID = 1

UNIT_CODES = {
    0x10: "% vol (%O2)", 0x20: "% air saturation", 0x40: "ppb (ug/L)",
    0x80: "ppm (mg/L)", 0x80000000: "Torr", 0x04000000: "umol/L",
    0x20000000: "hPa", 0x40000000: "ppm gas",
}


def crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else crc >> 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def read_regs(ser, addr, count):
    pdu = bytes([DEV_ID, 3, (addr >> 8) & 0xFF, addr & 0xFF,
                 (count >> 8) & 0xFF, count & 0xFF])
    ser.reset_input_buffer()
    ser.write(pdu + crc16(pdu))
    time.sleep(0.25)
    resp = ser.read(5 + 2 * count)
    if len(resp) < 5 or resp[1] != 3:
        raise IOError(f"bad response: {resp.hex()}")
    nbytes = resp[2]
    return resp[3:3 + nbytes]


def to_float(payload, i):
    # Two registers per float, low word first, big-endian within word.
    reg1 = payload[i:i + 2]      # low word
    reg2 = payload[i + 2:i + 4]  # high word
    return struct.unpack(">f", reg2 + reg1)[0]


def to_int(payload, i):
    reg1 = payload[i:i + 2]
    reg2 = payload[i + 2:i + 4]
    return struct.unpack(">I", reg2 + reg1)[0]


def main():
    ser = serial.Serial(PORT, 19200, bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_TWO,
                        timeout=0.6)

    fw = read_regs(ser, 1031, 8)
    print("Firmware Version :", fw.decode("latin-1", "replace").strip())
    sn = read_regs(ser, 1063, 8)
    print("Serial Number    :", sn.decode("latin-1", "replace").strip())

    unit_raw = to_int(read_regs(ser, 2089, 2), 0)
    print("Oxygen Unit code : 0x%X (%s)" % (unit_raw, UNIT_CODES.get(unit_raw, "unknown")))

    print("\nLive measurement (register block 4895, 14 regs):")
    blk = read_regs(ser, 4895, 14)
    print("  Pressure          : %.2f hPa" % to_float(blk, 0))
    print("  Reference Amplitude: %.1f uV" % to_float(blk, 4))
    print("  Oxygen Amplitude   : %.1f uV" % to_float(blk, 8))
    print("  Oxygen Phase shift : %.3f deg" % to_float(blk, 12))
    print("  Temperature        : %.3f C" % to_float(blk, 16))
    print("  Calculated Oxygen  : %.3f (%s)" % (to_float(blk, 20),
                                                UNIT_CODES.get(unit_raw, "?")))
    print("  Error register     : 0x%X" % to_int(blk, 24))
    ser.close()


if __name__ == "__main__":
    main()
