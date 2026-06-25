import struct
import re

# On-disk calibration file (0:cal.bin) format, v3:
#   header: uint32 magic ('VCAL' = 0x4C414356), uint16 version (3),
#           uint16 payload_size (== sizeof(bmx160_calibration_t) = 108)
#   payload: 27 little-endian floats:
#     acc_offset[3], acc_soft_iron[9], gyr_offset[3], mag_offset[3], mag_soft_iron[9]
#   (v3 replaced acc_scale[3] with the full acc_soft_iron[9].)
CALIB_MAGIC = 0x4C414356
CALIB_VERSION = 3
PAYLOAD_FLOATS = 27
PAYLOAD_BYTES = PAYLOAD_FLOATS * 4  # 108
HEADER_BYTES = 8


def parse_hexdump(hexdump_str):
    # Extract hex values (ignore offsets)
    hex_values = re.findall(r'\b[0-9a-fA-F]{4}\b', hexdump_str)

    # Convert to bytes (little endian words)
    byte_array = bytearray()
    for word in hex_values:
        # Each word is 16-bit, need to split into bytes (little endian)
        val = int(word, 16)
        byte_array.append(val & 0xFF)
        byte_array.append((val >> 8) & 0xFF)

    return byte_array


def decode_calibration(data):
    if len(data) < HEADER_BYTES + PAYLOAD_BYTES:
        raise ValueError(
            f"Expected at least {HEADER_BYTES + PAYLOAD_BYTES} bytes, "
            f"got {len(data)}")

    magic, version, payload_size = struct.unpack('<IHH', data[:HEADER_BYTES])
    if magic != CALIB_MAGIC:
        raise ValueError(
            f"Bad magic 0x{magic:08X} (expected 0x{CALIB_MAGIC:08X}); "
            f"this may be a pre-v2 headerless file.")
    if version != CALIB_VERSION:
        raise ValueError(f"Unsupported version {version} (expected {CALIB_VERSION})")
    if payload_size != PAYLOAD_BYTES:
        raise ValueError(
            f"Header payload_size {payload_size} != expected {PAYLOAD_BYTES}")

    floats = struct.unpack(
        '<%df' % PAYLOAD_FLOATS,
        data[HEADER_BYTES:HEADER_BYTES + PAYLOAD_BYTES])

    result = {
        "acc_offset":    floats[0:3],
        "acc_soft_iron": floats[3:12],   # row-major 3x3
        "gyr_offset":    floats[12:15],
        "mag_offset":    floats[15:18],
        "mag_soft_iron": floats[18:27],  # row-major 3x3
    }

    return result


def pretty_print(calib):
    for key, values in calib.items():
        print(f"{key}:")
        if key in ("acc_soft_iron", "mag_soft_iron"):
            for r in range(3):
                row = values[r * 3:r * 3 + 3]
                print("  [{: .6f} {: .6f} {: .6f}]".format(*row))
        else:
            for i, v in enumerate(values):
                print(f"  [{i}] = {v:.6f}")
        print()


if __name__ == "__main__":
    # Build a sample v3 file (identity calibration) and decode it, so the demo
    # stays self-consistent with the on-disk format.
    payload = struct.pack(
        '<%df' % PAYLOAD_FLOATS,
        0.0, 0.0, 0.0,        # acc_offset
        1.0, 0.0, 0.0,        # acc_soft_iron (identity 3x3)
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
        0.0, 0.0, 0.0,        # gyr_offset
        0.0, 0.0, 0.0,        # mag_offset
        1.0, 0.0, 0.0,        # mag_soft_iron (identity 3x3)
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0)
    header = struct.pack('<IHH', CALIB_MAGIC, CALIB_VERSION, PAYLOAD_BYTES)
    data = header + payload

    calib = decode_calibration(data)
    pretty_print(calib)
