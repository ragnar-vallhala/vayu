import struct
import re

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
    if len(data) < 60:
        raise ValueError(f"Expected at least 60 bytes, got {len(data)}")

    floats = struct.unpack('<15f', data[:60])  # little-endian, 15 floats

    result = {
        "acc_offset": floats[0:3],
        "acc_scale":  floats[3:6],
        "gyr_offset": floats[6:9],
        "mag_offset": floats[9:12],
        "mag_scale":  floats[12:15],
    }

    return result


def pretty_print(calib):
    for key, values in calib.items():
        print(f"{key}:")
        for i, v in enumerate(values):
            print(f"  [{i}] = {v:.6f}")
        print()


if __name__ == "__main__":
    hexdump = """
0000000 0000 0000 0000 0000 0000 0000 0000 3f80
0000010 0000 3f80 0000 3f80 8000 3e69 6000 3ee8
0000020 3000 3ee0 0000 0000 0000 0000 0000 0000
0000030 0000 3f80 0000 3f80 0000 3f80
"""

    data = parse_hexdump(hexdump)
    calib = decode_calibration(data)
    pretty_print(calib)