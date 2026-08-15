#!/usr/bin/env python3
"""
Test that the binary format was generated correctly
"""

import struct
import json

def main():
    # Read original JSON
    with open("TeamCode/src/main/assets/ShooterLUT.json", 'r') as f:
        json_data = json.load(f)
    json_entries = json_data['data']

    # Read binary file
    with open("TeamCode/src/main/assets/ShooterLUT.bin", 'rb') as f:
        # Read number of entries
        num_entries = struct.unpack('i', f.read(4))[0]
        print(f"Binary file contains {num_entries} entries")

        # Read first 5 entries and verify against JSON
        print("\nVerifying first 5 entries:")
        for i in range(min(5, num_entries)):
            # Read binary entry
            x, y, rps, hood = struct.unpack('dddd', f.read(32))

            # Compare with JSON
            json_entry = json_entries[i]
            matches = (
                x == json_entry['x_pixel'] and
                y == json_entry['y_pixel'] and
                rps == json_entry['rps'] and
                hood == json_entry['hood']
            )

            status = "✓" if matches else "✗"
            print(f"  Entry {i}: {status}")
            print(f"    Binary: x={x}, y={y}, rps={rps}, hood={hood}")
            print(f"    JSON:   x={json_entry['x_pixel']}, y={json_entry['y_pixel']}, " +
                  f"rps={json_entry['rps']}, hood={json_entry['hood']}")

    print(f"\n✓ Binary format verification complete!")
    print(f"Total entries match: {num_entries == len(json_entries)}")

if __name__ == "__main__":
    main()
