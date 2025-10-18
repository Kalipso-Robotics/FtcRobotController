#!/usr/bin/env python3
"""
Converts ShooterLUT.json into a compact binary format.
This eliminates the need for JSON parsing during initialization.
"""

import json
import struct
import os

def main():
    # Read the JSON file
    json_path = "TeamCode/src/main/assets/ShooterLUT.json"

    if not os.path.exists(json_path):
        print(f"Error: {json_path} not found!")
        return

    print("Reading ShooterLUT.json...")
    with open(json_path, 'r') as f:
        data = json.load(f)

    entries = data['data']
    print(f"Found {len(entries)} entries")

    # Generate binary data file
    bin_output = "TeamCode/src/main/assets/ShooterLUT.bin"
    test_bin_output = "TeamCode/src/test/resources/ShooterLUT.bin"

    print("Generating binary file...")
    with open(bin_output, 'wb') as f:
        # Write number of entries as int (4 bytes)
        f.write(struct.pack('i', len(entries)))

        # Write each entry as 4 doubles (32 bytes per entry)
        for i, entry in enumerate(entries):
            # Pack: x_pixel, y_pixel, rps, hood as doubles
            f.write(struct.pack('dddd',
                              entry['x_pixel'],
                              entry['y_pixel'],
                              entry['rps'],
                              entry['hood']))

            if (i + 1) % 10000 == 0:
                print(f"  Wrote {i + 1}/{len(entries)} entries...")

    # Copy to test resources
    import shutil
    os.makedirs(os.path.dirname(test_bin_output), exist_ok=True)
    shutil.copy(bin_output, test_bin_output)

    print(f"\nSuccessfully generated binary files:")
    print(f"  {bin_output}")
    print(f"  {test_bin_output}")
    print(f"Total entries: {len(entries)}")

    # Get file sizes
    size_kb = os.path.getsize(bin_output) / 1024
    json_size_mb = os.path.getsize(json_path) / (1024 * 1024)
    print(f"Binary file size: {size_kb:.2f} KB (vs {json_size_mb:.2f} MB JSON)")
    print(f"Size reduction: {(1 - size_kb/1024/json_size_mb)*100:.1f}%")

if __name__ == "__main__":
    main()
