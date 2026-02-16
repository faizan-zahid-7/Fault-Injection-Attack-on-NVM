import serial
import csv
import os
import re

ser = serial.Serial('COM4', 115200, timeout=1)

base_dir = os.path.dirname(os.path.abspath(__file__))
output_dir = os.path.join(base_dir, 'mram4_data')
os.makedirs(output_dir, exist_ok=True)

current_writer = None
current_file = None
current_byte = None
byte_offset = 0

print("Logging started...")

try:
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(f"Received: {line}")

            # Detect start of new test cycle
            match = re.match(r"=== Writing value 0x([0-9A-Fa-f]{2}) to entire memory ===", line)
            if match:
                byte_str = match.group(1).upper()

                # Close previous file
                if current_file:
                    current_file.close()
                    print(f"Saved mram_{current_byte}.csv in fram1_data")

                # Open new CSV for this byte
                current_byte = byte_str
                filename = os.path.join(output_dir, f"mram_{current_byte}.csv")
                current_file = open(filename, 'w', newline='')
                current_writer = csv.writer(current_file)
                current_writer.writerow(['Written Byte', 'Byte Offset', 'Read Byte'])
                byte_offset = 0  # reset offset counter
                print(f"Started new CSV: fram_{current_byte}.csv")
                continue

            # Detect memory output lines like "0x0E 0x0E 0x0E ..."
            if line.startswith("0x"):
                hex_values = re.findall(r'0x([0-9A-Fa-f]{2})', line)
                for val in hex_values:
                    if current_writer:
                        current_writer.writerow([current_byte, byte_offset, val.upper()])
                        byte_offset += 1
                if current_file:
                    current_file.flush()

except KeyboardInterrupt:
    print("Logging stopped by user.")
finally:
    if current_file:
        current_file.close()
        print(f"Final CSV for 0x{current_byte} saved.")
    ser.close()
    print("Serial port closed.")
