import os
import csv

# Input folder where your original FRAM CSV files are
input_folder = os.path.join(os.path.dirname(__file__), 'fram1_data')
# Output folder to save separate fault analysis results
output_folder = os.path.join(os.path.dirname(__file__), 'fram1_fault_analysis')
os.makedirs(output_folder, exist_ok=True)

def nibble_shift_analysis(written_hex, read_hex):
    try:
        written = int(written_hex, 16)
        read = int(read_hex, 16)
    except ValueError:
        return 'Invalid Data'

    if written == read:
        return 'OK'

    # Full 8-bit rotations
    if ((written << 4) | (written >> 4)) & 0xFF == read:
        return 'Rotation Left 4'
    if ((written >> 4) | (written << 4)) & 0xFF == read:
        return 'Rotation Right 4'

    # Nibble rotation (swap high and low nibbles)
    written_swapped = ((written & 0x0F) << 4) | ((written & 0xF0) >> 4)
    if written_swapped == read:
        return 'Nibble Swap'

    # Nibble-level shift
    w_high = (written & 0xF0) >> 4
    w_low = written & 0x0F
    r_high = (read & 0xF0) >> 4
    r_low = read & 0x0F

    def shift_detect(w, r, part):
        if ((w << 1) & 0x0F) == r:
            return f'{part} Left Shift'
        if (w >> 1) == r:
            return f'{part} Right Shift'
        if ((w << 2) & 0x0F) == r:
            return f'{part} Double Left Shift'
        if (w >> 2) == r:
            return f'{part} Double Right Shift'
        return f'{part} Unknown'

    high_result = shift_detect(w_high, r_high, 'High')
    low_result = shift_detect(w_low, r_low, 'Low')

    return f'{high_result}, {low_result}'

# Process each FRAM file
for filename in sorted(os.listdir(input_folder)):
    if filename.startswith("fram_") and filename.endswith(".csv"):
        written_byte = filename.split('_')[1].split('.')[0].upper()  # Extract written byte from filename
        input_path = os.path.join(input_folder, filename)

        with open(input_path, 'r') as in_csv:
            reader = csv.reader(in_csv)
            next(reader)  # Skip header

            output_path = os.path.join(output_folder, f"faults_{filename}")
            with open(output_path, 'w', newline='') as out_csv:
                writer = csv.writer(out_csv)
                writer.writerow(['File', 'Offset', 'Written Byte', 'Read Byte', 'Fault Type'])

                fault_count = 0
                for row in reader:
                    if len(row) != 3:
                        continue

                    offset = row[0].strip()
                    read_byte = row[2].strip().upper()

                    if read_byte != written_byte:
                        fault_type = nibble_shift_analysis(written_byte, read_byte)
                        writer.writerow([filename, offset, written_byte, read_byte, fault_type])
                        fault_count += 1

                    if fault_count >= 10:
                        break
