import sys
import struct
import serial



# --- Constants ---
START_FRAME = 0xAAAA
SERIAL_PORT = "/dev/ttyS0"  # Default serial port used in C code

# Important: Check if baud rate etc. are needed for your device
BAUD_RATE = 19200 # Common default, adjust if needed

# --- Structure Definition (using struct format string) ---
# < : Little-endian (check if receiver needs Big-endian '>')
# H : unsigned short (uint16_t, 2 bytes) - start_of_frame
# h : short (int16_t, 2 bytes) - steer
# h : short (int16_t, 2 bytes) - speed
# H : unsigned short (uint16_t, 2 bytes) - checksum
# Total size = 2 + 2 + 2 + 2 = 8 bytes
PACK_FORMAT = '<HhhH' # Ensure endianness matches receiver!

print(f"Opening serial port {SERIAL_PORT} at {BAUD_RATE} baud...")
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)


def calculate_checksum(steer_val, speed_val):
    """Calculates the checksum based on the C logic."""
    # Note: Python's integers handle arbitrary size.
    # The XOR operation works directly.
    # We mask with 0xFFFF to ensure the result fits uint16_t.
    # Ensure steer/speed are treated as numbers for XOR
    checksum = (START_FRAME ^ steer_val ^ speed_val) & 0xFFFF
    return checksum

def Control(r_steer_in, r_speed_in):
    speed_in = int(r_speed_in)
    steer_in = int(r_steer_in)
    # 3. Calculate Checksum
    checksum_val = calculate_checksum(steer_in, speed_in)

    #    print(f"Start Frame: {START_FRAME:#04x}")
    # print(f"Steer      : {steer_in}")
    print(f"Speed      : {speed_in}")
    # print(f"Checksum   : {checksum_val:#04x} ({checksum_val})")

    # 4. Pack the data into a binary structure
    try:
        packed_data = struct.pack(PACK_FORMAT, START_FRAME, steer_in, speed_in, checksum_val)
        print(f"Packed data ({len(packed_data)} bytes): {packed_data.hex()}")
    except struct.error as e:
        print(f"Error packing data: {e}")
        print("This might happen if steer/speed values are outside the int16 range.")
        sys.exit(1)


Control(int(sys.argv[1]), int(sys.argv[2]))
