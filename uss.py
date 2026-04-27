import serial
import time

# Configure UART port
# Use '/dev/ttyS0' or '/dev/ttyAMA0' depending on your RPi4 setup
# If using USB-to-UART adapter, use '/dev/ttyUSB0'
ser = serial.Serial(
    port='/dev/ttyS0',
    baudrate=115200,
    timeout=1
)

COM = 0x55

def read_distance():
    buffer_RTT = [0] * 4
    
    # Send trigger command
    ser.write(bytes([COM]))
    time.sleep(0.1)  # 100ms delay
    
    if ser.in_waiting > 0:
        time.sleep(0.004)  # 4ms delay
        
        first_byte = ser.read(1)
        if len(first_byte) > 0 and first_byte[0] == 0xff:
            buffer_RTT[0] = 0xff
            
            # Read remaining 3 bytes
            remaining = ser.read(3)
            if len(remaining) == 3:
                buffer_RTT[1] = remaining[0]
                buffer_RTT[2] = remaining[1]
                buffer_RTT[3] = remaining[2]
                
                # Checksum verification (only lower 8 bits)
                CS = (buffer_RTT[0] + buffer_RTT[1] + buffer_RTT[2]) & 0xFF
                
                if buffer_RTT[3] == CS:
                    distance = (buffer_RTT[1] << 8) + buffer_RTT[2]
                    return distance
    return None

def main():
    print("Underwater Ultrasonic Sensor - RPi4")
    print("------------------------------------")
    
    try:
        while True:
            distance = read_distance()
            if distance is not None:
                print(f"Distance: {distance} mm")
            else:
                print("No valid reading")
            time.sleep(0.1)
                
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        ser.close()

if __name__ == "__main__":
    main()

