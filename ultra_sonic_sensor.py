import serial
import time

COM = 0x55

class DistanceSensor():
    def __init__(
        self,
        port: str = '/dev/ttyS0',
        baudrate: int =115200,
        timeout: float=1
        ):

        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout
        )

    def read_distance(self) -> float:
        buffer_RTT = [0] * 4
        
        # Send trigger command
        self.ser.write(bytes([COM]))
        time.sleep(0.1)  # 100ms delay
        
        if self.ser.in_waiting > 0:
            time.sleep(0.004)  # 4ms delay
            
            first_byte = self.ser.read(1)
            if len(first_byte) > 0 and first_byte[0] == 0xff:
                buffer_RTT[0] = 0xff
                
                # Read remaining 3 bytes
                remaining = self.ser.read(3)
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

    sensor = DistanceSensor()
    try:
        while True:
            distance = sensor.read_distance()
            if distance is not None:
                print(f"Distance: {distance} mm")
            else:
                print("No valid reading")
            time.sleep(0.1)
                
    except KeyboardInterrupt:
        print("\nStopping...")

if __name__ == "__main__":
    main()

