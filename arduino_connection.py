import serial
import time
import numpy as np
# # Configure UART port
# # Use '/dev/ttyS0' or '/dev/ttyAMA0' depending on your RPi4 setup
# # If using USB-to-UART adapter, use '/dev/ttyUSB0'
# ser = serial.Serial(
#     port='/dev/ttyACM0',
#     baudrate=9600,
#     timeout=1
# )
# time.sleep(2)

# ser.write(b'Hello Arduion\n')

# response = ser.readline().decode().strip()
# print(response)

class MotorSender():
    def __init__(self, port='/dev/ttyACM0', baudrate=9600, timout=1):
        self.ser = serial.Serial(
            port='/dev/ttyACM0',
            baudrate=9600,
            timeout=1
        )
        time.sleep(2)

    def send_data(self, data):
        self.ser.write(data)

    def read_data(self):
        return self.ser.readline().decode().strip()

    def send_to_arduino(self, motors):
        # Example: sending bytes over serial
        # Make sure arduino is open: self.arduino = serial.Serial(...)
        try:
            signal_string = ",".join(map(str, np.clip(np.array(motors).astype(np.int32), 0, 255))) + "x"
            self.ser.write(signal_string.encode())
        except Exception as e:
            print("Arduino write error:", e)

if __name__ == "__main__":
    motor_sender = MotorSender()
    motor_sender.send_data(b'Hello Arduion\n')  
    print(motor_sender.read_data())