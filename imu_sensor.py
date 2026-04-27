import time
import board
import busio
import adafruit_adxl34x
import adafruit_bmp280
import smbus2

ITG_ADDR = 0x68

class IMUSensor():
    def __init__(
        self
    ):

        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.bus = smbus2.SMBus(1)


        self.bus.write_byte_data(ITG_ADDR, 0x3E, 0x00)  # Wake
        self.bus.write_byte_data(ITG_ADDR, 0x15, 0x07)  # Sample rate
        self.bus.write_byte_data(ITG_ADDR, 0x16, 0x1E)  # DLPF

        self.gyro_offset = [0,0,0]

    def set_gyro_offset(self, gyro_offset):
        self.gyro_offset = gyro_offset
    def read_gyro_raw(self):
        data = self.bus.read_i2c_block_data(ITG_ADDR, 0x1D, 6)
        gx = (data[2] << 8) | data[3]
        gy = (data[0] << 8) | data[1]
        gz = (data[4] << 8) | data[5]

        if gx > 32767: gx -= 65536
        if gy > 32767: gy -= 65536
        if gz > 32767: gz -= 65536

        return -gx, gy, gz

    def scan_i2c(self):
        devices = []
        for addr in range(0x03, 0x78):
            try:
                self.bus.write_quick(addr)
                devices.append(hex(addr))
            except:
                pass
        return devices

    def calibrate_gyro(self, samples=100):
        print("Calibrating gyro... keep device still")
        ox = oy = oz = 0
        for _ in range(samples):
            gx, gy, gz = self.read_gyro_raw()
            ox += gx
            oy += gy
            oz += gz
            time.sleep(0.01)
        return [ox/samples, oy/samples, oz/samples]

    def read_gyro(self):
        gx, gy, gz = self.read_gyro_raw()

        gx -= self.gyro_offset[0]
        gy -= self.gyro_offset[1]
        gz -= self.gyro_offset[2]

        return gx/14.375, gy/14.375, gz/14.375

if __name__ == "__main__":
    sensor = IMUSensor()

    while True:
        try:
            gx, gy, gz = sensor.read_gyro()
            print(
                f"G:({gx:.2f},{gy:.2f},{gz:.2f}) "
            )

        except Exception as e:
            print("Read error:", e)

        time.sleep(0.5)