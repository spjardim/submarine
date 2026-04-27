import time
import board
import busio
import adafruit_adxl34x
import adafruit_bmp280
import smbus2

# ================= I2C SETUP =================
i2c = busio.I2C(board.SCL, board.SDA)
bus = smbus2.SMBus(1)

# ================= I2C SCAN =================
def scan_i2c():
    devices = []
    for addr in range(0x03, 0x78):
        try:
            bus.write_quick(addr)
            devices.append(hex(addr))
        except:
            pass
    return devices

print("I2C devices found:", scan_i2c())

# ================= ACCEL =================
accel = adafruit_adxl34x.ADXL345(i2c)

# ================= BAROMETER =================
bmp = adafruit_bmp280.Adafruit_BMP280_I2C(i2c, address=0x77)
bmp.sea_level_pressure = 1013.25  # adjust for your location

# ================= GYRO (ITG-3205) =================
ITG_ADDR = 0x68

bus.write_byte_data(ITG_ADDR, 0x3E, 0x00)  # Wake
bus.write_byte_data(ITG_ADDR, 0x15, 0x07)  # Sample rate
bus.write_byte_data(ITG_ADDR, 0x16, 0x1E)  # DLPF

def read_gyro_raw():
    data = bus.read_i2c_block_data(ITG_ADDR, 0x1D, 6)
    gx = (data[0] << 8) | data[1]
    gy = (data[2] << 8) | data[3]
    gz = (data[4] << 8) | data[5]

    if gx > 32767: gx -= 65536
    if gy > 32767: gy -= 65536
    if gz > 32767: gz -= 65536

    return gx, gy, gz

# --- Gyro calibration ---
def calibrate_gyro(samples=100):
    print("Calibrating gyro... keep device still")
    ox = oy = oz = 0
    for _ in range(samples):
        gx, gy, gz = read_gyro_raw()
        ox += gx
        oy += gy
        oz += gz
        time.sleep(0.01)
    return ox/samples, oy/samples, oz/samples

gyro_offset = calibrate_gyro()

def read_gyro():
    gx, gy, gz = read_gyro_raw()

    gx -= gyro_offset[0]
    gy -= gyro_offset[1]
    gz -= gyro_offset[2]

    return gx/14.375, gy/14.375, gz/14.375

# ================= MAGNETOMETER =================
MAG_ADDR = None
MAG_TYPE = None

# Detect magnetometer
devices = scan_i2c()
if '0xd' in devices:
    MAG_ADDR = 0x0D
    MAG_TYPE = "VCM5883L"
elif '0x1e' in devices:
    MAG_ADDR = 0x1E
    MAG_TYPE = "HMC5883L"

def init_mag():
    if MAG_TYPE == "VCM5883L":
        bus.write_byte_data(MAG_ADDR, 0x0B, 0x01)
        bus.write_byte_data(MAG_ADDR, 0x09, 0x1D)
    elif MAG_TYPE == "HMC5883L":
        bus.write_byte_data(MAG_ADDR, 0x00, 0x70)
        bus.write_byte_data(MAG_ADDR, 0x01, 0x20)
        bus.write_byte_data(MAG_ADDR, 0x02, 0x00)

def read_mag():
    data = bus.read_i2c_block_data(MAG_ADDR, 0x00, 6)

    if MAG_TYPE == "VCM5883L":
        mx = (data[1] << 8) | data[0]
        my = (data[3] << 8) | data[2]
        mz = (data[5] << 8) | data[4]
    else:  # HMC5883L
        mx = (data[0] << 8) | data[1]
        mz = (data[2] << 8) | data[3]
        my = (data[4] << 8) | data[5]

    if mx > 32767: mx -= 65536
    if my > 32767: my -= 65536
    if mz > 32767: mz -= 65536

    return mx, my, mz

if MAG_ADDR:
    init_mag()
    print(f"Magnetometer detected: {MAG_TYPE} @ {hex(MAG_ADDR)}")
else:
    print("WARNING: No magnetometer found!")

# ================= MAIN LOOP =================
print("\nReading SEN0140 IMU...\n")

while True:
    try:
        ax, ay, az = accel.acceleration
        gx, gy, gz = read_gyro()

        if MAG_ADDR:
            mx, my, mz = read_mag()
        else:
            mx = my = mz = 0

        print(
            # f"A:({ax:.2f},{ay:.2f},{az:.2f}) "
            f"G:({gx:.2f},{gy:.2f},{gz:.2f}) "
            # f"M:({mx},{my},{mz}) "
            # f"P:{bmp.pressure:.1f}hPa "
            # f"T:{bmp.temperature:.1f}C "
            # f"Alt:{bmp.altitude:.1f}m"
        )

    except Exception as e:
        print("Read error:", e)

    time.sleep(0.5)