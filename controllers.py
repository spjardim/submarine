import time
import numpy as np


class PID:
    def __init__(self, kp, ki, kd, output_limits=(-127, 127)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_out, self.max_out = output_limits

        self.prev_error = 0
        self.integral = 0
        self.prev_time = None

    def reset(self):
        self.prev_error = 0
        self.integral = 0
        self.prev_time = None

    def update(self, setpoint, measurement):
        now = time.time()
        if self.prev_time is None:
            self.prev_time = now
            return 0

        dt = now - self.prev_time
        self.prev_time = now
        if dt <= 0:
            return 0

        error = setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error

        return float(np.clip(output, self.min_out, self.max_out))


class IBVSController:
    """
    IBVS controller for a quadrotor-style submarine with a forward-facing camera.

    Physical setup
    --------------
    4 vertical thrusters in quadrotor X layout:

        M1 (FL) ---- M2 (FR)
           |              |
        M4 (RL) ---- M3 (RR)

    All motors produce upward thrust, giving:
        Heave  (+up)      : all motors increase equally
        Pitch+ (nose up)  : rear motors increase, front motors decrease
        Roll+  (right up) : left motors increase, right motors decrease
        Yaw+   (CCW)      : M1,M3 increase / M2,M4 decrease
                            (flip all yaw signs if rotation is backwards)

    Surge (forward motion) is produced by pitching the sub nose-down so the
    thrust vector gains a forward horizontal component — standard quadrotor
    behaviour. There are NO horizontal thrusters.

    Camera
    ------
    Forward-facing, mounted roughly level with the sub body.
    Image convention: u increases rightward, v increases downward.

    IBVS mapping
    ------------
    Horizontal pixel error (u_err = pixel_u - target_u):
        Positive → target is RIGHT of centre → roll RIGHT (sub translates right)
        Roll keeps heading fixed; sub moves laterally rather than rotating.

        Sign convention (verify on bench):
            Roll+ = right side UP → thrust tilts LEFT → sub moves LEFT
            Roll- = right side DOWN → thrust tilts RIGHT → sub moves RIGHT
            So u_err > 0 needs roll_cmd < 0 → measurement=+u_err on the PID.

    Vertical pixel error (v_err = pixel_v - target_v):
        Positive → target is BELOW centre → pitch nose DOWN
        (brings target toward centre AND moves sub forward/down)

    Surge (approach):
        Once the target is horizontally centred within `centre_deadband_px`,
        a steady nose-down pitch bias is ramped in to drive the sub forward.
    """

    def __init__(
        self,
        image_width: int = 360,
        image_height: int = 360,
        roll_kp: float = 0.15,
        roll_ki: float = 0.0,
        roll_kd: float = 0.05,
        pitch_kp: float = 0.15,
        pitch_ki: float = 0.0,
        pitch_kd: float = 0.05,
        # Base PWM that keeps the sub neutrally buoyant — tune on the bench.
        hover_throttle: float = 128.0,
        max_roll_authority: float = 60.0,
        max_pitch_authority: float = 120.0,
        # Nose-down pitch bias added when target is centred → forward surge.
        # Set to 0 to disable autonomous approach.
        surge_pitch_bias: float = 20.0,
        # Pixel radius considered "centred" before surge engages.
        centre_deadband_px: float = 10.0,
    ):
        self.cx_img = image_width / 2.0
        self.cy_img = image_height / 2.0

        self.hover_throttle = hover_throttle
        self.max_roll  = max_roll_authority
        self.max_pitch = max_pitch_authority
        self.surge_bias = surge_pitch_bias
        self.deadband = centre_deadband_px

        self.roll_pid  = PID(roll_kp,  roll_ki,  roll_kd,  (-max_roll_authority,  max_roll_authority))
        self.pitch_pid = PID(pitch_kp, pitch_ki, pitch_kd, (-max_pitch_authority, max_pitch_authority))

    def update(
        self,
        pixel_u: float,
        pixel_v: float,
        target_u: float,
        target_v: float,
    ) -> list[int]:
        """
        Compute motor commands from image-space error.

        Returns [m1, m2, m3, m4] each in [0, 255].
            m1 = front-left
            m2 = front-right
            m3 = rear-right
            m4 = rear-left
        """
        u_err = pixel_u - target_u   # +ve → target is RIGHT of centre
        v_err = pixel_v - target_v   # +ve → target is BELOW centre

        # Roll: translate laterally toward horizontal error.
        # measurement=+u_err → negative output when target is right → Roll- → sub moves right.
        roll_cmd  = self.roll_pid.update(setpoint=0.0, measurement=u_err)

        # Pitch: tilt camera toward vertical error.
        pitch_cmd = self.pitch_pid.update(setpoint=0.0, measurement=-v_err)

        # Surge bias: engage a nose-down bias once horizontally centred.
        horiz_error = abs(u_err)
        if horiz_error < self.deadband:
            ramp = 1.0 - (horiz_error / self.deadband)
            pitch_cmd += self.surge_bias * ramp

        pitch_cmd = float(np.clip(pitch_cmd, -self.max_pitch, self.max_pitch))
        roll_cmd  = float(np.clip(roll_cmd,  -self.max_roll,  self.max_roll))

        # Motor mixing
        #
        #   M1 (FL)   M2 (FR)
        #   M4 (RL)   M3 (RR)
        #
        # Heave  : all +h
        # Pitch+ (nose up)  : rear motors +p, front motors -p
        # Roll+  (right up) : left motors +r, right motors -r
        #
        h = self.hover_throttle
        p = pitch_cmd
        r = roll_cmd

        m1 = h - p + r   # FL (left)
        m2 = h - p - r   # FR (right)
        m3 = h + p - r   # RR (right)
        m4 = h + p + r   # RL (left)

        motors = [int(np.clip(m, 0, 255)) for m in [m1, m2, m3, m4]]
        return motors

    def reset(self):
        """Call when tracking is lost to clear PID integrators."""
        self.roll_pid.reset()
        self.pitch_pid.reset()


class SubController:
    def __init__(self, depth_sensor, imu, arduino):

        self.depth_pid = PID(2, 0.0, 0.2, (-127, 127))
        self.roll_pid  = PID(1, 0, 0.5, (-100, 100))
        self.pitch_pid = PID(1, 0, 0.1, (-100, 100))
        self.yaw_pid   = PID(1, 0, 0,   (-100, 100))

        self.ibvs = IBVSController(
            image_width=360,
            image_height=360,
            roll_kp=0.17,
            roll_kd=0.07,
            pitch_kp=0.15,
            pitch_kd=0.05,
            hover_throttle=128.0,       # ← tune until sub hovers level
            max_roll_authority=50.0,
            max_pitch_authority=60.0,
            surge_pitch_bias=30.0,      # ← tune forward approach aggressiveness
            centre_deadband_px=30.0,
        )

    def update_ibvs(
        self,
        pixel_u: float,
        pixel_v: float,
        target_u: float = 180.0,
        target_v: float = 180.0,
        sonar_distance: float | None = None,
        gyro: tuple = (0.0, 0.0, 0.0),
    ) -> list[int]:
        # TODO: This needs to use the gyro to control the inner loop
        # NOTE: split ibvs and the sub controller one makes a command and the other outputs the motors
        return self.ibvs.update(
            pixel_u=pixel_u,
            pixel_v=pixel_v,
            target_u=target_u,
            target_v=target_v,
        )

    def reset_ibvs(self):
        """Call when target is lost to reset PID integrators."""
        self.ibvs.reset()

    def update(self, cur_depth, cur_attitude, target_depth, target_roll, target_pitch, target_yaw):
        depth = cur_depth or 0
        roll, pitch, yaw = cur_attitude

        throttle = self.depth_pid.update(target_depth, depth)
        roll_c   = self.roll_pid.update(target_roll, roll)
        pitch_c  = self.pitch_pid.update(target_pitch, pitch)
        yaw_c    = self.yaw_pid.update(target_yaw, yaw)

        m1 = throttle - pitch_c + roll_c - yaw_c + 100
        m2 = throttle - pitch_c - roll_c + yaw_c + 100
        m3 = throttle + pitch_c - roll_c - yaw_c + 100
        m4 = throttle + pitch_c + roll_c + yaw_c + 100

        motors = [int(max(0, min(255, m))) for m in [m1, m2, m3, m4]]
        return motors

    def send_to_arduino(self, motors):
        try:
            signal_string = ",".join(map(str, np.clip(np.array(motors).astype(np.int32), 0, 255))) + "x"
            self.arduino.send_data(signal_string.encode())
        except Exception as e:
            print("Arduino write error:", e)
