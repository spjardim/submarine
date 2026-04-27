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

    Two-axis control
    ----------------
    Camera Y axis (up/down in image, v_err):
        Drives heave to match the target's depth.
        v_err > 0 → target BELOW centre → sub too high → decrease heave (sink).
        v_err < 0 → target ABOVE centre → sub too low  → increase heave (rise).

    Camera Z axis (forward, always):
        A constant nose-down pitch bias drives the sub forward along Z.
        Pitch tilts the thrust vector forward — the only way to surge with
        vertical-only thrusters.

    Horizontal pixel error (u_err) is ignored — target is always on the forward axis.
    """

    def __init__(
        self,
        hover_throttle: float    = 128.0,
        heave_kp: float          = 0.3,
        heave_ki: float          = 0.0,
        heave_kd: float          = 0.05,
        max_heave_correction: float = 30.0,
        surge_pitch_bias: float  = 30.0,
        max_pitch_authority: float = 60.0,
    ):
        self.hover_throttle = hover_throttle
        self.surge_bias     = surge_pitch_bias
        self.max_pitch      = max_pitch_authority

        self.heave_pid = PID(heave_kp, heave_ki, heave_kd, (-max_heave_correction, max_heave_correction))

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
        v_err = pixel_v - target_v   # +ve → target BELOW centre → sub too high

        # Heave: PID on v_err matches the sub's depth to the target (camera Y axis).
        # v_err > 0 → sink → negative correction; v_err < 0 → rise → positive correction.
        heave_correction = self.heave_pid.update(setpoint=0.0, measurement=v_err)
        h = float(np.clip(self.hover_throttle + heave_correction, 0, 255))

        # Pitch: constant nose-down bias drives forward along camera Z axis.
        p = float(np.clip(self.surge_bias, -self.max_pitch, self.max_pitch))

        # Motor mixing
        #
        #   M1 (FL)   M2 (FR)
        #   M4 (RL)   M3 (RR)
        #
        # Heave  : all +h
        # Pitch+ (nose up)  : rear +p, front -p
        #
        m1 = h - p   # FL
        m2 = h - p   # FR
        m3 = h + p   # RR
        m4 = h + p   # RL

        return [int(np.clip(m, 0, 255)) for m in [m1, m2, m3, m4]]

    def reset(self):
        self.heave_pid.reset()


class SubController:
    def __init__(self, depth_sensor, imu, arduino):

        self.ibvs = IBVSController(
            hover_throttle      = 128.0,  # tune until sub hovers level
            heave_kp            = 0.3,    # tune: pixels → PWM depth correction
            heave_kd            = 0.05,
            max_heave_correction= 30.0,
            surge_pitch_bias    = 30.0,   # tune: forward approach aggressiveness
            max_pitch_authority = 60.0,
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

