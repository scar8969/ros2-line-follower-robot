#!/usr/bin/env python3
"""pid_model.py — pure-logic PID line-following controller (shared by the ROS2
node and the headless simulator).

Clean-room reimplementation of the controller described in the original
project:

    Control Output = Kp * Error + Ki * Integral(Error) + Kd * Derivative(Error)

plus the behaviors that make it track well on a real track:
  * anti-windup integral clamping,
  * dynamic cornering: linear speed scales down as |error| grows, so the robot
    slows before sharp turns instead of blowing through them,
  * line-loss search: when the sensor reports "no line" for long enough, the
    controller stops dead-reckoning the last error and sweeps the robot in the
    direction it was last steering until the line is re-acquired.

Sign convention (matches the original): a positive error means the line is to
the RIGHT, and the robot must turn right -> angular.z = -steering.
"""

import time

# Search states
SEARCH_IDLE = 0      # line visible, normal PID tracking
SEARCH_HOLD = 1      # line lost, holding last error (short grace period)
SEARCH_SWEEP = 2     # line lost long enough, sweeping to re-acquire


class PIDController:
    """PID with anti-windup, speed scaling, and line-loss search recovery.

    Holds integral/derivative/search state between calls; call `reset()` to
    clear it.
    """

    def __init__(
        self,
        Kp: float = 0.8,
        Ki: float = 0.02,
        Kd: float = 0.15,
        base_linear_speed: float = 0.12,
        min_linear_speed: float = 0.04,
        max_angular_z: float = 2.5,
        integral_clamp: float = 1.0,
        search_hold_time: float = 0.4,
        search_sweep_speed: float = 0.6,
        search_sweep_angular: float = 1.2,
    ):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.base_linear_speed = base_linear_speed
        self.min_linear_speed = min_linear_speed
        self.max_angular_z = max_angular_z
        self.integral_clamp = integral_clamp
        self.search_hold_time = search_hold_time
        self.search_sweep_speed = search_sweep_speed
        self.search_sweep_angular = search_sweep_angular

        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = time.monotonic()
        self._have_prev_error = False

        self.search_state = SEARCH_IDLE
        self._lost_since = 0.0
        self._sweep_dir = 1.0   # +1 = turn right (positive angular), -1 = left

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = time.monotonic()
        self._have_prev_error = False
        self.search_state = SEARCH_IDLE
        self._lost_since = 0.0
        self._sweep_dir = 1.0

    def update(self, error: float | None, dt: float | None = None) -> tuple:
        """Step the controller.

        Parameters
        ----------
        error : normalized line error in [-1, 1], or None if the sensor
                reports no line visible.
        dt    : seconds since the last call (defaults to monotonic clock delta)

        Returns
        -------
        (linear_x, angular_z) — the Twist command the ROS2 node publishes and
        the simulator applies directly.
        """
        if dt is None:
            now = time.monotonic()
            dt = now - self._prev_time
            self._prev_time = now
        if dt <= 0.0:
            dt = 1e-3

        if error is None:
            return self._handle_line_lost(dt)

        # Line visible: normal PID path, reset search state.
        was_searching = self.search_state != SEARCH_IDLE
        self.search_state = SEARCH_IDLE
        self._lost_since = 0.0

        # --- P term ---
        p_term = self.Kp * error

        # --- I term, with anti-windup clamping ---
        self._integral += error * dt
        self._integral = max(-self.integral_clamp, min(self.integral_clamp, self._integral))
        i_term = self.Ki * self._integral

        # --- D term (skip on the first call to avoid a derivative kick from
        # the initial _prev_error=0.0) ---
        if self._have_prev_error:
            derivative = (error - self._prev_error) / dt
            d_term = self.Kd * derivative
        else:
            derivative = 0.0
            d_term = 0.0
        self._prev_error = error
        self._have_prev_error = True

        steering = p_term + i_term + d_term
        steering = max(-self.max_angular_z, min(self.max_angular_z, steering))

        # Dynamic cornering: slow down as the error grows.
        sharpness = min(1.0, abs(error))
        linear_speed = self.base_linear_speed - sharpness * (self.base_linear_speed - self.min_linear_speed)

        # After a search sweep re-acquires the line, ease back to full speed
        # instead of snapping (avoids a steering/acceleration jerk).
        if was_searching:
            linear_speed = min(linear_speed, self.search_sweep_speed * 1.2)

        return float(linear_speed), float(-steering)

    def _handle_line_lost(self, dt: float) -> tuple:
        """Line lost: hold briefly, then sweep in the last steering direction."""
        if self.search_state == SEARCH_IDLE:
            # Just lost the line: enter the hold phase.
            self.search_state = SEARCH_HOLD
            self._lost_since = 0.0
            # Remember which way we were steering so the sweep goes that way.
            self._sweep_dir = 1.0 if self._prev_error >= 0 else -1.0

        self._lost_since += dt

        if self.search_state == SEARCH_HOLD:
            if self._lost_since < self.search_hold_time:
                # Grace period: keep the last commanded steering, decaying.
                hold_error = self._prev_error * 0.5
                lin = self.min_linear_speed
                ang = -self.Kp * hold_error
                ang = max(-self.max_angular_z, min(self.max_angular_z, ang))
                return float(lin), float(ang)
            self.search_state = SEARCH_SWEEP

        # SEARCH_SWEEP: rotate in place toward the last-known line direction.
        lin = self.search_sweep_speed * 0.3   # crawl forward while sweeping
        ang = self._sweep_dir * self.search_sweep_angular
        ang = max(-self.max_angular_z, min(self.max_angular_z, ang))
        return float(lin), float(ang)
