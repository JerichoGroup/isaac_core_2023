"""This file defines a class to control and command a UDP robot in Isaac Sim."""

# ==================== Imports ====================
import math
import time
import numpy as np
from typing import Tuple
from transforms3d.euler import euler2mat, mat2euler
from transforms3d.quaternions import mat2quat, quat2mat

from .base_udp_sender import BaseUDPSender
from .udp_utils import lla_distance_to_m, meters_to_latlon, LLAPoint


# ==================== the UdpBot class ====================
class UdpBot(BaseUDPSender):
    """controls a UDP robot in Isaac Sim."""

    def __init__(self,
                 start_lat: float, start_lon: float, start_alt: float,
                 start_roll_d: float, start_pitch_d: float, start_yaw_d: float,
                 udp_port: float = 33333, send_rate_hz: float = 30,
                 broadcast: bool = True, target_ip: str = "127.0.0.1"
                ) -> None:

        super().__init__(udp_port=udp_port, send_rate_hz=send_rate_hz,
                         broadcast=broadcast, target_ip=target_ip)
        
        self._current_lat = start_lat
        self._current_lon = start_lon
        self._current_alt = start_alt

        self._R_world = euler2mat(
            math.radians(start_roll_d),
            math.radians(start_pitch_d),
            math.radians(start_yaw_d),
            axes='rxyz'
        )

        self._update_from_R_world()
        self._send_current_pose()

    def get_next_point(self) -> Tuple[float, float, float, float, float, float]:
        return (self._current_lat,
                self._current_lon,
                self._current_alt,
                self._current_body_roll_r,
                self._current_body_pitch_r,
                self._current_body_yaw_r)

    @staticmethod
    def _normalize_angle_r(angle_r: float) -> float:
        return (angle_r + math.pi) % (2.0 * math.pi) - math.pi

    def _update_from_R_world(self) -> None:
        roll_r, pitch_r, yaw_r = mat2euler(self._R_world, axes='rxyz')

        roll_r = self._normalize_angle_r(roll_r)
        pitch_r = self._normalize_angle_r(pitch_r)
        yaw_r = self._normalize_angle_r(yaw_r)

        self._current_world_roll_r = roll_r
        self._current_world_pitch_r = pitch_r
        self._current_world_yaw_r = yaw_r

        self._current_body_roll_r = roll_r
        self._current_body_pitch_r = pitch_r
        self._current_body_yaw_r = yaw_r

    def _send_current_pose(self) -> None:
        self.send_once(self._current_lat,
                       self._current_lon,
                       self._current_alt,
                       self._current_body_roll_r,
                       self._current_body_pitch_r,
                       self._current_body_yaw_r)

    def _apply_world_rotation(self, R_delta):
        self._R_world = R_delta @ self._R_world
        self._update_from_R_world()

    def _slerp(self, R1: np.ndarray, R2: np.ndarray, t: float) -> np.ndarray:
        q1 = mat2quat(R1)
        q2 = mat2quat(R2)

        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)

        dot = float(np.dot(q1, q2))

        if dot < 0.0:
            q2 = -q2
            dot = -dot

        if dot > 0.9995:
            q = q1 + t * (q2 - q1)
            q = q / np.linalg.norm(q)
            return quat2mat(q)

        theta_0 = math.acos(dot)
        sin_theta_0 = math.sin(theta_0)

        theta = theta_0 * t
        sin_theta = math.sin(theta)

        s0 = math.sin(theta_0 - theta) / sin_theta_0
        s1 = sin_theta / sin_theta_0

        q = s0 * q1 + s1 * q2
        q = q / np.linalg.norm(q)
        return quat2mat(q)

    # ==================== PRIVATE TURN ENGINE ====================
    def _turn_to(self, target_roll_d: float, target_pitch_d: float, target_yaw_d: float, duration_s: float = 1.0) -> None:

        steps = max(1, int(self.send_rate_hz * duration_s))
        dt = 1.0 / self.send_rate_hz
        start_time = time.perf_counter()

        R_start = self._R_world.copy()
        R_target = euler2mat(
            math.radians(target_roll_d),
            math.radians(target_pitch_d),
            math.radians(target_yaw_d),
            axes='rxyz'
        )

        for i in range(1, steps + 1):
            alpha = i / steps

            self._R_world = self._slerp(R_start, R_target, alpha)
            self._update_from_R_world()
            self._send_current_pose()

            next_tick = start_time + i * dt
            now = time.perf_counter()
            if next_tick > now:
                time.sleep(next_tick - now)

    # ==================== PUBLIC WORLD-FRAME ROTATIONS ====================
    def turn_roll(self, delta_roll_d: float, duration_s: float = 1.0) -> None:
        delta_r = math.radians(delta_roll_d)
        R_delta = euler2mat(delta_r, 0.0, 0.0, axes='rxyz')
        R_target = R_delta @ self._R_world

        roll_r, pitch_r, yaw_r = mat2euler(R_target, axes='rxyz')

        self._turn_to(math.degrees(roll_r),
                      math.degrees(pitch_r),
                      math.degrees(yaw_r),
                      duration_s)

    def turn_pitch(self, delta_pitch_d: float, duration_s: float = 1.0) -> None:
        delta_r = math.radians(delta_pitch_d)
        R_delta = euler2mat(0.0, delta_r, 0.0, axes='rxyz')
        R_target = R_delta @ self._R_world

        roll_r, pitch_r, yaw_r = mat2euler(R_target, axes='rxyz')

        self._turn_to(math.degrees(roll_r),
                      math.degrees(pitch_r),
                      math.degrees(yaw_r),
                      duration_s)

    def turn_yaw(self, delta_yaw_d: float, duration_s: float = 1.0) -> None:
        delta_r = math.radians(delta_yaw_d)
        R_delta = euler2mat(0.0, 0.0, delta_r, axes='rxyz')
        R_target = R_delta @ self._R_world

        roll_r, pitch_r, yaw_r = mat2euler(R_target, axes='rxyz')

        self._turn_to(math.degrees(roll_r),
                      math.degrees(pitch_r),
                      math.degrees(yaw_r),
                      duration_s)

    # ==================== LOOK-AT TARGET ====================
    def turn_to_point(self, target_lat: float, target_lon: float, target_alt: float, duration_s: float = 1.0,) -> None:

        start_lat = self._current_lat
        start_lon = self._current_lon
        start_alt = self._current_alt

        p_start = LLAPoint(start_lat, start_lon, start_alt)

        dy = lla_distance_to_m(p_start, LLAPoint(target_lat, start_lon, start_alt))
        if target_lat < start_lat:
            dy = -dy

        dx = lla_distance_to_m(p_start, LLAPoint(start_lat, target_lon, start_alt))
        if target_lon < start_lon:
            dx = -dx

        dz = target_alt - start_alt

        yaw_r = math.atan2(dx, dy)

        horizontal_dist = math.sqrt(dx * dx + dy * dy)
        pitch_r = math.atan2(dz, horizontal_dist)

        self._turn_to(
            math.degrees(self._current_world_roll_r),
            math.degrees(pitch_r),
            math.degrees(yaw_r),
            duration_s
        )

    # ==================== MOVEMENT ====================
    def move_to_point(self, target_lat: float, target_lon: float, target_alt: float,
                      target_roll_d: float, target_pitch_d: float, target_yaw_d: float,
                      duration_s: float = 1.0, look_at_target: bool = True, turn_duration_s: float = 0.5) -> None:

        if look_at_target:
            self.turn_to_point(target_lat, target_lon, target_alt, duration_s=turn_duration_s)

        steps = max(1, int(self.send_rate_hz * duration_s))
        dt = 1.0 / self.send_rate_hz
        start_time = time.perf_counter()

        start_lat = self._current_lat
        start_lon = self._current_lon
        start_alt = self._current_alt

        p_start = LLAPoint(start_lat, start_lon, start_alt)

        dy = lla_distance_to_m(p_start, LLAPoint(target_lat, start_lon, start_alt))
        if target_lat < start_lat:
            dy = -dy

        dx = lla_distance_to_m(p_start, LLAPoint(start_lat, target_lon, start_alt))
        if target_lon < start_lon:
            dx = -dx

        dz = target_alt - start_alt

        for i in range(1, steps + 1):
            alpha = i / steps

            x = alpha * dx
            y = alpha * dy
            z = start_alt + alpha * dz

            d_lat, d_lon = meters_to_latlon(y, x, start_lat)
            self._current_lat = start_lat + d_lat
            self._current_lon = start_lon + d_lon
            self._current_alt = z

            self._send_current_pose()

            next_tick = start_time + i * dt
            now = time.perf_counter()
            if next_tick > now:
                time.sleep(next_tick - now)

        if look_at_target:
            self._turn_to(target_roll_d, target_pitch_d, target_yaw_d, duration_s=turn_duration_s)




    def move_forward(self, distance_m: float, duration_s: float = 1.0) -> None:
        yaw_r = self._current_world_yaw_r

        # Isaac forward: +Y at yaw=0, so:
        dx = distance_m * math.sin(yaw_r)
        dy = distance_m * math.cos(yaw_r)

        d_lat, d_lon = meters_to_latlon(dy, dx, self._current_lat)
        target_lat = self._current_lat + d_lat
        target_lon = self._current_lon + d_lon

        self.move_to_point(target_lat=target_lat,
                           target_lon=target_lon,
                           target_alt=self._current_alt,
                           target_roll_d=math.degrees(self._current_world_roll_r),
                           target_pitch_d=math.degrees(self._current_world_pitch_r),
                           target_yaw_d=math.degrees(self._current_world_yaw_r),
                           duration_s=duration_s,
                           look_at_target=False)




    def move_right(self, distance_m: float, duration_s: float = 1.0) -> None:
        yaw_r = self._current_world_yaw_r

        # Correct right vector (clockwise 90° from forward)
        dx = distance_m * math.cos(yaw_r)
        dy = -distance_m * math.sin(yaw_r)

        d_lat, d_lon = meters_to_latlon(dy, dx, self._current_lat)
        target_lat = self._current_lat + d_lat
        target_lon = self._current_lon + d_lon

        self.move_to_point(target_lat=target_lat,
                        target_lon=target_lon,
                        target_alt=self._current_alt,
                        target_roll_d=math.degrees(self._current_world_roll_r),
                        target_pitch_d=math.degrees(self._current_world_pitch_r),
                        target_yaw_d=math.degrees(self._current_world_yaw_r),
                        duration_s=duration_s,
                        look_at_target=False)







    def move_up(self, distance_m: float, duration_s: float = 1.0) -> None:
        target_alt = self._current_alt + distance_m

        self.move_to_point(self._current_lat,
                           self._current_lon,
                           target_alt,
                           math.degrees(self._current_world_roll_r),
                           math.degrees(self._current_world_pitch_r),
                           math.degrees(self._current_world_yaw_r),
                           duration_s,
                           look_at_target=False)
