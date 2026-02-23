"""This file defines a class to control and command a UDP robot in Isaac Sim."""

# ==================== Imports ====================
from __future__ import annotations

import math
import time
import numpy as np
from typing import Tuple
from transforms3d.euler import euler2mat, mat2euler

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
        """Initialize the UdpBot with a starting po and an address"""

        super().__init__(udp_port=udp_port, send_rate_hz=send_rate_hz,
                         broadcast=broadcast, target_ip=target_ip)
        
        self._current_lat = start_lat
        self._current_lon = start_lon
        self._current_alt = start_alt

        start_roll_r = math.radians(start_roll_d)
        start_pitch_r = math.radians(start_pitch_d)
        start_yaw_r = math.radians(start_yaw_d)

        self._update_orientation_world_r(start_roll_r, start_pitch_r, start_yaw_r)

        self._send_current_pose()

    def get_next_point(self) -> Tuple[float, float, float, float, float, float]:
        """Returns the current point (lat, lon, alt, body roll/pitch/yaw)"""
        
        return (self._current_lat,
                self._current_lon,
                self._current_alt,
                self._current_body_roll_r,
                self._current_body_pitch_r,
                self._current_body_yaw_r)

    @staticmethod
    def _normalize_angle_r(angle_r: float) -> float:
        """Normalize angle in radians to [-pi, pi]"""
    
        return (angle_r + math.pi) % (2.0 * math.pi) - math.pi

    def _send_current_pose(self) -> None:
        """Send the current pose (lat, lon, alt, body roll/pitch/yaw) once."""

        self.send_once(self._current_lat,
                       self._current_lon,
                       self._current_alt,
                       self._current_body_roll_r,
                       self._current_body_pitch_r,
                       self._current_body_yaw_r)
        
    def _update_orientation_world_r(self, roll_r: float, pitch_r: float, yaw_r: float) -> None:
        """Update world-frame orientation and recompute body-frame orientation"""
        
        self._current_world_roll_r = roll_r
        self._current_world_pitch_r = pitch_r
        self._current_world_yaw_r = yaw_r

        # Isaac path (GUI) uses world angles directly as body angles
        self._current_body_roll_r = roll_r
        self._current_body_pitch_r = pitch_r
        self._current_body_yaw_r = yaw_r

    def turn_to_point(self, target_lat: float, target_lon: float, target_alt: float, duration_s: float = 1.0,) -> None:
        """Smoothly rotate in world frame to look at the given target LLA."""

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

        yaw_r = math.atan2(dy, dx)

        horizontal_dist = math.sqrt(dx * dx + dy * dy)
        pitch_r = math.atan2(-dz, horizontal_dist)

        yaw_r = self._normalize_angle_r(yaw_r)
        pitch_r = self._normalize_angle_r(pitch_r)

        roll_d = math.degrees(self._current_world_roll_r)
        pitch_d = math.degrees(pitch_r)
        yaw_d = math.degrees(yaw_r)

        self.turn_to(
            target_roll_d=roll_d,
            target_pitch_d=pitch_d,
            target_yaw_d=yaw_d,
            duration_s=duration_s,
        )

    def move_to_point(self, target_lat: float, target_lon: float, target_alt: float,
                      target_roll_d: float, target_pitch_d: float, target_yaw_d: float,
                      duration_s: float = 1.0, look_at_target: bool = True, turn_duration_s: float = 0.5) -> None:
        """Moves the bot to a new point by smoothly transitioning from the current to the target point"""

        if look_at_target:
            self.turn_to_point(target_lat, target_lon, target_alt, duration_s=turn_duration_s)

        steps = max(1, int(self.send_rate_hz * duration_s))
        dt = 1.0 / self.send_rate_hz
        start_time = time.perf_counter()

        start_lat = self._current_lat
        start_lon = self._current_lon
        start_alt = self._current_alt
        start_roll_r = self._current_world_roll_r
        start_pitch_r = self._current_world_pitch_r
        start_yaw_r = self._current_world_yaw_r

        target_roll_r = math.radians(target_roll_d)
        target_pitch_r = math.radians(target_pitch_d)
        target_yaw_r = math.radians(target_yaw_d)

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
            lat = start_lat + d_lat
            lon = start_lon + d_lon
            alt = z

            if look_at_target:
                roll_r = start_roll_r
                pitch_r = start_pitch_r
                yaw_r = start_yaw_r
            else:
                d_roll_r = self._normalize_angle_r(target_roll_r - start_roll_r)
                d_pitch_r = self._normalize_angle_r(target_pitch_r - start_pitch_r)
                d_yaw_r = self._normalize_angle_r(target_yaw_r - start_yaw_r)

                roll_r = start_roll_r + alpha * d_roll_r
                pitch_r = start_pitch_r + alpha * d_pitch_r
                yaw_r = start_yaw_r + alpha * d_yaw_r
                yaw_r = self._normalize_angle_r(yaw_r)

            self._current_lat = lat
            self._current_lon = lon
            self._current_alt = alt
            self._update_orientation_world_r(roll_r, pitch_r, yaw_r)

            self._send_current_pose()

            next_time = start_time + i * dt
            now = time.perf_counter()
            sleep_time = next_time - now
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        if look_at_target:
            self.turn_to(target_roll_d, target_pitch_d, target_yaw_d, duration_s=turn_duration_s)

    def turn_to(self, target_roll_d: float, target_pitch_d: float, target_yaw_d: float, duration_s: float = 1.0) -> None:
        """Turns the bot to a new orientation by smoothly transitioning from the current to the target point, in world frame"""

        steps = max(1, int(self.send_rate_hz * duration_s))
        dt = 1.0 / self.send_rate_hz
        start_time = time.perf_counter()

        start_roll_r = self._current_world_roll_r
        start_pitch_r = self._current_world_pitch_r
        start_yaw_r = self._current_world_yaw_r

        target_roll_r = math.radians(target_roll_d)
        target_pitch_r = math.radians(target_pitch_d)
        target_yaw_r = math.radians(target_yaw_d)

        delta_roll_r = self._normalize_angle_r(target_roll_r - start_roll_r)
        delta_pitch_r = self._normalize_angle_r(target_pitch_r - start_pitch_r)
        delta_yaw_r = self._normalize_angle_r(target_yaw_r - start_yaw_r)

        for i in range(1, steps + 1):
            alpha = i / steps

            roll_r = start_roll_r + alpha * delta_roll_r
            pitch_r = start_pitch_r + alpha * delta_pitch_r
            yaw_r = start_yaw_r + alpha * delta_yaw_r
            yaw_r = self._normalize_angle_r(yaw_r)

            self._update_orientation_world_r(roll_r, pitch_r, yaw_r)
            self._send_current_pose()

            next_tick = start_time + i * dt
            now = time.perf_counter()
            sleep_time = next_tick - now
            if sleep_time > 0:
                time.sleep(sleep_time)

    
    def move_forward(self, distance_m: float, duration_s: float = 1.0) -> None:
        """Moves the bot by a certain distance in the direction it is currently facing"""

        yaw_r = self._current_world_yaw_r

        dx = distance_m * math.cos(yaw_r)
        dy = distance_m * math.sin(yaw_r)

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
        """Moves the bot by a certain distance to the right with respect to the direction it is currently facing"""

        yaw_r = self._current_world_yaw_r - math.pi / 2.0

        dx = distance_m * math.cos(yaw_r)
        dy = distance_m * math.sin(yaw_r)
        
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
        """Moves the bot by a certain distance up with respect to the direction it is currently facing"""

        target_alt = self._current_alt + distance_m

        self.move_to_point(target_lat=self._current_lat,
                    target_lon=self._current_lon,
                    target_alt=target_alt,
                    target_roll_d=math.degrees(self._current_world_roll_r),
                    target_pitch_d=math.degrees(self._current_world_pitch_r),
                    target_yaw_d=math.degrees(self._current_world_yaw_r),
                    duration_s=duration_s,
                    look_at_target=False)
    
    def turn_roll(self, delta_roll_d: float, duration_s: float = 1.0) -> None:
        """Turns the bot by a certain angle on the roll axis in world frame"""

        delta_roll_r = math.radians(delta_roll_d)

        R_current = euler2mat(self._current_world_roll_r,
                              self._current_world_pitch_r,
                              self._current_world_yaw_r,
                              axes='rxyz')
        R_delta = euler2mat(delta_roll_r, 0.0, 0.0, axes='rxyz')
        R_new = R_delta @ R_current

        roll_r, pitch_r, yaw_r = mat2euler(R_new, axes='rxyz')

        self.turn_to(target_roll_d=math.degrees(roll_r),
                     target_pitch_d=math.degrees(pitch_r),
                     target_yaw_d=math.degrees(yaw_r),
                     duration_s=duration_s)
        
    def turn_pitch(self, delta_pitch_d: float, duration_s: float = 1.0) -> None:
        """Turns the bot by a certain angle on the pitch axis in world frame"""

        delta_pitch_r = math.radians(delta_pitch_d)

        R_current = euler2mat(self._current_world_roll_r,
                              self._current_world_pitch_r,
                              self._current_world_yaw_r,
                              axes='rxyz')
        R_delta = euler2mat(0.0, delta_pitch_r, 0.0, axes='rxyz')
        R_new = R_delta @ R_current

        roll_r, pitch_r, yaw_r = mat2euler(R_new, axes='rxyz')

        self.turn_to(target_roll_d=math.degrees(roll_r),
                     target_pitch_d=math.degrees(pitch_r),
                     target_yaw_d=math.degrees(yaw_r),
                     duration_s=duration_s)

    def turn_yaw(self, delta_yaw_d: float, duration_s: float = 1.0) -> None:
        """Turns the bot by a certain angle on the yaw axis in world frame"""

        delta_yaw_r = math.radians(delta_yaw_d)

        R_current = euler2mat(self._current_world_roll_r,
                              self._current_world_pitch_r,
                              self._current_world_yaw_r,
                              axes='rxyz')
        R_delta = euler2mat(0.0, 0.0, delta_yaw_r, axes='rxyz')
        R_new = R_delta @ R_current

        roll_r, pitch_r, yaw_r = mat2euler(R_new, axes='rxyz')

        self.turn_to(target_roll_d=math.degrees(roll_r),
                     target_pitch_d=math.degrees(pitch_r),
                     target_yaw_d=math.degrees(yaw_r),
                     duration_s=duration_s)
