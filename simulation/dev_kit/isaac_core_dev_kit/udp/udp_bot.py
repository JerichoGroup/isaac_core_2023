"""This file defines a class to control and command a UDP robot in Isaac Sim."""

# ==================== Imports ====================
from __future__ import annotations

import math
import time
from typing import Tuple

from .base_udp_sender import BaseUDPSender


# ==================== the UdpBot class ====================
class UdpBot(BaseUDPSender):
    """controls a UDP robot in Isaac Sim."""

    def __init__(self,
                 start_lat: float, start_lon: float, start_alt: float,
                 start_roll: float, start_pitch: float, start_yaw: float,
                 udp_port: float = 33333, send_rate_hz: float = 30,
                 broadcast: bool = True, target_ip: str = "127.0.0.1"
                ) -> None:
        """Initialize the UdpBot with a starting po and an address"""

        super().__init__(udp_port=udp_port, send_rate_hz=send_rate_hz,
                         broadcast=broadcast, target_ip=target_ip)
        
        self._current_lat = start_lat
        self._current_lon = start_lon
        self._current_alt = start_alt

        self._update_orientation_world(start_roll, start_pitch,start_yaw)

        self._send_current_pose()

    def get_next_point(self) -> Tuple[float, float, float, float, float, float]:
        """Returns the current point (lat, lon, alt, body roll/pitch/yaw)"""
        
        return (self._current_lat,
                self._current_lon,
                self._current_alt,
                self._current_body_roll,
                self._current_body_pitch,
                self._current_body_yaw)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
    
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def world_frame_to_body_frame(self, world_roll: float, world_pitch: float, world_yaw: float) -> Tuple[float, float, float]:
        """Converts world-frame ENU euler angles to NED body-frame euler angles based"""

        roll_enu = world_roll
        pitch_enu = world_pitch
        yaw_enu = world_yaw

        roll_ned = pitch_enu
        pitch_ned = roll_enu
        yaw_ned = -yaw_enu + math.pi / 2.0

        yaw_ned = self._normalize_angle(yaw_ned)

        return roll_ned, pitch_ned, yaw_ned
    
    def _send_current_pose(self) -> None:
        """Send the current pose (lat, lon, alt, body roll/pitch/yaw) once."""

        self.send_once(self._current_lat,
                       self._current_lon,
                       self._current_alt,
                       self._current_body_roll,
                       self._current_body_pitch,
                       self._current_body_yaw)
        
    def _update_orientation_world(self, roll: float, pitch: float, yaw: float) -> None:
        """Update world-frame orientation and recompute body-frame orientation"""
        
        self._current_world_roll = roll
        self._current_world_pitch = pitch
        self._current_world_yaw = yaw

        (self._current_body_roll,
         self._current_body_pitch,
         self._current_body_yaw) = self.world_frame_to_body_frame(roll, pitch, yaw)

    def move_to_point(self, target_lat: float, target_lon: float, target_alt: float,
                      target_roll: float, target_pitch: float, target_yaw: float, duration_s: float = 1.0) -> None:
        """Moves the bot to a new point by smoothly transitioning from the current to the target point"""

        steps = max(1, int(self.send_rate_hz * duration_s))
        dt = 1.0 / self.send_rate_hz

        start_time = time.perf_counter()

        start_lat = self._current_lat
        start_lon = self._current_lon
        start_alt = self._current_alt
        start_roll = self._current_world_roll
        start_pitch = self._current_world_pitch
        start_yaw = self._current_world_yaw

        for i in range(1, steps + 1):
            alpha = i / steps

            lat = start_lat + alpha * (target_lat - start_lat)
            lon = start_lon + alpha * (target_lon - start_lon)
            alt = start_alt + alpha * (target_alt - start_alt)

            roll = start_roll + alpha * (target_roll - start_roll)
            pitch = start_pitch + alpha * (target_pitch - start_pitch)
            yaw = start_yaw + alpha * (target_yaw - start_yaw)

            self._current_lat = lat
            self._current_lon = lon
            self._current_alt = alt
            self._update_orientation_world(roll, pitch, yaw)

            self._send_current_pose()

            next_time = start_time + i * dt
            now = time.perf_counter()
            sleep_time = next_time - now
            if sleep_time > 0:
                time.sleep(sleep_time)

    def turn_to(self, target_roll: float, target_pitch: float, target_yaw: float, duration_s: float = 1.0) -> None:
        """Turns the bot to a new orientation by smoothly transitioning from the current to the target point, in world frame"""

        self.move_to_point(target_lat=self._current_lat,
                           target_lon=self._current_lon,
                           target_alt=self._current_alt,
                           target_roll=target_roll,
                           target_pitch=target_pitch,
                           target_yaw=target_yaw,
                           duration_s=duration_s)
    
    def move_forward(self, distance_m: float, duration_s: float = 1.0) -> None:
        """Moves the bot by a certain distance in the direction it is currently facing"""

        yaw = self._current_world_yaw

        dx = distance_m * math.cos(yaw)
        dy = distance_m * math.sin(yaw)

        target_lat = self._current_lat + dy
        target_lon = self._current_lon + dx

        self.move_to_point(target_lat=target_lat,
                           target_lon=target_lon,
                           target_alt=self._current_alt,
                           target_roll=self._current_world_roll,
                           target_pitch=self._current_world_pitch,
                           target_yaw=self._current_world_yaw,
                           duration_s=duration_s)

    def move_right(self, distance_m: float, duration_s: float = 1.0) -> None:
        """Moves the bot by a certain distance to the right with respect to the direction it is currently facing"""

        yaw = self._current_world_yaw - math.pi / 2.0

        dx = distance_m * math.cos(yaw)
        dy = distance_m * math.sin(yaw)
        
        target_lat = self._current_lat + dy
        target_lon = self._current_lon + dx

        self.move_to_point(target_lat=target_lat,
                           target_lon=target_lon,
                           target_alt=self._current_alt,
                           target_roll=self._current_world_roll,
                           target_pitch=self._current_world_pitch,
                           target_yaw=self._current_world_yaw,
                           duration_s=duration_s)

    def move_up(self, distance_m: float, duration_s: float = 1.0) -> None:
        """Moves the bot by a certain distance up with respect to the direction it is currently facing"""

        target_alt = self._current_alt + distance_m

        self.move_to_point(target_lat=self._current_lat,
                    target_lon=self._current_lon,
                    target_alt=target_alt,
                    target_roll=self._current_world_roll,
                    target_pitch=self._current_world_pitch,
                    target_yaw=self._current_world_yaw,
                    duration_s=duration_s)
