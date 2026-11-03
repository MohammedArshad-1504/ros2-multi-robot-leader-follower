#!/usr/bin/env python3

import math


class ObstacleAvoidance:
    def __init__(
        self,
        stop_distance: float,
        slow_distance: float,
        inflation_radius: float,
    ):
        self.stop_distance = stop_distance
        self.slow_distance = slow_distance
        self.inflation_radius = inflation_radius

        self.front = float('inf')
        self.left = float('inf')
        self.right = float('inf')
        self.front_left = float('inf')
        self.front_right = float('inf')

    def update_scan(self, msg):
        self.front = float('inf')
        self.left = float('inf')
        self.right = float('inf')
        self.front_left = float('inf')
        self.front_right = float('inf')

        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue

            angle = msg.angle_min + index * msg.angle_increment
            distance = max(distance, 0.001)

            if abs(angle) <= math.radians(20.0):
                self.front = min(self.front, distance)
            elif math.radians(20.0) < angle <= math.radians(60.0):
                self.front_left = min(self.front_left, distance)
            elif math.radians(20.0) < -angle <= math.radians(60.0):
                self.front_right = min(self.front_right, distance)
            elif math.radians(60.0) < angle <= math.radians(120.0):
                self.left = min(self.left, distance)
            elif math.radians(60.0) < -angle <= math.radians(120.0):
                self.right = min(self.right, distance)

    def apply(self, linear, angular):
        if self.front < self.stop_distance:
            if self.front_left > self.front_right:
                return 0.0, 0.70
            return 0.0, -0.70

        if self.front < self.slow_distance:
            scale = (
                self.front - self.stop_distance
            ) / max(
                self.slow_distance - self.stop_distance,
                1e-6,
            )
            linear *= max(0.0, min(1.0, scale))

        return linear, angular

    def choose_escape_direction(self):
        left_score = min(self.front_left, self.left)
        right_score = min(self.front_right, self.right)

        if left_score >= right_score:
            return 1.0
        return -1.0

