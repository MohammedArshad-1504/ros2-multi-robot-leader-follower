#!/usr/bin/env python3

import math
from enum import Enum


class FormationState(Enum):
    WAITING = 'WAITING'
    FOLLOW = 'FOLLOW'
    CATCH_UP = 'CATCH_UP'
    REJOIN = 'REJOIN'
    HOLD = 'HOLD'
    STUCK = 'STUCK'


class FormationManager:
    def __init__(
        self,
        spacing: float,
        slot_tolerance: float,
        rejoin_distance: float,
        max_catchup_speed: float,
    ):
        self.spacing = spacing
        self.slot_tolerance = slot_tolerance
        self.rejoin_distance = rejoin_distance
        self.max_catchup_speed = max_catchup_speed

        self.follower_number = 1
        self.state = FormationState.WAITING

    def set_follower_number(self, follower_number: int):
        self.follower_number = max(1, int(follower_number))

    def slot_distance(self) -> float:
        return self.follower_number * self.spacing

    def calculate_slot_in_leader_frame(self, leader_x, leader_y):
        distance = self.slot_distance()
        return (
            leader_x - distance,
            leader_y,
        )

    def classify(self, target_distance: float):
        error = abs(target_distance)

        if error <= self.slot_tolerance:
            self.state = FormationState.HOLD
        elif error >= self.rejoin_distance:
            self.state = FormationState.REJOIN
        else:
            self.state = FormationState.CATCH_UP

        return self.state

    def velocity_scale(self, distance: float):
        if self.state == FormationState.REJOIN:
            return min(
                1.0,
                max(
                    0.5,
                    distance / max(self.rejoin_distance, 1e-6),
                ),
            )

        if self.state == FormationState.CATCH_UP:
            return min(
                1.0,
                max(
                    0.25,
                    distance / max(self.slot_distance(), 1e-6),
                ),
            )

        return 0.0

    @staticmethod
    def clamp_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

