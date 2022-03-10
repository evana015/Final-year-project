#!/usr/bin/env python
import math


# Planner TODO: decreasing and testing battery level (set 1 unit ^2 = -1%)
#               increase probability per iteration where not found
#               add a weighted found chance (using probability found)
#               take off and land must be in the plan but can be tested for on initiation (if hovering etc)
#
#               implement flow chart as rules

class Plan:

    def __init__(self, boundary_width, boundary_length, probability):
        self.boundary_width = boundary_width
        self.boundary_length = boundary_length
        self.probability = probability
        self.battery = 100
        self.in_flight = False
        self.actions = []

    def reduce_battery(self, action, boundary_x=1, boundary_y=1):
        switcher = {
            "take_off": 5,
            "land": 5,
            "ess": boundary_x * boundary_y,
            "cls": boundary_x * boundary_y,
            "ss": round((math.pi * (boundary_x / 2) ** 2), 1)
        }
        self.battery -= switcher.get(action, 0)

    def determine_found(self):
        return None

    def populate_actions(self):
        return None
