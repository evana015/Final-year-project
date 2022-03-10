#!/usr/bin/env python
import math


# Planner TODO: decreasing and testing battery level (set 1 unit ^2 = -1%)
#               increase probability per iteration where not found
#               add a weighted found chance (using probability found)
#               take off and land must be in the plan but can be tested for on initiation (if hovering etc)
#               Take a dictionary/list of rooms that will be iterated through, each room will have an ideal pattern
#               If not found move onto the next room, if last room and not found. Failed search and land
#               implement flow chart as rules

class Plan:

    def __init__(self, rooms, probability):
        self.rooms = rooms
        self.probability = probability
        self.battery = 100
        self.in_flight = False
        self.actions = []

    def reduce_battery(self, action, boundary_x=1, boundary_y=1):  # current implementation infers max size of all
        # rooms must be less than 80 units^2 or the battery will deplete mid plan
        switcher = {
            "take_off": 5,
            "land": 5,
            "ess": boundary_x * boundary_y,
            "cls": boundary_x * boundary_y,
            "ss": round((math.pi * (boundary_x / 2) ** 2), 1)
        }
        self.battery -= switcher.get(action, 0)

    def determine_found(self):
        # randomise a number between 0 to 1 to 1 dp
        # if random number is between 0 and self.probability then found
        # if not self.probability += 0.1
        return None

    def populate_actions(self):
        # set of rules aka the flow chart
        return None

    def iterate_rooms(self):
        # go through room by room until found or exhausted
        # rooms have the format [[boundary_x, boundary_y, bottom_left_co-ordinate AS (x,y)] ...]
        # moving rooms costs battery so determine that
        return None