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

    def __init__(self, rooms, probability, in_flight):
        self.rooms = rooms
        self.probability = probability
        self.battery = 100
        self.in_flight = in_flight
        self.found = False
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
        return switcher.get(action, 0)

    def determine_found(self):
        # randomise a number between 0 to 1 to 1 dp
        # if random number is between 0 and self.probability then found
        # if not self.probability += 0.1
        return None

    def populate_actions(self, boundary_x, boundary_y):
        action = ""  # set of rules aka the flow chart
        reduction = self.reduce_battery(action, boundary_x, boundary_y)
        if self.battery - reduction <= 5:
            return "land"
        else:
            self.battery -= reduction
            return action

    def create_plan(self):
        if not self.in_flight:
            self.reduce_battery("take_off")
            self.in_flight = True
            self.actions.append(["take_off"])

        for room in self.rooms:
            self.actions.append(["move_to", room[2], room[3]])
            action = self.populate_actions(room[0], room[1])
            if action == "land":
                self.actions.append(["land"])
                break
            elif self.determine_found():
                self.actions.append([action, room[0], room[1]])
                self.actions.append(["land"])
                self.found = True
                break
            else:
                self.actions.append([action, room[0], room[1]])

        # go through room by room until found or exhausted
        # rooms have the format [[boundary_x, boundary_y, room_starting_x, room_starting_y] ...]
        # moving rooms costs battery so determine that

    def get_actions(self):
        return self.actions
