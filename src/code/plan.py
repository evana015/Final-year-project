#!/usr/bin/env python
import math
import random

# Current implementation infers max size of all rooms must be less than 90 units^2 or the battery will deplete mid
# plan due to the 5 percent reserved for take_off and landing.
# Battery level decreases at a rate of 1 unit^2 == -1% as I dont have the capability to do real world tests
def reduce_battery(action, boundary_x=1, boundary_y=1):
    switcher = {
        "take_off": 5,
        "land": 5,
        "move_to": boundary_x + boundary_y,
        "ess": boundary_x * boundary_y,
        "cls": boundary_x * boundary_y,
        "ps": boundary_x * boundary_y,
        "ss": round((math.pi * (boundary_x / 2) ** 2), 1)
    }
    return switcher.get(action, 0)


class Plan:

    def __init__(self, rooms, probability, in_flight):
        self.rooms = rooms
        self.probability = probability
        self.battery = 100
        self.in_flight = in_flight
        self.found = False
        self.actions = []

    # randomise a float between 0 to 1 to 1 dp
    # if random number is between 0 and self.probability then found
    # if not self.probability += 0.1
    def determine_found(self):
        if random.random() <= self.probability:
            return True
        else:
            self.probability += 0.1  # increase probability if room has been explored but target not been found
            return False

    def populate_actions(self, boundary_x, boundary_y, move_to_x, move_to_y):
        area_of_room = boundary_x * boundary_y
        if area_of_room >= 35:  # determining if a room is of a large size using 35m^2 as the indicator of a large room
            if self.probability < 0.5:
                action = "ps"
            else:
                action = "cls"
        else:
            if self.probability < 0.5:
                action = "ess"
            else:
                action = "ss"
        movement_reduction = reduce_battery("move_to", move_to_x, move_to_y)
        reduction = reduce_battery(action, boundary_x, boundary_y) + movement_reduction
        if self.battery - reduction <= 5:
            return "land"
        else:
            self.battery -= reduction
            return action

    # go through room by room until found or exhausted
    # rooms have the format [[boundary_x, boundary_y, room_starting_x, room_starting_y] ...]
    def create_plan(self):
        if not self.in_flight:
            if self.battery >= 10:
                self.battery -= reduce_battery("take_off")
                self.in_flight = True
                self.actions.append(["take_off"])
            else:
                self.actions.append(["not enough battery to take off and safely land"])  # TODO: special case

        for room in self.rooms:
            action = self.populate_actions(room[0], room[1], room[2], room[3])
            if action == "land":
                break
            elif self.determine_found():
                self.actions.append(["move_to", room[2], room[3]])
                self.actions.append([action, room[0], room[1]])
                self.found = True
                break
            else:
                self.actions.append(["move_to", room[2], room[3]])
                self.actions.append([action, room[0], room[1]])

        self.actions.append(["land"])
        self.battery -= reduce_battery("land")
        self.in_flight = False

    def get_actions(self):
        return self.actions
