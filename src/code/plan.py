#!/usr/bin/env python
import math
import random
import pandas as pd
from datetime import datetime


# Current implementation infers max size of all partitions must be less than 90 units^2 or the battery will deplete mid
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

    def __init__(self, partitions, probability, in_flight):
        self.__partitions = partitions
        self.__probability = probability
        self.__battery = 100
        self.__in_flight = in_flight
        self.__found = False
        self.__actions = []
        self.__time_of_creation = None

    # randomise a float between 0 to 1 to 1 dp
    # if random number is between 0 and self.probability then found
    # if not self.probability += 0.1
    def determine_found(self):
        if random.random() <= self.__probability:
            return True
        else:
            self.__probability += 0.1  # increase probability if room has been explored but target not been found
            return False

    # Using a set of conditionals determine the ideal pattern for the current room Actions are returned to
    # create_plan if there is sufficient battery for the actions that will be added
    # Contextual move to coordinates allow for patterns that concern themselves with a datum start from the center
    # the room, as well as this ps should be started from the top left of the room
    def populate_actions(self, boundary_x, boundary_y, move_to_x, move_to_y):
        area_of_room = boundary_x * boundary_y
        if area_of_room >= 35:  # determining if a room is of a large size using 35m^2 as the indicator of a large room
            if self.__probability < 0.5:
                action = "ps"
                contextual_move_to_x = move_to_x
                contextual_move_to_y = move_to_y + boundary_y
            else:
                action = "cls"
                contextual_move_to_x = move_to_x
                contextual_move_to_y = move_to_y
        else:
            if self.__probability < 0.5:
                action = "ess"
                contextual_move_to_x = move_to_x + (boundary_x / 2)
                contextual_move_to_y = move_to_y + (boundary_y / 2)
            else:
                action = "ss"
                contextual_move_to_x = move_to_x + (boundary_x / 2)
                contextual_move_to_y = move_to_y + (boundary_y / 2)
        movement_reduction = reduce_battery("move_to", contextual_move_to_x, contextual_move_to_y)
        reduction = reduce_battery(action, boundary_x, boundary_y) + movement_reduction
        if self.__battery - reduction <= 5:
            return "land", None, None
        else:
            self.__battery -= reduction
            return action, contextual_move_to_x, contextual_move_to_y

    # go through room by room until found or exhausted
    # partitions have the format [[boundary_x, boundary_y, partition_starting_x, partition_starting_y] ...]
    def create_plan(self):
        self.__time_of_creation = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
        if not self.__in_flight:
            if self.__battery >= 10:
                self.__battery -= reduce_battery("take_off")
                self.__in_flight = True
                self.__actions.append(["take_off"])
            else:
                self.__actions.append(["not enough __battery to safely take off and land"])

        for partition in self.__partitions:
            action, contextual_x, contextual_y = self.populate_actions(partition[0], partition[1], partition[2],
                                                                       partition[3])
            if action == "land":
                break
            elif self.determine_found():
                self.__actions.append(["move_to", contextual_x, contextual_y])
                self.__actions.append([action, partition[0], partition[1]])
                self.__found = True
                break
            else:
                self.__actions.append(["move_to", contextual_x, contextual_y])
                self.__actions.append([action, partition[0], partition[1]])

        self.__actions.append(["land"])
        self.__battery -= reduce_battery("land")
        self.__in_flight = False

    def export_plan(self):
        df = pd.read_xml(r"/home/evana/catkin_ws/src/Final-year-project/src/exported plans/Plans")
        df.loc[len(df.index)] = [self.__time_of_creation, datetime.now().strftime("%d/%m/%Y %H:%M:%S"),
                                 ",".join(str(partition) for partition in self.__partitions),
                                 ",".join(str(action) for action in self.__actions),
                                 self.__battery, self.__found]
        df.to_xml(path_or_buffer=r"/home/evana/catkin_ws/src/Final-year-project/src/exported plans/Plans", index=False)

    # self.__partitions = partitions
    # self.__probability = probability
    # self.__battery = 100
    # self.__in_flight = in_flight
    # self.__found = False
    # self.__actions = []
    # self.__time_of_creation = None

    def get_partitions(self):
        return self.__partitions

    def get_probability(self):
        return self.__probability

    def get_battery(self):
        return self.__battery

    def get_in_flight(self):
        return self.__in_flight

    def get_found(self):
        return self.__found

    def get_actions(self):
        return self.__actions

    def get_time_of_creation(self):
        return self.__time_of_creation

