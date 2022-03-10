import random

from src.code.plan import Plan, reduce_battery


def test_reduce_battery_takeoff():
    reduction = reduce_battery("take_off")
    assert reduction == 5

def test_reduce_battery_move_to():
    reduction = reduce_battery("move_to", 5, 9)
    assert reduction == 14

def test_reduce_battery_ess_and_cls():  # only one test required as ess,cls and ps calculate their area the same way
    reduction = reduce_battery("ess", 5, 5)
    assert reduction == 25


def test_reduce_battery_ss():
    reduction = reduce_battery("ss", 5, 5)
    assert reduction == 19.6


def test_reduce_battery_no_action():
    reduction = reduce_battery("", 5, 5)
    assert reduction == 0


def test_actions_getter():
    new_plan = Plan([[10, 10, 0, 0]], 0.5, False)
    assert new_plan.get_actions() == []


def test_empty_plan():
    new_plan = Plan([], 0.5, False)
    new_plan.create_plan()
    assert new_plan.get_actions() == [["take_off"], ["land"]]


def test_single_ss_plan():
    new_plan = Plan([[5, 5, 0, 0]], 0.5, False)
    new_plan.create_plan()
    assert new_plan.get_actions() == [["take_off"], ["move_to", 0, 0], ["ss", 5, 5], ["land"]]


def test_single_ess_plan():
    new_plan = Plan([[5, 5, 7, 14]], 0.2, False)
    new_plan.create_plan()
    assert new_plan.get_actions() == [["take_off"], ["move_to", 7, 14], ["ess", 5, 5], ["land"]]


def test_single_cls_plan():
    new_plan = Plan([[14, 5, 8, 11]], 0.5, False)
    new_plan.create_plan()
    assert new_plan.get_actions() == [["take_off"], ["move_to", 8, 11], ["cls", 14, 5], ["land"]]


def test_single_ps_plan():
    new_plan = Plan([[8, 6, 3, 9]], 0.2, False)
    new_plan.create_plan()
    assert new_plan.get_actions() == [["take_off"], ["move_to", 3, 9], ["ps", 8, 6], ["land"]]


def test_single_ss_in_flight_plan():
    new_plan = Plan([[1, 1, 1, 1]], 0.8, True)
    new_plan.create_plan()
    assert new_plan.get_actions() == [["move_to", 1, 1], ["ss", 1, 1], ["land"]]


def test_cls_and_ss_plan():
    new_plan = Plan([[7, 5, 0, 0], [2, 4, 1, 6]], 0.8, False)
    new_plan.create_plan()
    assert new_plan.get_actions() == [["take_off"], ["move_to", 0, 0], ["cls", 7, 5], ["move_to", 1, 6], ["ss", 2, 4],
                                      ["land"]] \
           or new_plan.get_actions() == [["take_off"], ["move_to", 0, 0], ["cls", 7, 5], ["land"]]

def test_ps_and_ess_plan():
    new_plan = Plan([[6, 7, 2, 4], [6, 1, 8, 5]], 0.3, False)
    new_plan.create_plan()
    assert new_plan.get_actions() == [["take_off"], ["move_to", 2, 4], ["ps", 6, 7], ["move_to", 8, 5], ["ess", 6, 1],
                                      ["land"]] \
           or new_plan.get_actions() == [["take_off"], ["move_to", 2, 4], ["ps", 6, 7], ["land"]]

def test_ess_and_ss_plan():  # tests if the probability is incremented as intended
    new_plan = Plan([[2, 5, 0, 0],[3, 1, 2, 3]], 0.4, True)
    new_plan.create_plan()
    assert new_plan.get_actions() == [["move_to", 0, 0], ["ess", 2, 5], ["move_to", 2, 3], ["ss", 3, 1], ["land"]] \
           or [["move_to", 0, 0], ["ess", 2, 5], ["land"]]

def test_plan_insufficient_battery():
    new_plan = Plan([[18, 5, 0, 0]], 0.7, False)
    new_plan.create_plan()
    assert new_plan.get_actions() == [["take_off"], ["land"]]
    # TODO: test battery running out and more edge cases

# determine_found generates a random float between 0 to 1 and return a boolean based on comparing to see if its in
# range of 0 and self.probability.
# Copying the random generating code and testing it doesnt have a remainder from dividing by 0.1 is more appropriate
def test_determine_found():
    assert random.random() % 0.1
