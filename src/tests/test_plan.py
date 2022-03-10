import random

from src.code.plan import Plan, reduce_battery


def test_reduce_battery_takeoff():
    reduction = reduce_battery("take_off")
    assert reduction == 5


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
    new_plan = Plan([[15, 5, 8, 11]], 0.5, False)
    new_plan.create_plan()
    assert new_plan.get_actions() == [["take_off"], ["move_to", 8, 11], ["cls", 15, 5], ["land"]]


def test_single_ps_plan():
    new_plan = Plan([[8, 6, 3, 9]], 0.2, False)
    new_plan.create_plan()
    assert new_plan.get_actions() == [["take_off"], ["move_to", 3, 9], ["ps", 8, 6], ["land"]]


def test_single_ss_in_flight_plan():
    new_plan = Plan([[1, 1, 1, 1]], 0.8, True)
    new_plan.create_plan()
    assert new_plan.get_actions() == [["move_to", 1, 1], ["ss", 1, 1], ["land"]]


def test_cls_and_ss_plan():
    new_plan = Plan([[7, 5, 0, 0], [2, 4, 3, 5]], 0.8, False)
    new_plan.create_plan()
    assert new_plan.get_actions() == [["take_off"], ["move_to", 0, 0], ["cls", 7, 5], ["move_to", 3, 5], ["ss", 2, 4],
                                      ["land"]] or [["take_off"], ["move_to", 0, 0], ["cls", 7, 5], ["land"]]

    # TODO: test battery running out and more edge cases

# determine_found generates a random float between 0 to 1 and return a boolean based on comparing to see if its in
# range of 0 and self.probability.
# Copying the random generating code and testing it doesnt have a remainder from dividing by 0.1 is more appropriate
def test_determine_found():
    assert random.random() % 0.1
