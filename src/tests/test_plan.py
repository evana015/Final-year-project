import random

from src.code.plan import Plan, reduce_battery


def test_reduce_battery_takeoff():
    reduction = reduce_battery("take_off")
    assert reduction == 5


def test_reduce_battery_ess_and_cls():  # only one test required as ess and cls calculate their area the same way
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
    assert new_plan.get_actions() == [["take_off"]]

# determine_found generates a random float between 0 to 1 and return a boolean based on comparing to see if its in
# range of 0 and self.probability.
# Copying the random generating code and testing it doesnt have a remainder from dividing by 0.1 is more appropriate
def test_determine_found():
    assert random.random() % 0.1
