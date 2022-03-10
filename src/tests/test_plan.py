from src.code.plan import Plan


def test_reduce_battery_takeoff():
    new_plan = Plan([[10, 10, 0, 0]], 0.5, False)
    reduction = new_plan.reduce_battery("take_off")
    assert reduction == 5


def test_reduce_battery_ess_and_cls():  # only one test required as they calculate area the same way
    new_plan = Plan([[10, 10, 0, 0]], 0.5, False)
    reduction = new_plan.reduce_battery("ess", 5, 5)
    assert reduction == 25


def test_reduce_battery_ss():
    new_plan = Plan([[10, 10, 0, 0]], 0.5, False)
    reduction = new_plan.reduce_battery("ss", 5, 5)
    assert reduction == 19.6

def test_actions_getter():
    new_plan = Plan([[10, 10, 0, 0]], 0.5, False)
    assert new_plan.get_actions() == []

