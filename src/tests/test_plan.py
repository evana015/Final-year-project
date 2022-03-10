from src.code.plan import Plan


def test_reduce_battery_takeoff():
    new_plan = Plan(10, 10, 0.5)
    new_plan.reduce_battery("take_off")
    assert new_plan.battery == 95


def test_reduce_battery_ess_and_cls():  # only one test required as they calculate area the same way
    new_plan = Plan(10, 10, 0.5)
    new_plan.reduce_battery("ess", 5, 5)
    assert new_plan.battery == 75


def test_reduce_battery_ss():
    new_plan = Plan(10, 10, 0.5)
    new_plan.reduce_battery("ss", 5, 5)
    assert new_plan.battery == 80.4
