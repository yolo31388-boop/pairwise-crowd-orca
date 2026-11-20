import pytest
from gameplay import crowd_avoidance as mod

CASES = [
    ("horizon", 'm.orca_time_horizon(0.5, 1.0) == approx(2.0)'),
    ("radius", 'm.obstacle_radius((0, 0, 0, 2, 2, 2)) == approx(1.732051, rel=1e-5)'),
    ("goal", 'm.shared_goal((10, 10), (1, 1), False) == (10, 10)'),
    ("window", 'm.adaptive_window(2.0, 1.0) == approx(0.5)'),
    ("key", 'm.stable_key(7, 3) == "7:3"'),
    ("density", 'm.density_limit(10, 8) == 8'),
    ("knock", 'm.knockback_clear((2, 3)) == (0, 0)'),
    ("force", 'm.partition_force([(2, 0), (4, 0)]) == approx((3.0, 0.0))'),
    ("slope", 'm.slope_speed(5, 60) == approx(2.5)'),
    ("predict", 'm.predict_position((0, 0), (2, 0), (0, 0), 1) == approx((2.0, 0.0))'),
    ("neighbor", 'm.neighbor_weight(2, 4) == approx(0.5)'),
    ("reached", 'm.goal_reached((0, 0), (3, 4), 5) is True'),
    ("enabled", 'm.avoidance_enabled(3, 4) is False'),
    ("door", 'm.door_capacity(4, 0.5) == 4'),
    ("clamp", 'm.velocity_clamp(3, 4, 5) == approx((3.0, 4.0))')
]

@pytest.mark.parametrize("name,expr", CASES, ids=[c[0] for c in CASES])
def test_contract(name, expr):
    assert eval(expr, {"m": mod, "approx": pytest.approx})
