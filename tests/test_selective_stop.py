"""Stop-and-shoot missions stop only where the gimbal pans for a second photo."""

from flight_planner.kmz_builder import _single_shot_can_pass
from flight_planner.models import MissionConfig, Waypoint


def _row(spacing: float, speed: float, n: int = 4) -> list[Waypoint]:
    return [Waypoint(x=i * spacing, y=0.0, z=5.0, speed_ms=speed, heading_deg=0.0) for i in range(n)]


def test_single_shot_waypoints_pass_when_legs_are_long_enough():
    wps = _row(spacing=3.0, speed=2.0)          # 1.5 s legs
    assert _single_shot_can_pass(wps, MissionConfig(stop_at_waypoint=True)) == [True] * 4


def test_two_shot_waypoint_keeps_its_stop():
    wps = _row(spacing=3.0, speed=2.0)
    wps[1].extra_facade_indices = [7]
    assert _single_shot_can_pass(wps, MissionConfig(stop_at_waypoint=True))[1] is False


def test_short_legs_keep_the_stop():
    wps = _row(spacing=1.47, speed=2.0)         # 0.74 s legs: July lost photos at 0.58 s
    assert _single_shot_can_pass(wps, MissionConfig(stop_at_waypoint=True)) == [False] * 4


def test_big_heading_change_keeps_the_stop():
    wps = _row(spacing=3.0, speed=2.0)          # 1.5 s at 60°/s covers 90°
    wps[2].heading_deg = 120.0
    passes = _single_shot_can_pass(wps, MissionConfig(stop_at_waypoint=True))
    assert passes[1] is False and passes[2] is False and passes[3] is False
    assert passes[0] is True
