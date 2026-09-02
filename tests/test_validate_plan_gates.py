"""Plan-time gates that would have flagged the 2026-07-10 flight.

Measured on that flight (docs/flights/2026-07-10-second-custom-flight/ANALYSIS.md,
2026-09-02 addendum):

- 3.0 m/s over a 1.74 m median waypoint spacing = 0.58 s per waypoint in
  fly-through mode; 104 of 398 commanded photos never fired (FC action
  callbacks: 837 starts, 733 completions).
- Commanded heading steps of 28° p90 / 82° max per waypoint at 0.6 s per leg;
  the airframe's heading error was 14.4° median.
- 18.34 m standoff on the WIDE lens = 5.01 mm/px against a 2.0 target.

WPML's `toPointAndStopWithContinuityCurvature` ("the aircraft will stop at the
point") is DJI's mechanism for letting the action chain and the turn complete,
so the dwell/heading gates only fire in fly-through mode.
"""
from __future__ import annotations

import numpy as np

from flight_planner.models import (
    ActionType,
    Building,
    CameraAction,
    CameraName,
    Facade,
    MissionConfig,
    Waypoint,
)
from flight_planner.validate import median_gsd_mm_per_px, validate_mission


def _wp(i: int, x: float, *, y: float = 0.0, heading: float = 0.0, speed: float = 3.0,
        facade: int = 0) -> Waypoint:
    return Waypoint(
        x=x, y=y, z=10.0, heading_deg=heading, gimbal_pitch_deg=-20.0, speed_ms=speed,
        facade_index=facade, index=i,
        actions=[CameraAction(action_type=ActionType.TAKE_PHOTO)],
    )


def _facade_at(y: float) -> Facade:
    """4 x 4 m wall in the x-z plane at north offset ``y``, facing south."""
    v = np.array([[-2.0, y, 8.0], [2.0, y, 8.0], [2.0, y, 12.0], [-2.0, y, 12.0]])
    return Facade(vertices=v, normal=np.array([0.0, -1.0, 0.0]), component_tag="21.1",
                  label="north_wall_0", index=0)


def _codes(issues) -> set[str]:
    return {i.code for i in issues}


# --- action dwell -----------------------------------------------------------

def test_fly_through_at_3ms_over_1p74m_flags_action_dwell():
    wps = [_wp(0, 0.0), _wp(1, 1.74), _wp(2, 3.48)]
    cfg = MissionConfig(flight_speed_ms=3.0, stop_at_waypoint=False)
    issues = validate_mission(wps, cfg)
    assert "action_dwell_too_short" in _codes(issues)
    issue = next(i for i in issues if i.code == "action_dwell_too_short")
    assert issue.waypoint_indices == [1, 2]


def test_stop_at_waypoint_clears_action_dwell_warning():
    wps = [_wp(0, 0.0), _wp(1, 1.74), _wp(2, 3.48)]
    cfg = MissionConfig(flight_speed_ms=3.0, stop_at_waypoint=True)
    assert "action_dwell_too_short" not in _codes(validate_mission(wps, cfg))


def test_slow_fly_through_with_enough_dwell_is_not_flagged():
    wps = [_wp(0, 0.0, speed=1.0), _wp(1, 1.74, speed=1.0), _wp(2, 3.48, speed=1.0)]
    cfg = MissionConfig(flight_speed_ms=1.0, stop_at_waypoint=False)
    assert "action_dwell_too_short" not in _codes(validate_mission(wps, cfg))


def test_dwell_threshold_is_a_config_knob():
    wps = [_wp(0, 0.0), _wp(1, 1.74)]
    cfg = MissionConfig(flight_speed_ms=3.0, stop_at_waypoint=False, min_action_dwell_s=0.3)
    assert "action_dwell_too_short" not in _codes(validate_mission(wps, cfg))


# --- heading reachability ---------------------------------------------------

def test_heading_step_faster_than_yaw_rate_is_flagged_in_fly_through():
    wps = [_wp(0, 0.0, heading=0.0), _wp(1, 1.74, heading=82.0)]
    cfg = MissionConfig(flight_speed_ms=3.0, stop_at_waypoint=False, yaw_rate_deg_per_s=60.0)
    issues = validate_mission(wps, cfg)
    assert "heading_step_unreachable" in _codes(issues)


def test_small_heading_step_is_not_flagged():
    wps = [_wp(0, 0.0, heading=0.0), _wp(1, 1.74, heading=5.0)]
    cfg = MissionConfig(flight_speed_ms=3.0, stop_at_waypoint=False, yaw_rate_deg_per_s=60.0)
    assert "heading_step_unreachable" not in _codes(validate_mission(wps, cfg))


def test_heading_step_not_flagged_when_stopping_at_waypoints():
    wps = [_wp(0, 0.0, heading=0.0), _wp(1, 1.74, heading=82.0)]
    cfg = MissionConfig(flight_speed_ms=3.0, stop_at_waypoint=True, yaw_rate_deg_per_s=60.0)
    assert "heading_step_unreachable" not in _codes(validate_mission(wps, cfg))


def test_heading_step_wraps_across_north():
    wps = [_wp(0, 0.0, heading=350.0), _wp(1, 1.74, heading=5.0)]  # 15°, not 345°
    cfg = MissionConfig(flight_speed_ms=3.0, stop_at_waypoint=False, yaw_rate_deg_per_s=60.0)
    assert "heading_step_unreachable" not in _codes(validate_mission(wps, cfg))


# --- GSD ---------------------------------------------------------------------

def test_gsd_out_of_spec_at_18m_on_wide_suggests_a_longer_lens():
    building = Building(facades=[_facade_at(18.34)])
    wps = [_wp(0, 0.0), _wp(1, 1.0)]
    cfg = MissionConfig(target_gsd_mm_per_px=2.0, camera=CameraName.WIDE, stop_at_waypoint=True)
    issues = validate_mission(wps, cfg, building=building)
    issue = next(i for i in issues if i.code == "gsd_out_of_spec")
    assert "5.0" in issue.message and "2.0" in issue.message
    assert "medium_tele" in issue.message


def test_gsd_in_spec_at_7m_is_not_flagged():
    building = Building(facades=[_facade_at(7.0)])
    wps = [_wp(0, 0.0), _wp(1, 1.0)]
    cfg = MissionConfig(target_gsd_mm_per_px=2.0, camera=CameraName.WIDE, stop_at_waypoint=True)
    assert "gsd_out_of_spec" not in _codes(validate_mission(wps, cfg, building=building))


def test_median_gsd_helper():
    wps = [_wp(0, 0.0), _wp(1, 1.0)]
    assert median_gsd_mm_per_px(wps, [_facade_at(18.34)], CameraName.WIDE) is not None
    assert abs(median_gsd_mm_per_px(wps, [_facade_at(18.34)], CameraName.WIDE) - 5.01) < 0.05
    assert median_gsd_mm_per_px([_wp(0, 0.0, facade=-1)], [_facade_at(18.34)], CameraName.WIDE) is None
