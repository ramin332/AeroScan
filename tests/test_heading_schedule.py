"""Heading scheduling: keep the gimbal's required pan inside its ±60° travel.

Measured on the 2026-07-10 flight (JPEG XMP ground truth, 294 photos):

    commanded gimbal yaw == commanded heading at every waypoint (pan_cmd = 0)
    actual  |GimbalYaw - FlightYaw|  median 51.5°, p90 61.0°, 41/294 at the stop
    actual  |heading - commanded|    median 14.4°, p90 63.9°, max 178.8°
    actual  gimbal vs commanded yaw  median 44.2° off

The commanded gimbal pose pointed at the bus roughly twice as accurately as the
gimbal ever achieved (17.6° vs 35.4° median bearing error). So the planner was
right and the aircraft could not comply: we welded the gimbal's absolute-north
yaw to a heading containing 179° reversals, `smoothTransition` spread each
reversal across a 0.58 s leg, the airframe lagged, and the gimbal absorbed the
whole error until it ran out of travel and stayed there.

`schedule_headings` fixes the cause: the heading becomes a rate-limited pursuit
of the aim bearing, hard-capped so the residual pan never exceeds
`max_gimbal_pan_deg`. The gimbal keeps its exact absolute-north aim.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from flight_planner.gimbal_rewrite import schedule_headings


def _wrap(a):
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


def test_static_bearing_gives_static_heading():
    pos = np.array([[0.0, float(i), 0.0] for i in range(10)])
    bearings = np.zeros(10)
    head, pan = schedule_headings(pos, bearings, speeds=np.full(10, 2.0))
    assert np.allclose(head, 0.0, atol=1e-6)
    assert np.allclose(pan, 0.0, atol=1e-6)


def test_pan_never_exceeds_the_cap_even_on_a_180_reversal():
    """The 2026-07-10 failure mode, in miniature."""
    pos = np.array([[0.0, float(i) * 1.75, 0.0] for i in range(20)])
    bearings = np.zeros(20)
    bearings[10:] = 179.4  # the reversal that broke the real flight
    head, pan = schedule_headings(
        pos, bearings, speeds=np.full(20, 3.0), max_gimbal_pan_deg=50.0
    )
    assert np.abs(pan).max() <= 50.0 + 1e-6, f"pan hit {np.abs(pan).max():.1f}°"


def test_heading_rate_is_bounded_where_the_cap_allows():
    """On a gentle sweep the heading must not jump; it should ramp."""
    n = 40
    pos = np.array([[0.0, float(i) * 1.75, 0.0] for i in range(n)])
    bearings = np.linspace(0.0, 90.0, n)
    head, pan = schedule_headings(
        pos, bearings, speeds=np.full(n, 3.0),
        yaw_rate_deg_per_s=60.0, rate_fraction=0.5, max_gimbal_pan_deg=50.0,
    )
    leg_t = 1.75 / 3.0
    budget = 60.0 * 0.5 * leg_t
    steps = np.abs(_wrap(np.diff(head)))
    # Allow the cap to force a bigger step, but on a gentle sweep it never should.
    assert steps.max() <= budget + 1e-6, f"heading stepped {steps.max():.1f}° > budget {budget:.1f}°"


def test_cap_wins_over_the_rate_limit_when_they_conflict():
    """A reversal cannot be tracked at the rate limit; the cap must still hold.

    This is the honest trade: we would rather yaw the airframe faster than DJI's
    smoothTransition can deliver than let the gimbal saturate. The scheduler
    reports the violation so the caller can warn.
    """
    pos = np.array([[0.0, float(i) * 1.75, 0.0] for i in range(6)])
    bearings = np.array([0.0, 0.0, 179.0, 179.0, 179.0, 179.0])
    head, pan = schedule_headings(
        pos, bearings, speeds=np.full(6, 3.0),
        yaw_rate_deg_per_s=60.0, rate_fraction=0.5, max_gimbal_pan_deg=50.0,
    )
    assert np.abs(pan).max() <= 50.0 + 1e-6
    step = abs(float(_wrap(head[2] - head[1])))
    leg_t = 1.75 / 3.0
    assert step > 60.0 * 0.5 * leg_t, "expected the cap to force a larger-than-budget yaw"


def test_scheduler_reports_rate_violations():
    pos = np.array([[0.0, float(i) * 1.75, 0.0] for i in range(6)])
    gentle = np.linspace(0, 10, 6)
    violent = np.array([0.0, 0.0, 179.0, 179.0, 179.0, 179.0])
    _, _, v_gentle = schedule_headings(pos, gentle, speeds=np.full(6, 3.0), report=True)
    _, _, v_violent = schedule_headings(pos, violent, speeds=np.full(6, 3.0), report=True)
    assert v_gentle == 0
    assert v_violent >= 1


def test_zero_speed_does_not_divide_by_zero():
    pos = np.zeros((4, 3))
    head, pan = schedule_headings(pos, np.array([0.0, 90.0, 90.0, 90.0]), speeds=np.zeros(4))
    assert np.isfinite(head).all() and np.isfinite(pan).all()


@pytest.mark.parametrize("cap", [30.0, 50.0, 60.0])
def test_cap_is_respected_on_a_full_orbit(cap):
    """A close orbit sweeps the bearing through 360°. Pan must stay capped."""
    n = 120
    ang = np.linspace(-math.pi, math.pi, n)
    pos = np.c_[8 * np.cos(ang), 8 * np.sin(ang), np.full(n, 5.0)]
    bearings = np.degrees(np.arctan2(-pos[:, 0], -pos[:, 1]))  # always face the centre
    head, pan = schedule_headings(pos, bearings, speeds=np.full(n, 3.0), max_gimbal_pan_deg=cap)
    assert np.abs(pan).max() <= cap + 1e-6
