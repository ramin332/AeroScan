"""Rewrite imported DJI waypoints for NEN-2767 perpendicular inspection.

Takes AutoExplore-generated waypoints (photogrammetry rosette, pitch ~-19°)
and rewrites gimbal pitch/yaw so the camera faces each waypoint's nearest
outward facade head-on. This is the MVP deliverable — we keep DJI's
flight-tested trajectory and waypoint spacing, and only change where the
camera points.
"""

from __future__ import annotations

import math
from dataclasses import replace
from typing import Sequence

import numpy as np

from .models import (
    GIMBAL_TILT_MAX_DEG,
    GIMBAL_TILT_MIN_DEG,
    Facade,
    Waypoint,
)


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else v


def _wrap180(a):
    return (np.asarray(a, dtype=np.float64) + 180.0) % 360.0 - 180.0


def schedule_headings(
    positions: np.ndarray,
    bearings: np.ndarray,
    speeds: np.ndarray,
    *,
    max_gimbal_pan_deg: float = 50.0,
    yaw_rate_deg_per_s: float = 60.0,
    rate_fraction: float = 0.5,
    report: bool = False,
):
    """Rate-limited aircraft heading whose residual gimbal pan is bounded.

    Returns ``(heading_deg, pan_deg)``, or ``(heading, pan, rate_violations)``
    when ``report``. ``pan`` is the gimbal yaw required relative to the airframe,
    i.e. ``bearing - heading``; it is guaranteed within ``max_gimbal_pan_deg``.

    Why this exists. The augment used to set ``heading = bearing`` at every
    waypoint, so the *commanded* pan was 0 and the gimbal's ±60° travel went
    unused. But the bearing sequence contains ~179° reversals; DJI's
    ``smoothTransition`` spreads each across one ~0.58 s leg, and the M4E yaws at
    ~60°/s, so the airframe arrives ~135° short. The gimbal, commanded in the
    absolute-north frame, then has to make up the whole difference and saturates.
    Ground truth from the 2026-07-10 flight's JPEG XMP: actual gimbal pan sat at
    a median of 51.5° with a p90 of exactly 61.0° — pinned against the stop —
    while our commanded pose pointed at the target twice as accurately as the
    gimbal ever managed.

    So: chase the bearing no faster than the airframe can turn, and where that
    would leave the gimbal needing more than ``max_gimbal_pan_deg``, yaw harder.
    The cap wins over the rate limit — a gimbal at its mechanical stop is aiming
    at nothing, whereas an aggressive yaw merely costs time. Violations of the
    rate limit are counted and returned so the caller can warn at plan time.
    """
    positions = np.asarray(positions, dtype=np.float64).reshape(-1, 3)
    bearings = np.asarray(bearings, dtype=np.float64).reshape(-1)
    speeds = np.asarray(speeds, dtype=np.float64).reshape(-1)
    n = len(bearings)
    if n == 0:
        return (np.zeros(0), np.zeros(0), 0) if report else (np.zeros(0), np.zeros(0))

    heading = np.zeros(n)
    heading[0] = bearings[0]
    violations = 0

    for i in range(1, n):
        leg = float(np.linalg.norm(positions[i] - positions[i - 1]))
        speed = float(speeds[i - 1]) if speeds[i - 1] > 0.1 else 0.1
        budget = yaw_rate_deg_per_s * rate_fraction * (leg / speed)

        want = float(_wrap180(bearings[i] - heading[i - 1]))
        step = max(-budget, min(budget, want))
        h = float(_wrap180(heading[i - 1] + step))

        pan = float(_wrap180(bearings[i] - h))
        if abs(pan) > max_gimbal_pan_deg:
            # The cap wins. Yaw further than the rate budget allows and say so.
            h = float(_wrap180(bearings[i] - math.copysign(max_gimbal_pan_deg, pan)))
            if abs(float(_wrap180(h - heading[i - 1]))) > budget + 1e-9:
                violations += 1
        heading[i] = h

    pan = _wrap180(bearings - heading)
    return (heading, pan, violations) if report else (heading, pan)


def _pick_facade_for_waypoint(
    wp_xyz: np.ndarray,
    facades: Sequence[Facade],
    max_distance_m: float,
    wall_distance_bonus: float = 0.5,
    previous_index: int | None = None,
    switch_ratio: float = 0.8,
) -> tuple[int, float] | None:
    """Find the best outward-facing facade for this waypoint.

    Returns (facade_index, 3d_distance_to_center) or None if no facade is
    within ``max_distance_m`` in 3D on its outward side.

    Gate: the WP must be on the facet's outward side (``dot(normal,
    WP - center) > 0``). Sort metric: 3D distance from the WP to the
    facet's centroid (lower wins).

    Walls (label starts with ``wall_``) get their distance multiplied by
    ``wall_distance_bonus`` (default 0.5). This biases the picker toward
    walls when both wall and non-wall (roof/tilted) are candidates at
    similar 3D range — e.g., a wall at 8 m beats a roof at 4.5 m, but
    a roof at 3 m beats a wall at 8 m.

    History: this used to gate AND sort on signed perpendicular standoff
    (the projection of WP-to-center onto the normal). That had a hidden
    failure mode — a small facet 30 m away laterally could have a tiny
    perp standoff (because the WP happens to be 'in front of' its
    infinite plane) and beat a much closer facet whose plane is slightly
    further. Verified on Mijande/flight0016: 46% of WPs were picking
    facets > 20 m away in 3D, even with ``max_distance_m=60``, because
    the picker only bounded perp standoff. Switching the sort metric to
    3D distance makes the picker prefer geometrically near facets, which
    is what 'aim at the building's nearby surface' actually means.
    """
    best: tuple[int, float] | None = None
    best_weighted = float("inf")
    prev_weighted = float("inf")
    prev_hit: tuple[int, float] | None = None

    for i, f in enumerate(facades):
        n = _unit(np.asarray(f.normal, dtype=np.float64))
        c = np.asarray(f.center, dtype=np.float64)
        signed = float(np.dot(n, wp_xyz - c))
        if signed <= 0:
            continue  # WP is on the inward side or coplanar — facet not reachable
        dist3d = float(np.linalg.norm(c - wp_xyz))
        if dist3d > max_distance_m:
            continue
        is_wall = (f.label or "").startswith("wall_")
        weighted = dist3d * (wall_distance_bonus if is_wall else 1.0)
        if i == previous_index:
            prev_weighted = weighted
            prev_hit = (i, dist3d)
        if weighted < best_weighted:
            best_weighted = weighted
            best = (i, dist3d)

    # Hysteresis. The previous facade keeps the aim unless a challenger beats it
    # by a clear margin. Without this the picker re-decides from scratch at every
    # waypoint, so two near-equal facets trade the aim back and forth; measured on
    # the 2026-07-10 flights, 32% of adjacent waypoints switched target and four
    # of the switches were ~175° reversals to a facet on the opposite side. The
    # aircraft cannot yaw that fast, so the north-referenced gimbal command
    # saturates against the ±60° pan limit and the gimbal appears to lock.
    #
    # A previous facade that is now behind the waypoint or out of range never
    # reaches `prev_hit`, so it is dropped without argument.
    if prev_hit is not None and best is not None and best[0] != previous_index:
        if best_weighted > prev_weighted * switch_ratio:
            return prev_hit

    return best


def plane_groups(
    facades: Sequence[Facade],
    *,
    parallel_tol_deg: float = 5.0,
    coplanar_tol_m: float = 0.5,
) -> list[int]:
    """Group index per facet: facets that lie in (nearly) the same plane share a
    group. Two stacked slices of one wall are one target for the switch cost —
    hopping between them changes pitch, not where the aircraft looks — while a
    perpendicular wall round the corner is a different target.

    Why: CGAL region growing is tuned for many small facets (NEN-2767 wants the
    gimbal to square up on sills and panels), so a 7 m wall arrives as ~7 slices
    1–2 m² each. On busboom the greedy picker hopped between those slices and
    that alone accounted for a large share of its 76 target switches.
    """
    n = len(facades)
    parent = list(range(n))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    N = np.array([_unit(np.asarray(f.normal, float)) for f in facades]).reshape(n, 3)
    C = np.array([np.asarray(f.center, float) for f in facades]).reshape(n, 3)
    cos_tol = math.cos(math.radians(parallel_tol_deg))
    for i in range(n):
        for j in range(i + 1, n):
            if abs(float(N[i] @ N[j])) < cos_tol:
                continue
            if abs(float((C[j] - C[i]) @ N[i])) > coplanar_tol_m:
                continue
            ra, rb = find(i), find(j)
            if ra != rb:
                parent[rb] = ra
    roots = [find(i) for i in range(n)]
    remap = {r: k for k, r in enumerate(dict.fromkeys(roots))}
    return [remap[r] for r in roots]


def assign_facades_viterbi(
    waypoints: Sequence[Waypoint],
    facades: Sequence[Facade],
    *,
    max_distance_m: float,
    wall_distance_bonus: float = 0.5,
    switch_cost: float = 4.0,
    pitch_soft_deg: float = 45.0,
    groups: Sequence[int] | None = None,
) -> list[int]:
    """Choose a facet (or -1 = keep DJI's pose) for every waypoint by minimising
    the cost of the whole sequence — Viterbi over waypoints.

    Per-waypoint cost is the greedy picker's metric (3D distance, walls at
    ``wall_distance_bonus``) plus a soft penalty once the required pitch passes
    ``pitch_soft_deg``. "No target" costs ``max_distance_m``, so any reachable
    facet inside the cap beats it and nothing outside the cap is ever chosen.

    Changing target costs ``switch_cost × (1 + turn/90°)`` where ``turn`` is the
    bearing change the nose would have to make at that waypoint; facets in the
    same plane group cost nothing to switch between. That is what "look ahead and
    behind" means here: a one-waypoint decoy can never win because entering and
    leaving it costs more than it saves, and a corner is crossed once, where the
    accumulated gain finally exceeds the turn.

    Complexity O(N·F²) vectorised; 398 × 201 facets runs in well under a second.
    """
    N = len(waypoints)
    F = len(facades)
    if N == 0:
        return []
    if F == 0:
        return [-1] * N
    C = np.array([np.asarray(f.center, float) for f in facades]).reshape(F, 3)
    Nn = np.array([_unit(np.asarray(f.normal, float)) for f in facades]).reshape(F, 3)
    is_wall = np.array([(f.label or "").startswith("wall_") for f in facades])
    weight = np.where(is_wall, wall_distance_bonus, 1.0)
    grp = np.asarray(groups if groups is not None else plane_groups(facades))
    # state F == "none"; give it its own group so none↔facet counts as a switch
    G = np.append(grp, -1)
    same_group = G[:, None] == G[None, :]

    INF = float("inf")
    P = np.array([[w.x, w.y, w.z] for w in waypoints], float)
    node = np.full((N, F + 1), INF)
    bearing = np.full((N, F + 1), np.nan)
    for i in range(N):
        d3 = P[i] - C
        dist = np.linalg.norm(d3, axis=1)
        signed = np.einsum("ij,ij->i", d3, Nn)
        valid = (signed > 0) & (dist <= max_distance_m) & (dist > 1e-6)
        horiz = np.hypot(d3[:, 0], d3[:, 1])
        pitch = np.degrees(np.arctan2(-d3[:, 2], horiz))  # camera pitch to look at the centre
        pen = np.maximum(0.0, np.abs(pitch) - pitch_soft_deg) / pitch_soft_deg
        # The penalty orders targets; it must never disqualify one. Left
        # uncapped it multiplies the cost by up to 2, which can push a facet
        # that is inside the cap and plainly in view past the "no target" cost —
        # the gimbal then refuses to look down at a wall 8 m in front of it and
        # keeps DJI's pose instead. Capping just under the no-target cost keeps
        # the documented invariant ("any reachable facet beats no target") while
        # still preferring the squarer look.
        cost = np.minimum(dist * weight * (1.0 + pen), max_distance_m * (1.0 - 1e-6))
        node[i, :F] = np.where(valid, cost, INF)
        node[i, F] = max_distance_m
        b = np.degrees(np.arctan2(-d3[:, 0], -d3[:, 1]))     # bearing WP → facet, from north
        bearing[i, :F] = np.where(valid, b, np.nan)

    total = node[0].copy()
    back = np.zeros((N, F + 1), dtype=np.int32)
    for i in range(1, N):
        b = bearing[i]
        turn = np.abs(_wrap180(b[:, None] - b[None, :]))
        turn = np.where(np.isnan(turn), 90.0, turn)           # none↔facet: a middling turn
        T = np.where(same_group, 0.0, switch_cost * (1.0 + turn / 90.0))
        cand = total[:, None] + T                              # [from, to]
        back[i] = np.argmin(cand, axis=0)
        total = node[i] + cand[back[i], np.arange(F + 1)]
    picks = [0] * N
    s = int(np.argmin(total))
    for i in range(N - 1, -1, -1):
        picks[i] = -1 if s == F else s
        s = int(back[i, s])
    return picks


def aim_audit(
    waypoints: Sequence[Waypoint],
    facades: Sequence[Facade],
    *,
    far_standoff_m: float,
) -> dict:
    """Plan-time numbers for "is the gimbal about to point at the wrong thing":
    picks further than ``far_standoff_m``, aim reversals >90° between adjacent
    waypoints, target switches, single-waypoint blips (A→B→A), and unaimed
    waypoints. These are the four things the 2026-07-10 picture showed."""
    picks = [w.facade_index for w in waypoints]
    n = len(picks)
    standoff = []
    far = 0
    for w in waypoints:
        if w.facade_index is None or w.facade_index < 0 or w.facade_index >= len(facades):
            continue
        c = np.asarray(facades[w.facade_index].center, float)
        d = float(np.linalg.norm(np.array([w.x, w.y, w.z]) - c))
        standoff.append(d)
        if d > far_standoff_m:
            far += 1
    hd = np.array([w.heading_deg for w in waypoints], float)
    dh = np.abs(_wrap180(np.diff(hd))) if n > 1 else np.zeros(0)
    rev = int(np.sum(dh > 90.0))
    switches = sum(1 for a, b in zip(picks, picks[1:]) if a != b)
    blips = sum(1 for i in range(1, n - 1) if picks[i - 1] == picks[i + 1] != picks[i])
    # Target-aware versions: slices of one plane are one target, so a hop between
    # them is not a switch; and a >90° heading change while tracking the same
    # target is the trajectory (orbiting close), not the picker.
    grp = plane_groups(facades) if len(facades) else []
    g = [grp[p] if (p is not None and 0 <= p < len(grp)) else -1 for p in picks]
    group_switches = sum(1 for a, b in zip(g, g[1:]) if a != b)
    rev_with_change = int(sum(1 for i in range(1, n) if dh[i - 1] > 90.0 and g[i] != g[i - 1]))
    group_blips = sum(1 for i in range(1, n - 1) if g[i - 1] == g[i + 1] != g[i])
    return {
        "far_picks": far,
        "far_standoff_m": far_standoff_m,
        "reversals_gt90": rev,
        "reversals_with_target_change": rev_with_change,
        "switches": switches,
        "target_switches": group_switches,
        "single_blips": blips,
        "target_blips": group_blips,
        "unaimed": sum(1 for p in picks if p is None or p < 0),
        "distinct_targets": len({p for p in picks if p is not None and p >= 0}),
        "standoff_p90_m": float(np.percentile(standoff, 90)) if standoff else None,
        "standoff_max_m": float(max(standoff)) if standoff else None,
    }


def rewrite_gimbals_perpendicular(
    waypoints: list[Waypoint],
    facades: list[Facade],
    max_distance_m: float = 60.0,
    pitch_margin_deg: float = 2.0,
    preserve_heading: bool = True,
    wall_distance_bonus: float = 0.5,
    switch_ratio: float = 0.8,
    max_gimbal_pan_deg: float = 50.0,
    yaw_rate_deg_per_s: float = 60.0,
    heading_rate_fraction: float = 0.5,
    command_gimbal_yaw: bool = False,
    assign_mode: str = "viterbi",
    switch_cost: float = 4.0,
) -> list[Waypoint]:
    """Rewrite ``gimbal_pitch_deg`` / ``gimbal_yaw_deg`` so each waypoint
    photographs the nearest facade head-on.

    Parameters
    ----------
    waypoints
        Imported DJI waypoints in local ENU (x=E, y=N, z=Up).
    facades
        Building facades; ``facade.normal`` must be outward-facing.
    max_distance_m
        Skip facades further than this from the waypoint. Waypoints with no
        qualifying facade keep their original gimbal pose.
    pitch_margin_deg
        Margin from hardware limits (Matrice 4E: -90°..+35°).
    preserve_heading
        If True (default), only update the gimbal and leave aircraft heading
        alone — safer, since DJI's waypoint heading drives turn smoothing.
        Gimbal yaw is then stored absolutely (relative to north, not aircraft).
    switch_ratio
        Hysteresis on facade selection. The previously-tracked facade keeps the
        aim unless a challenger's weighted distance is below
        ``switch_ratio * previous_weighted``. 1.0 disables hysteresis (the old
        memoryless behaviour). Lower is stickier.
    max_gimbal_pan_deg
        Cap on the gimbal yaw required relative to the airframe. Only applies
        when ``preserve_heading`` is False (we are scheduling the heading). The
        M4E's mechanical pan limit is ±60°; 50° leaves margin for the airframe
        overshooting its commanded heading. Measured on 2026-07-10, the old
        ``heading = bearing`` scheme drove actual pan to a median of 51.5° and
        pinned it at the 60° stop on 14% of photos.
    yaw_rate_deg_per_s, heading_rate_fraction
        The airframe's yaw rate and the fraction of it the heading schedule is
        allowed to demand per leg. The fraction exists because DJI's
        ``smoothTransition`` ramps rather than slews at the limit.
    command_gimbal_yaw
        False (default): emit NO gimbal yaw command. ``gimbal_yaw_deg=None`` makes
        kmz_builder leave yaw uncommanded, so the gimbal follows the airframe nose
        and the required pan is identically zero. Only pitch is commanded.

        Why the default flipped. On 2026-07-10 we commanded absolute-north gimbal
        yaw equal to the heading, so the *commanded* pan was 0° everywhere. The
        JPEG XMP shows the *actual* pan sat at a median of 51.5° with a p90 of
        exactly 61.0° — pinned against the ±60° stop on 14% of photos — while the
        airframe tracked its commanded heading to within 14° median. The gimbal
        was 44° away from the yaw we asked for. Our commanded pose pointed at the
        target twice as accurately (17.6° median bearing error) as the gimbal ever
        achieved (35.4°). The yaw axis was not obeying, and it is the only axis
        with a mechanical limit to saturate against. Removing the command removes
        the failure mode; azimuth then comes from the nose, which does work.

        True restores the old absolute-north yaw command, with the heading
        scheduled so the required pan stays within ``max_gimbal_pan_deg``.
    assign_mode
        ``"viterbi"`` (default): choose targets for the whole sequence at once
        (``assign_facades_viterbi``) — looks ahead and behind, so one-waypoint
        blips and corner ping-pong cannot happen. ``"greedy"``: the previous
        per-waypoint nearest-facet picker with ``switch_ratio`` hysteresis.
    switch_cost
        Viterbi only. Cost of changing target, scaled by the turn it demands.
        Measured on busboom (2026-09-02): greedy produced 76 switches and 23
        reversals >90° on 398 waypoints; 4.0 (in weighted metres) is the value
        that removes the blips without pinning the aim to a wall it has passed.

    Waypoints with no facet inside ``max_distance_m`` keep DJI's own gimbal pose
    and heading — Smart3D already pointed them at the target — rather than
    grabbing whatever is furthest away that still qualifies. On busboom the old
    60 m cap let waypoints over empty tarmac aim 20–31 m across the lot.

    Returns a new list of Waypoints; input list is not mutated.
    """
    if assign_mode not in ("viterbi", "greedy"):
        raise ValueError(f"assign_mode must be 'viterbi' or 'greedy', got {assign_mode!r}")
    seq_picks: list[int] | None = None
    if assign_mode == "viterbi":
        seq_picks = assign_facades_viterbi(
            waypoints, facades,
            max_distance_m=max_distance_m,
            wall_distance_bonus=wall_distance_bonus,
            switch_cost=switch_cost,
        )
    pitch_min = GIMBAL_TILT_MIN_DEG + pitch_margin_deg
    pitch_max = GIMBAL_TILT_MAX_DEG - pitch_margin_deg

    out: list[Waypoint] = []
    previous_index: int | None = None
    for wi, wp in enumerate(waypoints):
        pos = np.array([wp.x, wp.y, wp.z], dtype=np.float64)
        if seq_picks is not None:
            si = seq_picks[wi]
            pick = None if si < 0 else (si, float(np.linalg.norm(np.asarray(facades[si].center, float) - pos)))
        else:
            pick = _pick_facade_for_waypoint(
                pos,
                facades,
                max_distance_m,
                wall_distance_bonus=wall_distance_bonus,
                previous_index=previous_index,
                switch_ratio=switch_ratio,
            )
        if pick is None:
            out.append(replace(wp, facade_index=wp.facade_index))
            continue

        idx, _ = pick
        previous_index = idx
        facade = facades[idx]
        center = np.asarray(facade.center, dtype=np.float64)

        # Camera look direction = WP → facade center.
        look = center - pos
        norm = float(np.linalg.norm(look))
        if norm < 1e-6:
            out.append(replace(wp, facade_index=wp.facade_index))
            continue
        look = look / norm
        # Yaw: bearing from north, clockwise. ENU x=East, y=North.
        yaw_rad = math.atan2(look[0], look[1])
        yaw_deg = (math.degrees(yaw_rad) + 360.0) % 360.0
        if yaw_deg > 180.0:
            yaw_deg -= 360.0

        # Pitch: angle from horizontal. Positive = up, negative = down.
        horiz = math.hypot(look[0], look[1])
        pitch_deg = math.degrees(math.atan2(look[2], horiz))
        pitch_deg = max(pitch_min, min(pitch_max, pitch_deg))

        new_wp = replace(
            wp,
            gimbal_pitch_deg=float(pitch_deg),
            # None => kmz_builder emits no yaw command; the gimbal follows the nose.
            gimbal_yaw_deg=float(yaw_deg) if command_gimbal_yaw else None,
            # Heading is scheduled below once every bearing is known; leaving it
            # equal to the bearing here would be the bug we are fixing.
            heading_deg=wp.heading_deg if preserve_heading else float(yaw_deg),
            facade_index=idx,
            # Drop the photogrammetry rosette action group — we want a single
            # head-on photo per waypoint, not the 5-pose smart oblique.
            actions=[a for a in wp.actions if getattr(a, "action_type", None)],
        )
        out.append(new_wp)

    if not preserve_heading and command_gimbal_yaw and out:
        # The gimbal keeps its exact absolute-north aim (`gimbal_yaw_deg`). The
        # heading becomes a rate-limited pursuit of that aim, capped so the
        # residual pan stays inside the gimbal's travel. Waypoints with no facade
        # keep their original bearing as the schedule's target.
        positions = np.array([[w.x, w.y, w.z] for w in out], dtype=np.float64)
        bearings = np.array(
            [
                float(w.gimbal_yaw_deg) if w.gimbal_yaw_deg is not None else float(w.heading_deg)
                for w in out
            ],
            dtype=np.float64,
        )
        speeds = np.array([float(w.speed_ms or 0.0) for w in out], dtype=np.float64)
        heading, _pan, violations = schedule_headings(
            positions,
            bearings,
            speeds,
            max_gimbal_pan_deg=max_gimbal_pan_deg,
            yaw_rate_deg_per_s=yaw_rate_deg_per_s,
            rate_fraction=heading_rate_fraction,
            report=True,
        )
        out = [replace(w, heading_deg=float(h)) for w, h in zip(out, heading)]
        if violations:
            print(
                f"[gimbal] heading schedule: {violations} waypoint(s) needed more yaw "
                f"than {heading_rate_fraction:.0%} of {yaw_rate_deg_per_s:.0f}°/s to keep "
                f"gimbal pan within ±{max_gimbal_pan_deg:.0f}°. The aircraft will yaw hard there."
            )

    return out
