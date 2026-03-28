"""Flight path optimization for multi-facade inspection missions.

Provides waypoint deduplication, facade visit ordering (TSP via 2-opt),
sweep direction selection, and full-path local search to minimize total
transit distance while preserving per-facade boustrophedon coverage.

Industry context:
- Commercial tools (Hammer Missions, Pix4Dscan) use similar approaches:
  facade ordering + sweep direction + local search.
- Redundant coverage at corners is acceptable for photogrammetry, but
  duplicate waypoints waste battery time (~32 min effective on M4E).
- The problem decomposes into: per-facade coverage (boustrophedon, already
  solved) + facade ordering (small TSP) + sweep direction + path smoothing.

Uses only numpy + scipy (already project dependencies). No extra installs.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial import KDTree

from flight_planner.models import Waypoint


@dataclass
class OptimizationResult:
    """Metrics from the optimization pass."""

    waypoints_before: int = 0
    waypoints_after: int = 0
    waypoints_merged: int = 0
    facade_order_before: list[int] = field(default_factory=list)
    facade_order_after: list[int] = field(default_factory=list)
    facades_reversed: list[int] = field(default_factory=list)
    transit_distance_before: float = 0.0
    transit_distance_after: float = 0.0
    two_opt_improvements: int = 0


# ---------------------------------------------------------------------------
# 1. Waypoint deduplication via KDTree
# ---------------------------------------------------------------------------

def deduplicate_waypoints(
    facade_groups: list[list[Waypoint]],
    merge_radius_m: float = 1.0,
    max_gimbal_angle_diff_deg: float = 20.0,
) -> tuple[list[list[Waypoint]], int]:
    """Merge near-coincident waypoints across different facade groups.

    Two waypoints are merged when:
    - Their 3D distance is < merge_radius_m
    - They belong to different facades (intra-facade dedup is already done)
    - Their gimbal angles are within max_gimbal_angle_diff_deg

    When merged, the waypoint with higher facade index is removed and its
    coverage is considered redundant (the remaining waypoint already covers
    that area). This is the industry-standard approach: slight redundancy
    at corners is acceptable; duplicate stops are not.

    Returns (updated_groups, count_removed).
    """
    if len(facade_groups) < 2:
        return facade_groups, 0

    # Build flat list with group/index tracking
    all_positions = []
    wp_refs: list[tuple[int, int]] = []  # (group_idx, wp_idx_in_group)
    for gi, group in enumerate(facade_groups):
        for wi, wp in enumerate(group):
            all_positions.append([wp.x, wp.y, wp.z])
            wp_refs.append((gi, wi))

    if not all_positions:
        return facade_groups, 0

    positions = np.array(all_positions)
    tree = KDTree(positions)

    # Find pairs within merge radius
    to_remove: set[tuple[int, int]] = set()  # (group_idx, wp_idx)
    pairs = tree.query_pairs(r=merge_radius_m)

    for i, j in pairs:
        gi, wi = wp_refs[i]
        gj, wj = wp_refs[j]

        # Only merge across different facades
        if gi == gj:
            continue

        wp_i = facade_groups[gi][wi]
        wp_j = facade_groups[gj][wj]

        # Check gimbal angle similarity
        pitch_diff = abs(wp_i.gimbal_pitch_deg - wp_j.gimbal_pitch_deg)
        yaw_diff = abs(wp_i.gimbal_yaw_deg - wp_j.gimbal_yaw_deg)
        if yaw_diff > 180:
            yaw_diff = 360 - yaw_diff

        if pitch_diff <= max_gimbal_angle_diff_deg and yaw_diff <= max_gimbal_angle_diff_deg:
            # Remove the one from the higher-indexed facade (arbitrary but consistent)
            if gi > gj:
                to_remove.add((gi, wi))
            else:
                to_remove.add((gj, wj))

    if not to_remove:
        return facade_groups, 0

    # Rebuild groups without removed waypoints
    new_groups = []
    for gi, group in enumerate(facade_groups):
        new_group = [
            wp for wi, wp in enumerate(group)
            if (gi, wi) not in to_remove
        ]
        if new_group:
            new_groups.append(new_group)

    return new_groups, len(to_remove)


# ---------------------------------------------------------------------------
# 2. Facade ordering via 2-opt TSP
# ---------------------------------------------------------------------------

def _facade_distance_matrix(groups: list[list[Waypoint]]) -> np.ndarray:
    """Build a distance matrix between facade groups.

    Uses the minimum of (last_wp_A -> first_wp_B) and (last_wp_A -> last_wp_B)
    to account for potential group reversal.
    """
    n = len(groups)
    dist = np.zeros((n, n))
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            # Try both endpoints of group j as entry point
            li = groups[i][-1]
            fj = groups[j][0]
            lj = groups[j][-1]

            d_forward = math.sqrt(
                (li.x - fj.x) ** 2 + (li.y - fj.y) ** 2 + (li.z - fj.z) ** 2
            )
            d_reverse = math.sqrt(
                (li.x - lj.x) ** 2 + (li.y - lj.y) ** 2 + (li.z - lj.z) ** 2
            )
            dist[i, j] = min(d_forward, d_reverse)
    return dist


def _two_opt(order: list[int], dist_matrix: np.ndarray, max_iterations: int = 1000) -> tuple[list[int], int]:
    """Improve a TSP tour using 2-opt local search.

    Iteratively reverses sub-tours to reduce total distance.
    Standard algorithm, O(n^2) per iteration, typically converges fast
    for n < 100 (we have ~4-20 facades).

    Returns (improved_order, improvement_count).
    """
    n = len(order)
    if n < 4:
        return order, 0

    improved = True
    improvements = 0

    for _ in range(max_iterations):
        if not improved:
            break
        improved = False
        for i in range(1, n - 1):
            for j in range(i + 1, n):
                # Cost of current edges
                old_cost = (
                    dist_matrix[order[i - 1], order[i]]
                    + dist_matrix[order[j], order[(j + 1) % n]]
                )
                # Cost if we reverse the segment [i..j]
                new_cost = (
                    dist_matrix[order[i - 1], order[j]]
                    + dist_matrix[order[i], order[(j + 1) % n]]
                )
                if new_cost < old_cost - 1e-9:
                    order[i:j + 1] = reversed(order[i:j + 1])
                    improved = True
                    improvements += 1

    return order, improvements


def order_facades_tsp(groups: list[list[Waypoint]]) -> tuple[list[list[Waypoint]], list[int], list[int], int]:
    """Order facade groups to minimize total transit distance using 2-opt TSP.

    1. Build distance matrix between facade endpoints
    2. Seed with nearest-neighbor tour
    3. Improve with 2-opt

    Returns (ordered_groups, order_before, order_after, two_opt_improvements).
    """
    n = len(groups)
    if n <= 2:
        return groups, list(range(n)), list(range(n)), 0

    dist = _facade_distance_matrix(groups)

    # Seed: nearest-neighbor from closest-to-origin
    centroids = np.array([
        [(g[0].x + g[-1].x) / 2, (g[0].y + g[-1].y) / 2, (g[0].z + g[-1].z) / 2]
        for g in groups
    ])
    start = int(np.argmin(np.sum(centroids ** 2, axis=1)))

    remaining = set(range(n))
    nn_order = [start]
    remaining.discard(start)
    while remaining:
        last = nn_order[-1]
        nearest = min(remaining, key=lambda i: dist[last, i])
        nn_order.append(nearest)
        remaining.discard(nearest)

    order_before = list(nn_order)

    # Improve with 2-opt
    order_after, improvements = _two_opt(list(nn_order), dist)

    return [groups[i] for i in order_after], order_before, order_after, improvements


# ---------------------------------------------------------------------------
# 3. Sweep direction selection (group reversal)
# ---------------------------------------------------------------------------

def optimize_sweep_directions(groups: list[list[Waypoint]]) -> tuple[list[list[Waypoint]], list[int]]:
    """For each facade, choose forward or reversed sweep order to minimize
    the entry distance from the previous facade's exit point.

    The boustrophedon pattern is symmetric — reversing the waypoint list
    gives the same coverage in reverse order. We pick whichever direction
    creates the shortest transition from the previous group.

    Returns (groups_with_optimal_directions, list_of_reversed_facade_indices).
    """
    if len(groups) < 2:
        return groups, []

    result = [groups[0]]  # First group stays as-is
    reversed_indices = []

    for i in range(1, len(groups)):
        prev_exit = result[-1][-1]
        group = groups[i]

        # Distance to first wp (forward sweep)
        d_forward = math.sqrt(
            (prev_exit.x - group[0].x) ** 2
            + (prev_exit.y - group[0].y) ** 2
            + (prev_exit.z - group[0].z) ** 2
        )
        # Distance to last wp (reversed sweep)
        d_reverse = math.sqrt(
            (prev_exit.x - group[-1].x) ** 2
            + (prev_exit.y - group[-1].y) ** 2
            + (prev_exit.z - group[-1].z) ** 2
        )

        if d_reverse < d_forward - 0.1:  # 10cm threshold to avoid unnecessary flips
            result.append(list(reversed(group)))
            reversed_indices.append(i)
        else:
            result.append(group)

    return result, reversed_indices


# ---------------------------------------------------------------------------
# 4. Transit distance calculation
# ---------------------------------------------------------------------------

def _total_transit_distance(groups: list[list[Waypoint]]) -> float:
    """Sum of 3D distances between consecutive facade exit/entry points."""
    total = 0.0
    for i in range(1, len(groups)):
        prev = groups[i - 1][-1]
        curr = groups[i][0]
        total += math.sqrt(
            (prev.x - curr.x) ** 2
            + (prev.y - curr.y) ** 2
            + (prev.z - curr.z) ** 2
        )
    return total


def _total_path_distance(waypoints: list[Waypoint]) -> float:
    """Total 3D path distance across all waypoints."""
    total = 0.0
    for i in range(1, len(waypoints)):
        a, b = waypoints[i - 1], waypoints[i]
        total += math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
    return total


# ---------------------------------------------------------------------------
# 5. Main optimizer pipeline
# ---------------------------------------------------------------------------

def optimize_flight_path(
    facade_groups: list[list[Waypoint]],
    merge_radius_m: float = 1.0,
    enable_dedup: bool = True,
    enable_tsp: bool = True,
    enable_sweep_reversal: bool = True,
    max_gimbal_angle_diff_deg: float = 20.0,
) -> tuple[list[list[Waypoint]], OptimizationResult]:
    """Full optimization pipeline for multi-facade inspection missions.

    Pipeline order:
    1. Waypoint deduplication (merge near-coincident cross-facade waypoints)
    2. Facade ordering via 2-opt TSP
    3. Sweep direction selection (reverse facade groups for shorter transitions)

    Each step can be independently enabled/disabled for testing.

    Args:
        facade_groups: List of per-facade waypoint lists (boustrophedon order).
        merge_radius_m: Dedup merge radius in meters. Default 1.0m.
        enable_dedup: Enable cross-facade waypoint deduplication.
        enable_tsp: Enable 2-opt TSP facade ordering (vs greedy NN).
        enable_sweep_reversal: Enable sweep direction optimization.
        max_gimbal_angle_diff_deg: Max gimbal angle difference for merge eligibility.

    Returns:
        (optimized_groups, result) where result contains optimization metrics.
    """
    result = OptimizationResult()
    result.waypoints_before = sum(len(g) for g in facade_groups)
    result.transit_distance_before = _total_transit_distance(facade_groups)
    result.facade_order_before = list(range(len(facade_groups)))

    groups = facade_groups

    # Step 1: Cross-facade deduplication
    if enable_dedup and len(groups) > 1:
        groups, merged = deduplicate_waypoints(
            groups,
            merge_radius_m=merge_radius_m,
            max_gimbal_angle_diff_deg=max_gimbal_angle_diff_deg,
        )
        result.waypoints_merged = merged

    # Step 2: Facade ordering via TSP with 2-opt
    if enable_tsp and len(groups) > 2:
        groups, order_before, order_after, improvements = order_facades_tsp(groups)
        result.facade_order_after = order_after
        result.two_opt_improvements = improvements
    else:
        result.facade_order_after = list(range(len(groups)))

    # Step 3: Sweep direction optimization
    if enable_sweep_reversal and len(groups) > 1:
        groups, reversed_indices = optimize_sweep_directions(groups)
        result.facades_reversed = reversed_indices

    result.waypoints_after = sum(len(g) for g in groups)
    result.transit_distance_after = _total_transit_distance(groups)

    return groups, result
