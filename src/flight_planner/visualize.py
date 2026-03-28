"""3D visualization of buildings, waypoints, and camera direction vectors.

Uses matplotlib for 3D scatter/quiver plots. Install with:
    pip install matplotlib
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from .models import Building, Facade, Waypoint


def plot_mission(
    building: Building,
    waypoints: list[Waypoint],
    title: str = "Inspection Mission",
    save_path: Optional[str] = None,
    show: bool = True,
) -> None:
    """Plot a 3D view of the building wireframe, waypoints, and camera directions.

    Args:
        building: The building with facades.
        waypoints: List of mission waypoints (in local ENU coords).
        title: Plot title.
        save_path: Optional path to save the figure.
        show: Whether to display the plot interactively.
    """
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection="3d")

    # Color map for facades
    facade_colors = [
        "#4a90d9",  # blue
        "#d94a4a",  # red
        "#4ad94a",  # green
        "#d9d94a",  # yellow
        "#d94ad9",  # magenta
        "#4ad9d9",  # cyan
        "#d9914a",  # orange
        "#914ad9",  # purple
        "#4a9191",  # teal
        "#91914a",  # olive
    ]

    # Draw building facades as transparent polygons
    for i, facade in enumerate(building.facades):
        verts = facade.vertices
        color = facade_colors[i % len(facade_colors)]

        # Draw edges
        n = len(verts)
        for j in range(n):
            p1 = verts[j]
            p2 = verts[(j + 1) % n]
            ax.plot3D(
                [p1[0], p2[0]],
                [p1[1], p2[1]],
                [p1[2], p2[2]],
                color=color,
                linewidth=1.5,
            )

        # Draw filled polygon (semi-transparent)
        poly = Poly3DCollection([verts], alpha=0.15, facecolor=color, edgecolor=color)
        ax.add_collection3d(poly)

        # Draw normal vector from center
        center = facade.center
        normal = facade.normal * 2  # scale for visibility
        ax.quiver(
            center[0], center[1], center[2],
            normal[0], normal[1], normal[2],
            color=color,
            arrow_length_ratio=0.2,
            linewidth=2,
        )

        # Label
        ax.text(
            center[0] + normal[0] * 0.5,
            center[1] + normal[1] * 0.5,
            center[2] + normal[2] * 0.5,
            facade.label,
            fontsize=7,
            color=color,
        )

    # Draw waypoints
    if waypoints:
        wp_x = [wp.x for wp in waypoints]
        wp_y = [wp.y for wp in waypoints]
        wp_z = [wp.z for wp in waypoints]

        # Color waypoints by facade index
        facade_indices = [wp.facade_index for wp in waypoints]
        unique_facades = sorted(set(facade_indices))
        color_map = {fi: facade_colors[fi % len(facade_colors)] for fi in unique_facades}
        wp_colors = [color_map[fi] for fi in facade_indices]

        ax.scatter(wp_x, wp_y, wp_z, c=wp_colors, s=15, marker="o", alpha=0.7, depthshade=True)

        # Draw camera direction vectors (pointing toward facade)
        for wp in waypoints:
            # Camera direction = opposite of the heading direction
            heading_rad = np.radians(wp.heading_deg)
            # Heading is clockwise from north: dx = sin(h), dy = cos(h)
            # Camera faces in heading direction
            cam_dx = np.sin(heading_rad) * 1.0
            cam_dy = np.cos(heading_rad) * 1.0
            cam_dz = np.tan(np.radians(wp.gimbal_pitch_deg)) * 1.0 if wp.gimbal_pitch_deg != -90 else -1.0

            ax.quiver(
                wp.x, wp.y, wp.z,
                cam_dx, cam_dy, cam_dz,
                color=color_map.get(wp.facade_index, "gray"),
                arrow_length_ratio=0.15,
                linewidth=0.5,
                alpha=0.4,
            )

        # Draw flight path (connect consecutive waypoints)
        ax.plot3D(wp_x, wp_y, wp_z, "k-", linewidth=0.3, alpha=0.3)

    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_zlabel("Up (m)")
    ax.set_title(title)

    # Equal aspect ratio
    all_points = []
    for facade in building.facades:
        all_points.extend(facade.vertices.tolist())
    if waypoints:
        all_points.extend([[wp.x, wp.y, wp.z] for wp in waypoints])
    if all_points:
        all_points = np.array(all_points)
        mid = all_points.mean(axis=0)
        max_range = (all_points.max(axis=0) - all_points.min(axis=0)).max() / 2
        ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
        ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
        ax.set_zlim(0, max(mid[2] + max_range, all_points[:, 2].max() + 2))

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"Plot saved to: {save_path}")

    if show:
        plt.show()

    plt.close(fig)
