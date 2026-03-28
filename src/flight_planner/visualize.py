"""Visualization of buildings, waypoints, and camera direction vectors.

Two modes:
  - plot_mission(): matplotlib 3D scatter/quiver plot (local ENU coords)
  - export_mission_html(): standalone HTML file with Leaflet satellite map
"""

from __future__ import annotations

import json
import math
import os
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


def _facade_color(index: int) -> str:
    colors = [
        "#2563eb", "#dc2626", "#16a34a", "#ca8a04",
        "#9333ea", "#0891b2", "#ea580c", "#6d28d9",
        "#0d9488", "#65a30d",
    ]
    return colors[index % len(colors)]


def _escape_html(text: str) -> str:
    """Escape HTML special characters to prevent XSS."""
    return (
        text.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
        .replace("'", "&#x27;")
    )


def prepare_leaflet_data(building: Building, waypoints: list[Waypoint]) -> dict:
    """Prepare JSON-serializable data for the 2D Leaflet satellite map viewer.

    This is a public helper used by both export_mission_html() and the server API.
    """
    center_lat = building.lat
    center_lon = building.lon

    # Building footprint polygon (ground-level vertices of walls)
    ground_coords = []
    for facade in building.facades:
        if abs(facade.normal[2]) < 0.01:  # vertical walls only
            for v in facade.vertices:
                if abs(v[2]) < 0.1:  # ground-level vertices
                    lat_offset = v[1] / (111132.92 - 559.82 * math.cos(2 * math.radians(center_lat)))
                    lon_offset = v[0] / (111412.84 * math.cos(math.radians(center_lat)))
                    ground_coords.append([center_lon + lon_offset, center_lat + lat_offset])

    if ground_coords:
        cx = sum(c[0] for c in ground_coords) / len(ground_coords)
        cy = sum(c[1] for c in ground_coords) / len(ground_coords)
        ground_coords.sort(key=lambda c: math.atan2(c[1] - cy, c[0] - cx))
        ground_coords.append(ground_coords[0])  # close polygon

    # Waypoints grouped by facade
    facade_groups: dict[int, list[dict]] = {}
    for wp in waypoints:
        fi = wp.facade_index
        if fi not in facade_groups:
            facade_groups[fi] = []
        facade_groups[fi].append({
            "index": wp.index,
            "lat": wp.lat,
            "lon": wp.lon,
            "alt": round(wp.z, 1),
            "heading": round(wp.heading_deg, 1),
            "gimbal_pitch": round(wp.gimbal_pitch_deg, 1),
            "facade_index": fi,
            "component": wp.component_tag,
        })

    facade_meta = {}
    for f in building.facades:
        facade_meta[str(f.index)] = {
            "label": f.label,
            "color": _facade_color(f.index),
            "azimuth": round(f.azimuth_deg, 0),
            "component": f.component_tag,
        }

    return {
        "facadeGroups": facade_groups,
        "facadeMeta": facade_meta,
        "flightPath": [[wp.lon, wp.lat] for wp in waypoints],
        "buildingPoly": ground_coords,
        "center": [center_lat, center_lon],
        "buildingLabel": _escape_html(building.label or "Building"),
        "buildingDims": f"{building.width}m x {building.depth}m x {building.height}m",
        "waypointCount": len(waypoints),
        "facadeCount": len(building.facades),
    }


def prepare_threejs_data(building: Building, waypoints: list[Waypoint]) -> dict:
    """Prepare JSON-serializable data for the 3D Three.js viewer.

    This is a public helper used by both export_mission_3d_html() and the server API.
    """
    facade_data = []
    for f in building.facades:
        facade_data.append({
            "vertices": f.vertices.tolist(),
            "normal": f.normal.tolist(),
            "label": f.label,
            "index": f.index,
            "component": f.component_tag,
            "color": _facade_color(f.index),
        })

    wp_data = []
    for wp in waypoints:
        wp_data.append({
            "x": round(wp.x, 2),
            "y": round(wp.y, 2),
            "z": round(wp.z, 2),
            "heading": round(wp.heading_deg, 1),
            "gimbal_pitch": round(wp.gimbal_pitch_deg, 1),
            "facade_index": wp.facade_index,
            "index": wp.index,
            "component": wp.component_tag,
        })

    return {
        "facades": facade_data,
        "waypoints": wp_data,
        "buildingLabel": _escape_html(building.label or "Building"),
        "buildingDims": f"{building.width}m x {building.depth}m x {building.height}m",
        "buildingHeight": building.height,
    }


def export_mission_html(
    building: Building,
    waypoints: list[Waypoint],
    output_path: str,
    title: str = "AeroScan Inspection Mission",
) -> str:
    """Export an interactive HTML map showing the mission on satellite imagery.

    Uses Leaflet.js with Esri satellite tiles. No API key needed.
    Opens directly in any browser.
    """
    safe_title = _escape_html(title)
    data = prepare_leaflet_data(building, waypoints)
    data_json = json.dumps(data)

    html = _LEAFLET_HTML_TEMPLATE.replace("__TITLE__", safe_title).replace("__DATA_JSON__", data_json)

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    with open(output_path, "w") as f:
        f.write(html)

    return os.path.abspath(output_path)


def export_mission_3d_html(
    building: Building,
    waypoints: list[Waypoint],
    output_path: str,
    title: str = "AeroScan Inspection Mission - 3D",
) -> str:
    """Export an interactive 3D HTML viewer using Three.js.

    Full orbit/rotate/zoom controls. Shows building as solid geometry,
    waypoints as colored spheres at actual altitude, camera direction arrows,
    and the flight path.

    Controls:
      - Left-click drag: rotate/orbit
      - Right-click drag: pan
      - Scroll: zoom
      - Click waypoint: show info

    Args:
        building: The building with facades.
        waypoints: List of waypoints (local ENU coordinates used).
        output_path: Path for the .html file.
        title: Page title.

    Returns:
        Absolute path to the generated HTML file.
    """
    safe_title = _escape_html(title)
    data = prepare_threejs_data(building, waypoints)
    data_json = json.dumps(data)

    html = _THREEJS_HTML_TEMPLATE.replace("__TITLE__", safe_title).replace("__DATA_JSON__", data_json)

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    with open(output_path, "w") as f:
        f.write(html)

    return os.path.abspath(output_path)


_THREEJS_HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>__TITLE__</title>
<style>
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body { background: #1a1a2e; font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; overflow: hidden; }
  #c { display: block; width: 100vw; height: 100vh; }
  .panel {
    position: absolute; top: 12px; right: 12px; z-index: 10;
    background: rgba(20,20,40,0.92); border-radius: 8px; color: #ccc;
    padding: 14px 16px; max-width: 260px; box-shadow: 0 2px 16px rgba(0,0,0,0.4);
    font-size: 13px; max-height: calc(100vh - 24px); overflow-y: auto;
    backdrop-filter: blur(8px);
  }
  .panel h2 { font-size: 15px; color: #fff; margin-bottom: 6px; }
  .panel h3 { font-size: 11px; color: #888; margin: 10px 0 4px; text-transform: uppercase; letter-spacing: 0.5px; }
  .stat { color: #aaa; font-size: 12px; }
  .stat b { color: #eee; }
  .legend-item { display: flex; align-items: center; gap: 6px; padding: 2px 0; cursor: pointer; font-size: 12px; }
  .legend-dot { width: 10px; height: 10px; border-radius: 50%; flex-shrink: 0; }
  .legend-item.off .legend-dot { opacity: 0.15; }
  .legend-item.off span { color: #555; text-decoration: line-through; }
  .controls { font-size: 11px; color: #666; margin-top: 10px; line-height: 1.6; }
  #info {
    position: absolute; bottom: 12px; left: 12px; z-index: 10;
    background: rgba(20,20,40,0.92); border-radius: 6px; color: #ccc;
    padding: 10px 14px; font-size: 12px; display: none;
    backdrop-filter: blur(8px);
  }
  #info b { color: #fff; }
  .btn {
    margin-top: 6px; padding: 4px 10px; border: 1px solid #444; border-radius: 4px;
    background: rgba(255,255,255,0.05); cursor: pointer; font-size: 11px; color: #aaa;
  }
  .btn:hover { background: rgba(255,255,255,0.1); color: #fff; }
</style>
</head>
<body>
<canvas id="c"></canvas>
<div class="panel">
  <h2 id="panelTitle"></h2>
  <p class="stat" id="panelStats"></p>
  <h3>Facades</h3>
  <div id="legend"></div>
  <h3>Layers</h3>
  <label class="legend-item"><input type="checkbox" id="togPath" checked> <span>Flight path</span></label>
  <label class="legend-item"><input type="checkbox" id="togArrows" checked> <span>Camera arrows</span></label>
  <label class="legend-item"><input type="checkbox" id="togGround" checked> <span>Ground grid</span></label>
  <button class="btn" id="btnReset">Reset camera</button>
  <div class="controls">
    <b style="color:#aaa">Controls:</b><br>
    Left drag: orbit<br>
    Right drag: pan<br>
    Scroll: zoom<br>
    Click dot: waypoint info
  </div>
</div>
<div id="info"></div>

<script type="importmap">
{
  "imports": {
    "three": "https://unpkg.com/three@0.164.1/build/three.module.js",
    "three/addons/": "https://unpkg.com/three@0.164.1/examples/jsm/"
  }
}
</script>
<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const D = __DATA_JSON__;

document.getElementById('panelTitle').textContent = document.title;
document.getElementById('panelStats').textContent =
  D.waypoints.length + ' waypoints \u00b7 ' + D.facades.length + ' facades';

// Scene
const canvas = document.getElementById('c');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setClearColor(0x1a1a2e);

const scene = new THREE.Scene();
scene.fog = new THREE.Fog(0x1a1a2e, 80, 200);

const camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.1, 500);
camera.position.set(35, 25, 35);

const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.target.set(0, D.buildingHeight / 2, 0);
controls.update();

// Lighting
scene.add(new THREE.AmbientLight(0xffffff, 0.5));
var dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
dirLight.position.set(20, 40, 30);
scene.add(dirLight);
scene.add(new THREE.HemisphereLight(0x4488cc, 0x332211, 0.3));

// Ground grid
var gridHelper = new THREE.GridHelper(100, 50, 0x333355, 0x222244);
scene.add(gridHelper);

// Ground plane (subtle)
var groundGeo = new THREE.PlaneGeometry(100, 100);
var groundMat = new THREE.MeshStandardMaterial({ color: 0x1a1a2e, roughness: 1 });
var ground = new THREE.Mesh(groundGeo, groundMat);
ground.rotation.x = -Math.PI / 2;
ground.position.y = -0.01;
scene.add(ground);

// Axes helper (small, at origin)
var axes = new THREE.AxesHelper(3);
scene.add(axes);

// Convert ENU (x=East, y=North, z=Up) to Three.js (x=right, y=up, z=toward camera)
function enu(x, y, z) { return new THREE.Vector3(x, z, -y); }

// Build facades
var facadeGroups = {};
D.facades.forEach(function(f) {
  var color = new THREE.Color(f.color);
  var group = new THREE.Group();

  // Solid face
  var verts = f.vertices;
  if (verts.length >= 3) {
    var shape = new THREE.BufferGeometry();
    var positions = [];
    // Triangulate as a fan from first vertex
    for (var i = 1; i < verts.length - 1; i++) {
      positions.push(verts[0][0], verts[0][2], -verts[0][1]);
      positions.push(verts[i][0], verts[i][2], -verts[i][1]);
      positions.push(verts[i+1][0], verts[i+1][2], -verts[i+1][1]);
    }
    shape.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    shape.computeVertexNormals();
    var faceMat = new THREE.MeshStandardMaterial({
      color: color, transparent: true, opacity: 0.35, side: THREE.DoubleSide, roughness: 0.8
    });
    group.add(new THREE.Mesh(shape, faceMat));

    // Wireframe edges
    var edgePositions = [];
    for (var i = 0; i < verts.length; i++) {
      var a = verts[i], b = verts[(i + 1) % verts.length];
      edgePositions.push(a[0], a[2], -a[1], b[0], b[2], -b[1]);
    }
    var edgeGeo = new THREE.BufferGeometry();
    edgeGeo.setAttribute('position', new THREE.Float32BufferAttribute(edgePositions, 3));
    var edgeMat = new THREE.LineBasicMaterial({ color: color, linewidth: 2 });
    group.add(new THREE.LineSegments(edgeGeo, edgeMat));
  }

  scene.add(group);
  facadeGroups[f.index] = group;
});

// Build waypoints as instanced spheres per facade
var wpSphereGeo = new THREE.SphereGeometry(0.25, 8, 6);
var facadeWpMeshes = {};
var facadeArrowGroups = {};
var wpPositions = [];  // for raycasting

var facadeIndices = {};
D.waypoints.forEach(function(wp) {
  if (!facadeIndices[wp.facade_index]) facadeIndices[wp.facade_index] = [];
  facadeIndices[wp.facade_index].push(wp);
});

Object.keys(facadeIndices).forEach(function(fi) {
  var wps = facadeIndices[fi];
  var meta = D.facades.find(function(f) { return f.index === parseInt(fi); });
  var color = meta ? new THREE.Color(meta.color) : new THREE.Color(0x888888);

  // Instanced mesh for waypoint spheres
  var mesh = new THREE.InstancedMesh(wpSphereGeo, new THREE.MeshStandardMaterial({
    color: color, roughness: 0.4, metalness: 0.1
  }), wps.length);

  var dummy = new THREE.Object3D();
  wps.forEach(function(wp, i) {
    dummy.position.copy(enu(wp.x, wp.y, wp.z));
    dummy.updateMatrix();
    mesh.setMatrixAt(i, dummy.matrix);
    wpPositions.push({ pos: enu(wp.x, wp.y, wp.z), wp: wp, fi: fi });
  });
  mesh.instanceMatrix.needsUpdate = true;
  scene.add(mesh);
  facadeWpMeshes[fi] = mesh;

  // Camera direction arrows
  var arrowGroup = new THREE.Group();
  wps.forEach(function(wp) {
    var origin = enu(wp.x, wp.y, wp.z);
    var headingRad = wp.heading * Math.PI / 180;
    var pitchRad = wp.gimbal_pitch * Math.PI / 180;
    var dx = Math.sin(headingRad) * Math.cos(pitchRad);
    var dy = Math.cos(headingRad) * Math.cos(pitchRad);
    var dz = Math.sin(pitchRad);
    var dir = enu(dx, dy, dz).normalize();
    var arrow = new THREE.ArrowHelper(dir, origin, 2.0, color.getHex(), 0.3, 0.15);
    arrowGroup.add(arrow);
  });
  scene.add(arrowGroup);
  facadeArrowGroups[fi] = arrowGroup;
});

// Flight path line
var pathPositions = [];
D.waypoints.forEach(function(wp) {
  var p = enu(wp.x, wp.y, wp.z);
  pathPositions.push(p.x, p.y, p.z);
});
var pathGeo = new THREE.BufferGeometry();
pathGeo.setAttribute('position', new THREE.Float32BufferAttribute(pathPositions, 3));
var pathLine = new THREE.Line(pathGeo, new THREE.LineBasicMaterial({ color: 0xffffff, opacity: 0.3, transparent: true }));
scene.add(pathLine);

// Legend
var legendDiv = document.getElementById('legend');
D.facades.forEach(function(f) {
  var count = (facadeIndices[f.index] || []).length;
  var item = document.createElement('div');
  item.className = 'legend-item';
  var dot = document.createElement('span');
  dot.className = 'legend-dot';
  dot.style.backgroundColor = f.color;
  var label = document.createElement('span');
  label.textContent = f.label + ' (' + count + ')';
  item.appendChild(dot);
  item.appendChild(label);
  item.addEventListener('click', function() {
    var isOff = item.classList.toggle('off');
    if (facadeGroups[f.index]) facadeGroups[f.index].visible = !isOff;
    if (facadeWpMeshes[f.index]) facadeWpMeshes[f.index].visible = !isOff;
    if (facadeArrowGroups[f.index]) facadeArrowGroups[f.index].visible = !isOff;
  });
  legendDiv.appendChild(item);
});

// Layer toggles
document.getElementById('togPath').addEventListener('change', function(e) {
  pathLine.visible = e.target.checked;
});
document.getElementById('togArrows').addEventListener('change', function(e) {
  Object.values(facadeArrowGroups).forEach(function(g) { g.visible = e.target.checked; });
});
document.getElementById('togGround').addEventListener('change', function(e) {
  gridHelper.visible = e.target.checked;
  ground.visible = e.target.checked;
});
document.getElementById('btnReset').addEventListener('click', function() {
  camera.position.set(35, 25, 35);
  controls.target.set(0, D.buildingHeight / 2, 0);
  controls.update();
});

// Raycasting for click-to-inspect
var raycaster = new THREE.Raycaster();
raycaster.params.Points = { threshold: 0.5 };
var mouse = new THREE.Vector2();
var infoDiv = document.getElementById('info');

canvas.addEventListener('click', function(e) {
  mouse.x = (e.clientX / window.innerWidth) * 2 - 1;
  mouse.y = -(e.clientY / window.innerHeight) * 2 + 1;
  raycaster.setFromCamera(mouse, camera);

  // Find closest waypoint to ray
  var bestDist = 1.5;
  var bestWp = null;
  var bestFi = null;
  wpPositions.forEach(function(entry) {
    var d = raycaster.ray.distanceToPoint(entry.pos);
    if (d < bestDist) {
      bestDist = d;
      bestWp = entry.wp;
      bestFi = entry.fi;
    }
  });

  if (bestWp) {
    var meta = D.facades.find(function(f) { return f.index === parseInt(bestFi); });
    var facadeLabel = meta ? meta.label : 'unknown';
    infoDiv.style.display = 'block';
    var t = document.createElement('div');
    var lines = [
      'WP ' + bestWp.index,
      'Facade: ' + facadeLabel,
      'Position: (' + bestWp.x + ', ' + bestWp.y + ', ' + bestWp.z + ')m',
      'Heading: ' + bestWp.heading + '\u00b0',
      'Gimbal pitch: ' + bestWp.gimbal_pitch + '\u00b0',
      'Component: ' + bestWp.component
    ];
    infoDiv.textContent = '';
    var b = document.createElement('b');
    b.textContent = lines[0];
    infoDiv.appendChild(b);
    for (var i = 1; i < lines.length; i++) {
      infoDiv.appendChild(document.createElement('br'));
      infoDiv.appendChild(document.createTextNode(lines[i]));
    }
  } else {
    infoDiv.style.display = 'none';
  }
});

// Resize
window.addEventListener('resize', function() {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

// Animate
function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}
animate();
</script>
</body>
</html>
"""


_LEAFLET_HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>__TITLE__</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; }
  #map { position: absolute; top: 0; left: 0; right: 0; bottom: 0; }
  .panel {
    position: absolute; top: 12px; right: 12px; z-index: 1000;
    background: rgba(255,255,255,0.95); border-radius: 8px;
    padding: 14px 16px; max-width: 280px; box-shadow: 0 2px 12px rgba(0,0,0,0.15);
    font-size: 13px; max-height: calc(100vh - 24px); overflow-y: auto;
  }
  .panel h2 { font-size: 15px; margin-bottom: 8px; }
  .panel h3 { font-size: 12px; color: #666; margin: 10px 0 4px; text-transform: uppercase; letter-spacing: 0.5px; }
  .legend-item { display: flex; align-items: center; gap: 6px; padding: 2px 0; cursor: pointer; }
  .legend-dot { width: 10px; height: 10px; border-radius: 50%; flex-shrink: 0; }
  .legend-item.off .legend-dot { opacity: 0.2; }
  .legend-item.off span { color: #aaa; text-decoration: line-through; }
  .stat { color: #555; }
  .stat b { color: #222; }
  .toggle-btn {
    margin-top: 8px; padding: 4px 10px; border: 1px solid #ddd; border-radius: 4px;
    background: #f8f8f8; cursor: pointer; font-size: 12px;
  }
  .toggle-btn:hover { background: #eee; }
</style>
</head>
<body>
<div id="map"></div>
<div class="panel">
  <h2 id="panelTitle"></h2>
  <p class="stat" id="panelStats"></p>
  <h3>Facades</h3>
  <div id="legend"></div>
  <h3>Layers</h3>
  <label class="legend-item"><input type="checkbox" id="togPath" checked> Flight path</label>
  <label class="legend-item"><input type="checkbox" id="togArrows" checked> Camera direction</label>
  <button class="toggle-btn" onclick="fitAll()">Fit to view</button>
</div>

<script>
const D = __DATA_JSON__;

document.getElementById('panelTitle').textContent = document.title;
document.getElementById('panelStats').textContent =
  D.waypointCount + ' waypoints \u00b7 ' + D.facadeCount + ' facades';

const map = L.map('map', { zoomControl: true }).setView(D.center, 19);

const satellite = L.tileLayer(
  'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
  { attribution: 'Esri', maxZoom: 21 }
).addTo(map);

const osm = L.tileLayer(
  'https://{s}.tile.openstreetmap.org/{z}/{y}/{x}.png',
  { attribution: '\u00a9 OSM', maxZoom: 19 }
);

L.control.layers({ 'Satellite': satellite, 'Street': osm }, {}, { position: 'bottomleft' }).addTo(map);

if (D.buildingPoly.length > 0) {
  var coords = D.buildingPoly.map(function(c) { return [c[1], c[0]]; });
  L.polygon(coords, {
    color: '#fff', weight: 2, fillColor: '#334155', fillOpacity: 0.4, dashArray: '4'
  }).addTo(map).bindPopup(D.buildingLabel + '<br>' + D.buildingDims);
}

var facadeLayers = {};
var arrowLayers = {};

Object.keys(D.facadeGroups).forEach(function(fi) {
  var wps = D.facadeGroups[fi];
  var meta = D.facadeMeta[fi] || { label: 'unknown', color: '#888' };
  var color = meta.color;
  var markers = L.layerGroup();
  var arrows = L.layerGroup();

  wps.forEach(function(wp) {
    var circle = L.circleMarker([wp.lat, wp.lon], {
      radius: 4, color: color, fillColor: color, fillOpacity: 0.8, weight: 1
    });
    var popupText = 'WP ' + wp.index +
      '\nFacade: ' + meta.label +
      '\nAlt: ' + wp.alt + 'm' +
      '\nHeading: ' + wp.heading + '\u00b0' +
      '\nGimbal: ' + wp.gimbal_pitch + '\u00b0' +
      '\nComponent: ' + wp.component +
      '\n' + wp.lat.toFixed(6) + ', ' + wp.lon.toFixed(6);
    circle.bindPopup(popupText.replace(/\n/g, '<br>'));
    markers.addLayer(circle);

    var headingRad = wp.heading * Math.PI / 180;
    var arrowLen = 0.000025;
    var endLat = wp.lat + Math.cos(headingRad) * arrowLen;
    var endLon = wp.lon + Math.sin(headingRad) * arrowLen;
    arrows.addLayer(L.polyline([[wp.lat, wp.lon], [endLat, endLon]], {
      color: color, weight: 1.5, opacity: 0.5
    }));
  });

  markers.addTo(map);
  arrows.addTo(map);
  facadeLayers[fi] = markers;
  arrowLayers[fi] = arrows;
});

var pathLine = L.polyline(
  D.flightPath.map(function(c) { return [c[1], c[0]]; }),
  { color: '#fff', weight: 1, opacity: 0.4, dashArray: '3 6' }
).addTo(map);

var legendDiv = document.getElementById('legend');
Object.keys(D.facadeMeta).forEach(function(fi) {
  var meta = D.facadeMeta[fi];
  var count = (D.facadeGroups[fi] || []).length;
  var item = document.createElement('div');
  item.className = 'legend-item';
  var dot = document.createElement('span');
  dot.className = 'legend-dot';
  dot.style.backgroundColor = meta.color;
  var label = document.createElement('span');
  label.textContent = meta.label + ' (' + count + ')';
  item.appendChild(dot);
  item.appendChild(label);
  item.addEventListener('click', function() {
    var isOff = item.classList.toggle('off');
    if (isOff) {
      map.removeLayer(facadeLayers[fi]);
      map.removeLayer(arrowLayers[fi]);
    } else {
      facadeLayers[fi].addTo(map);
      arrowLayers[fi].addTo(map);
    }
  });
  legendDiv.appendChild(item);
});

document.getElementById('togPath').addEventListener('change', function(e) {
  e.target.checked ? pathLine.addTo(map) : map.removeLayer(pathLine);
});
document.getElementById('togArrows').addEventListener('change', function(e) {
  Object.values(arrowLayers).forEach(function(layer) {
    e.target.checked ? layer.addTo(map) : map.removeLayer(layer);
  });
});

function fitAll() {
  var allPts = [];
  Object.values(D.facadeGroups).forEach(function(wps) {
    wps.forEach(function(wp) { allPts.push([wp.lat, wp.lon]); });
  });
  if (allPts.length) map.fitBounds(L.latLngBounds(allPts).pad(0.15));
}
fitAll();
</script>
</body>
</html>
"""
