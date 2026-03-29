"""Simulated photogrammetric reconstruction pipeline.

Renders synthetic photos from mission waypoints, reconstructs a 3D mesh
via TSDF fusion, and reimports the result through build_building_from_mesh().
Tests the full capture → reconstruct → reimport loop without a physical flight.

Requires the 'mesh' extra: pip install -e ".[mesh]"

On macOS, Open3D GPU rendering must run on the main thread.  We therefore
spawn a **subprocess** for the render+TSDF step so it gets its own main
thread.  The subprocess communicates via files on disk (PLY mesh + JSON).
"""

from __future__ import annotations

import json
import logging
import math
import subprocess
import sys
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

from .building_import import build_building_from_mesh
from .geometry import generate_mission_waypoints
from .models import (
    AlgorithmConfig,
    Building,
    CameraSpec,
    CAMERAS,
    MissionConfig,
    Waypoint,
)
from .visualize import prepare_leaflet_data, prepare_threejs_data

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Simulation task tracking
# ---------------------------------------------------------------------------

@dataclass
class SimulationTask:
    task_id: str
    status: str = "pending"  # pending | rendering | reconstructing | importing | generating | complete | error
    progress: float = 0.0
    message: str = ""
    result: Optional[dict] = None
    error: Optional[str] = None
    version_id: Optional[str] = None


_tasks: dict[str, SimulationTask] = {}
_task_lock = threading.Lock()


def get_task(task_id: str) -> SimulationTask | None:
    with _task_lock:
        return _tasks.get(task_id)


def _save_to_db(task: SimulationTask):
    """Persist a completed simulation to the database."""
    from .server.database import SimulationRecord, get_db
    db = get_db()
    try:
        record = SimulationRecord(
            task_id=task.task_id,
            status=task.status,
            source_version=task.version_id,
            result_json=json.dumps(task.result) if task.result else None,
            output_dir=str(Path("sim_output") / task.task_id),
        )
        # Upsert
        existing = db.query(SimulationRecord).filter_by(task_id=task.task_id).first()
        if existing:
            existing.status = record.status
            existing.result_json = record.result_json
        else:
            db.add(record)
        db.commit()
    finally:
        db.close()


def list_tasks() -> list[dict]:
    """Return all simulation tasks — in-progress from memory, completed from DB."""
    from .server.database import SimulationRecord, get_db

    result = []

    # In-progress tasks from memory
    with _task_lock:
        for t in _tasks.values():
            if t.status not in ("complete", "error"):
                result.append({
                    "task_id": t.task_id,
                    "status": t.status,
                    "progress": round(t.progress, 3),
                    "message": t.message,
                })

    # Completed tasks from database
    db = get_db()
    try:
        records = db.query(SimulationRecord).order_by(SimulationRecord.created_at.desc()).all()
        for r in records:
            # Skip if already in the in-progress list
            if any(x["task_id"] == r.task_id for x in result):
                continue
            result.append(r.to_summary())
    finally:
        db.close()

    return result


def get_task_result(task_id: str) -> dict | None:
    """Get a completed simulation result from DB."""
    from .server.database import SimulationRecord, get_db
    db = get_db()
    try:
        record = db.query(SimulationRecord).filter_by(task_id=task_id).first()
        if record and record.result_json:
            return json.loads(record.result_json)
        return None
    finally:
        db.close()


def delete_task(task_id: str) -> bool:
    """Remove a simulation task from memory, DB, and disk."""
    import shutil
    from .server.database import SimulationRecord, get_db

    # Remove from memory
    with _task_lock:
        _tasks.pop(task_id, None)

    # Remove from DB
    db = get_db()
    try:
        record = db.query(SimulationRecord).filter_by(task_id=task_id).first()
        if record:
            db.delete(record)
            db.commit()
    finally:
        db.close()

    # Remove output files
    output_dir = Path("sim_output") / task_id
    if output_dir.exists():
        shutil.rmtree(output_dir, ignore_errors=True)
    return True


def _update_task(task: SimulationTask, **kwargs):
    with _task_lock:
        for k, v in kwargs.items():
            setattr(task, k, v)


# ---------------------------------------------------------------------------
# Helpers: build mesh + camera data for the subprocess
# ---------------------------------------------------------------------------

_FACADE_COLORS = {
    "north": [0.72, 0.42, 0.32],
    "south": [0.80, 0.52, 0.37],
    "east":  [0.76, 0.46, 0.34],
    "west":  [0.66, 0.39, 0.29],
    "roof":  [0.42, 0.42, 0.47],
}


def _classify_direction(normal) -> str:
    nz = abs(normal[2])
    if nz > 0.3:
        return "roof"
    az = math.degrees(math.atan2(normal[0], normal[1])) % 360
    if az < 45 or az >= 315:
        return "north"
    if az < 135:
        return "east"
    if az < 225:
        return "south"
    return "west"


def _export_render_mesh(building: Building, out_path: Path, raw_mesh: dict | None = None):
    """Export the building mesh for rendering.

    If raw_mesh data is available (from a mesh upload), use the full
    triangle mesh — this is what the drone camera would actually see.
    Otherwise (preset buildings), extrude the facade polygons into solid
    slabs so the renderer sees geometry from every angle.
    """
    import trimesh

    if raw_mesh and "positions" in raw_mesh and "indices" in raw_mesh:
        # Use the actual raw mesh — the real building surface
        n_v = len(raw_mesh["positions"]) // 3
        n_f = len(raw_mesh["indices"]) // 3
        logger.info(f"_export_render_mesh: using RAW MESH ({n_v} verts, {n_f} faces)")
        positions = np.array(raw_mesh["positions"], dtype=np.float64).reshape(-1, 3)
        indices = np.array(raw_mesh["indices"], dtype=np.int64).reshape(-1, 3)
        mesh = trimesh.Trimesh(vertices=positions, faces=indices)
        trimesh.repair.fix_normals(mesh)

        # Assign per-face colors based on face normal direction
        face_colors = np.zeros((len(mesh.faces), 4), dtype=np.uint8)
        for i, fn in enumerate(mesh.face_normals):
            base = np.array(_FACADE_COLORS.get(_classify_direction(fn), [0.5, 0.5, 0.5]))
            noise = np.random.uniform(-0.04, 0.04, 3)
            c = np.clip(base + noise, 0, 1)
            face_colors[i] = [int(c[0]*255), int(c[1]*255), int(c[2]*255), 255]
        mesh.visual.face_colors = face_colors
        mesh.export(str(out_path))
        return

    # --- Fallback for preset buildings: extrude facade slabs ---
    THICKNESS = 0.25

    all_verts, all_tris, all_colors = [], [], []
    offset = 0

    for facade in building.facades:
        verts = np.asarray(facade.vertices, dtype=np.float64)
        n = len(verts)
        if n < 3:
            continue

        normal = np.asarray(facade.normal, dtype=np.float64)
        base_color = np.array(
            _FACADE_COLORS.get(_classify_direction(normal), [0.5, 0.5, 0.5])
        )

        front = verts
        back = verts - normal * THICKNESS

        f0 = offset
        b0 = offset + n
        for v in front:
            all_verts.append(v)
            noise = np.random.uniform(-0.05, 0.05, 3)
            all_colors.append(np.clip(base_color + noise, 0, 1))
        for v in back:
            all_verts.append(v)
            noise = np.random.uniform(-0.05, 0.05, 3)
            all_colors.append(np.clip(base_color * 0.7 + noise, 0, 1))

        for i in range(1, n - 1):
            all_tris.append([f0, f0 + i, f0 + i + 1])
        for i in range(1, n - 1):
            all_tris.append([b0, b0 + i + 1, b0 + i])
        for i in range(n):
            j = (i + 1) % n
            all_tris.append([f0 + i, f0 + j, b0 + j])
            all_tris.append([f0 + i, b0 + j, b0 + i])

        offset += 2 * n

    colors_u8 = (np.array(all_colors) * 255).astype(np.uint8)
    mesh = trimesh.Trimesh(
        vertices=np.array(all_verts),
        faces=np.array(all_tris),
        vertex_colors=colors_u8,
    )
    mesh.export(str(out_path))


def _write_render_input(
    waypoints: list[Waypoint],
    camera: CameraSpec,
    render_scale: float,
    voxel_size: float,
    out_path: Path,
):
    """Write waypoint poses + camera intrinsics as JSON for the subprocess."""
    wps = []
    for wp in waypoints:
        if wp.is_transition:
            continue
        wps.append({
            "index": wp.index,
            "x": wp.x, "y": wp.y, "z": wp.z,
            "heading_deg": wp.heading_deg,
            "gimbal_pitch_deg": wp.gimbal_pitch_deg,
            "facade_index": wp.facade_index,
        })

    data = {
        "waypoints": wps,
        "camera": {
            "image_width_px": camera.image_width_px,
            "image_height_px": camera.image_height_px,
            "focal_length_mm": camera.focal_length_mm,
            "sensor_width_mm": camera.sensor_width_mm,
            "sensor_height_mm": camera.sensor_height_mm,
        },
        "render_scale": render_scale,
        "voxel_size": voxel_size,
    }
    out_path.write_text(json.dumps(data))


# ---------------------------------------------------------------------------
# Subprocess render script (runs as its own process → main thread)
# ---------------------------------------------------------------------------

_RENDER_SCRIPT = r'''
"""Subprocess: pyrender RGB+depth rendering + Open3D TSDF fusion.

Uses pyrender (pyglet/macOS OpenGL) for rendering — works on macOS.
Uses Open3D only for TSDF fusion (no GPU context needed).
"""
import json, math, os, sys
import numpy as np

DEPTH_NOISE_STD_M  = 0.005
POSE_NOISE_POS_M   = 0.01
POSE_NOISE_ROT_DEG = 0.15
DEPTH_MAX_M        = 40.0


def _wp_pose_opengl(wp, add_noise=False):
    """Camera-to-world pose in OpenGL convention (pyrender expects this).

    OpenGL: X=right, Y=up, -Z=forward.
    ENU:    X=east, Y=north, Z=up.
    """
    hh = math.radians(wp["heading_deg"])
    pp = math.radians(wp["gimbal_pitch_deg"])

    # Forward in ENU
    fwd = np.array([math.sin(hh)*math.cos(pp), math.cos(hh)*math.cos(pp), math.sin(pp)])
    fwd /= np.linalg.norm(fwd)

    world_up = np.array([0.0, 0.0, 1.0])
    right = np.cross(fwd, world_up)
    rn = np.linalg.norm(right)
    if rn < 1e-6:
        right = np.array([1.0, 0.0, 0.0])
    else:
        right /= rn
    up = np.cross(right, fwd)
    up /= np.linalg.norm(up)

    pos = np.array([wp["x"], wp["y"], wp["z"]])

    if add_noise:
        pos = pos + np.random.normal(0, POSE_NOISE_POS_M, 3)
        axis = np.random.randn(3); axis /= np.linalg.norm(axis) + 1e-9
        ang = np.radians(np.random.normal(0, POSE_NOISE_ROT_DEG))
        K = np.array([[0,-axis[2],axis[1]],[axis[2],0,-axis[0]],[-axis[1],axis[0],0]])
        dR = np.eye(3) + np.sin(ang)*K + (1-np.cos(ang))*K@K
        right = dR @ right; up = dR @ up; fwd = dR @ fwd

    # Camera-to-world: columns = right, up, -forward, pos  (OpenGL)
    c2w = np.eye(4)
    c2w[:3, 0] = right
    c2w[:3, 1] = up
    c2w[:3, 2] = -fwd   # OpenGL looks along -Z
    c2w[:3, 3] = pos
    return c2w


def _c2w_to_w2c_cv(c2w):
    """Convert OpenGL camera-to-world → OpenCV world-to-camera (for TSDF).

    OpenGL: X=right, Y=up, -Z=fwd   →   OpenCV: X=right, Y=down, Z=fwd
    1. Invert c2w → w2c in OpenGL convention
    2. Pre-multiply flip to convert camera-space axes from OpenGL to OpenCV
    """
    flip = np.diag([1, -1, -1, 1]).astype(np.float64)
    w2c = flip @ np.linalg.inv(c2w)
    return w2c


def main():
    mesh_path, input_json, output_mesh, images_dir = sys.argv[1:5]

    import trimesh, pyrender
    from PIL import Image
    import open3d as o3d

    with open(input_json) as f:
        data = json.load(f)

    cam_data = data["camera"]
    scale = data["render_scale"]
    voxel_size = data["voxel_size"]
    waypoints = data["waypoints"]

    w = max(1, int(cam_data["image_width_px"] * scale))
    h = max(1, int(cam_data["image_height_px"] * scale))
    fx = cam_data["focal_length_mm"] / cam_data["sensor_width_mm"] * w
    fy = cam_data["focal_length_mm"] / cam_data["sensor_height_mm"] * h

    # --- pyrender scene ---
    tm = trimesh.load(mesh_path)
    if isinstance(tm, trimesh.Scene):
        tm = trimesh.util.concatenate([g for g in tm.geometry.values()])
    trimesh.repair.fix_normals(tm)

    scene = pyrender.Scene(bg_color=[140, 184, 224, 255], ambient_light=[0.3, 0.3, 0.3])
    scene.add(pyrender.Mesh.from_trimesh(tm, smooth=False))
    scene.add(pyrender.DirectionalLight(color=[1,1,1], intensity=4.0),
              pose=np.eye(4))

    cam = pyrender.IntrinsicsCamera(fx=fx, fy=fy, cx=w/2, cy=h/2, znear=0.1, zfar=80.0)
    cam_node = scene.add(cam, pose=np.eye(4))

    renderer = pyrender.OffscreenRenderer(w, h)

    # --- Open3D TSDF (CPU only, no GPU context) ---
    intrinsic = o3d.camera.PinholeCameraIntrinsic(w, h, fx, fy, w/2, h/2)
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=voxel_size, sdf_trunc=voxel_size * 5,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
    )

    total = len(waypoints)
    photos = []

    for i, wp in enumerate(waypoints):
        # Render from TRUE pose
        c2w_true = _wp_pose_opengl(wp, add_noise=False)
        scene.set_pose(cam_node, c2w_true)
        color, depth = renderer.render(scene)

        # Save photo
        img_name = f"wp_{wp['index']:04d}.png"
        img_path = os.path.join(images_dir, img_name)
        Image.fromarray(color).save(img_path)

        photos.append({
            "index": wp["index"], "path": img_path,
            "facade_index": wp["facade_index"],
            "position": [round(wp["x"],2), round(wp["y"],2), round(wp["z"],2)],
            "heading": round(wp["heading_deg"],1),
            "gimbal_pitch": round(wp["gimbal_pitch_deg"],1),
        })

        # TSDF: noisy pose + noisy depth
        c2w_noisy = _wp_pose_opengl(wp, add_noise=True)
        w2c = _c2w_to_w2c_cv(c2w_noisy)

        depth_f = depth.astype(np.float32)
        depth_f[depth_f > DEPTH_MAX_M] = 0.0
        valid = depth_f > 0
        noise = np.random.normal(0, DEPTH_NOISE_STD_M, depth_f.shape).astype(np.float32)
        depth_f[valid] += noise[valid] * (1.0 + depth_f[valid] * 0.05)
        depth_f[depth_f < 0] = 0.0

        color_o3d = o3d.geometry.Image(color.copy())
        depth_o3d = o3d.geometry.Image(depth_f)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_o3d, depth_o3d,
            depth_scale=1.0, depth_trunc=DEPTH_MAX_M,
            convert_rgb_to_intensity=False,
        )
        volume.integrate(rgbd, intrinsic, w2c)

        if (i + 1) % max(1, total // 10) == 0:
            print(f"PROGRESS {i+1}/{total}", flush=True)

    renderer.delete()

    recon = volume.extract_triangle_mesh()
    recon.compute_vertex_normals()
    o3d.io.write_triangle_mesh(output_mesh, recon)

    meta_path = output_mesh.replace(".ply", "_photos.json")
    with open(meta_path, "w") as f:
        json.dump(photos, f)

    print(f"DONE verts={len(recon.vertices)} tris={len(recon.triangles)}", flush=True)

if __name__ == "__main__":
    main()
'''


# ---------------------------------------------------------------------------
# Render via subprocess or fallback
# ---------------------------------------------------------------------------

def _run_render_subprocess(
    building: Building,
    waypoints: list[Waypoint],
    camera: CameraSpec,
    output_dir: Path,
    task: SimulationTask | None,
    render_scale: float,
    voxel_size: float,
    raw_mesh: dict | None = None,
) -> tuple[Path, list[dict]]:
    """Spawn a subprocess for Open3D rendering (gets its own main thread)."""
    mesh_ply = output_dir / "original.ply"
    input_json = output_dir / "render_input.json"
    recon_ply = output_dir / "reconstructed.ply"
    images_dir = output_dir / "images"
    images_dir.mkdir(exist_ok=True)

    # Write inputs for subprocess — use raw mesh if available
    _export_render_mesh(building, mesh_ply, raw_mesh=raw_mesh)
    _write_render_input(waypoints, camera, render_scale, voxel_size, input_json)

    # Write the render script to /tmp (NOT in project dir — uvicorn's
    # file watcher would see it and restart the server, killing the task).
    import tempfile
    script_path = Path(tempfile.gettempdir()) / f"aeroscan_render_{output_dir.name}.py"
    script_path.write_text(_RENDER_SCRIPT)

    inspection_count = sum(1 for wp in waypoints if not wp.is_transition)

    proc = subprocess.Popen(
        [sys.executable, str(script_path),
         str(mesh_ply), str(input_json), str(recon_ply), str(images_dir)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        cwd=str(Path(__file__).parent.parent.parent),  # project root for imports
    )

    # Read progress from stdout
    while True:
        line = proc.stdout.readline()
        if not line and proc.poll() is not None:
            break
        line = line.strip()
        if line.startswith("PROGRESS ") and task:
            parts = line.split()[1].split("/")
            done, total = int(parts[0]), int(parts[1])
            _update_task(task,
                         progress=0.1 + 0.6 * done / total,
                         message=f"Rendered & fused {done}/{total} photos")
        elif line.startswith("DONE"):
            logger.info("Subprocess: %s", line)

    rc = proc.wait()
    if rc != 0:
        stderr = proc.stderr.read()
        raise RuntimeError(f"Render subprocess failed (rc={rc}): {stderr[-500:]}")

    # Read photo metadata
    photos_json = recon_ply.with_name("reconstructed_photos.json")
    photos: list[dict] = []
    if photos_json.exists():
        photos = json.loads(photos_json.read_text())

    return recon_ply, photos


def _reconstruct_fallback(
    building: Building,
    waypoints: list[Waypoint],
    output_dir: Path,
    noise_std: float = 0.03,
) -> tuple[Path, list[dict]]:
    """Fallback: export mesh with gaussian noise, no rendering.

    Simulates photogrammetric reconstruction artifacts by perturbing
    vertices. Still exercises the full reimport pipeline.
    """
    import trimesh

    verts_list, tris_list = [], []
    offset = 0
    for facade in building.facades:
        v = np.asarray(facade.vertices, dtype=np.float64)
        n = len(v)
        if n < 3:
            continue
        for i in range(1, n - 1):
            tris_list.append([offset, offset + i, offset + i + 1])
        for vert in v:
            verts_list.append(vert)
        offset += n

    verts = np.array(verts_list)
    faces = np.array(tris_list)

    # Subdivide for denser geometry (more realistic reconstruction output)
    mesh = trimesh.Trimesh(vertices=verts, faces=faces)
    for _ in range(2):
        mesh = mesh.subdivide()

    # Add gaussian noise to simulate reconstruction imprecision
    mesh.vertices += np.random.normal(0, noise_std, mesh.vertices.shape)

    path = output_dir / "reconstructed.ply"
    mesh.export(str(path))

    # Generate photo metadata (no actual images)
    photos = []
    for wp in waypoints:
        if wp.is_transition:
            continue
        photos.append({
            "index": wp.index,
            "path": "",
            "facade_index": wp.facade_index,
            "position": [round(wp.x, 2), round(wp.y, 2), round(wp.z, 2)],
            "heading": round(wp.heading_deg, 1),
            "gimbal_pitch": round(wp.gimbal_pitch_deg, 1),
        })

    return path, photos


# ---------------------------------------------------------------------------
# Main simulation
# ---------------------------------------------------------------------------

def run_simulation(
    building: Building,
    waypoints: list[Waypoint],
    config: MissionConfig,
    algo: AlgorithmConfig,
    task: SimulationTask | None = None,
    render_scale: float = 0.1,
    voxel_size: float = 0.03,
    raw_mesh: dict | None = None,
) -> dict:
    """Run the full simulate-reconstruct-reimport pipeline."""
    output_dir = Path("sim_output") / (task.task_id if task else "default")
    output_dir.mkdir(parents=True, exist_ok=True)

    camera = CAMERAS[config.camera]

    # --- Phase 1: Render + reconstruct ---
    if task:
        _update_task(task, status="rendering", progress=0.1,
                     message="Rendering synthetic photos & fusing depth maps…")

    used_method = "tsdf_fusion"
    try:
        mesh_path, photos = _run_render_subprocess(
            building, waypoints, camera, output_dir, task,
            render_scale=render_scale, voxel_size=voxel_size,
            raw_mesh=raw_mesh,
        )
    except Exception as e:
        logger.warning("Render subprocess failed (%s), using fallback", e)
        if task:
            _update_task(task, status="reconstructing", progress=0.5,
                         message="Render failed — using mesh perturbation fallback")
        mesh_path, photos = _reconstruct_fallback(building, waypoints, output_dir)
        used_method = "noise_fallback"

    # --- Phase 2: Decimate + reimport reconstructed mesh ---
    if task:
        _update_task(task, status="importing", progress=0.75,
                     message="Decimating & importing reconstructed mesh…")

    # TSDF at 2cm voxels produces huge meshes (millions of faces).
    # Decimate to a manageable size for facade extraction.
    import trimesh as _tm
    recon_tm = _tm.load(str(mesh_path))
    target_faces = 50_000
    if len(recon_tm.faces) > target_faces:
        ratio = 1.0 - target_faces / len(recon_tm.faces)
        logger.info("Decimating TSDF mesh: %d → ~%d faces (%.0f%% reduction)",
                     len(recon_tm.faces), target_faces, ratio * 100)
        import fast_simplification
        verts, faces = fast_simplification.simplify(
            recon_tm.vertices, recon_tm.faces, target_reduction=ratio)
        recon_tm = _tm.Trimesh(vertices=verts, faces=faces)
        decimated_path = mesh_path.with_name("reconstructed_decimated.ply")
        recon_tm.export(str(decimated_path))
        mesh_path = decimated_path

    mesh_data = mesh_path.read_bytes()
    recon_building = build_building_from_mesh(
        mesh_data=mesh_data,
        file_type="ply",
        lat=building.lat,
        lon=building.lon,
        height=None,
        name=f"{building.label or 'building'}_reconstructed",
    )

    # --- Phase 3: Generate new mission from reconstructed building ---
    if task:
        _update_task(task, status="generating", progress=0.85,
                     message="Generating mission from reconstructed building…")

    recon_waypoints, _gen_stats = generate_mission_waypoints(
        recon_building, config, algo,
    )

    # --- Phase 4: Build viewer data + comparison ---
    if task:
        _update_task(task, progress=0.95, message="Preparing results…")

    threejs = prepare_threejs_data(recon_building, recon_waypoints)
    if hasattr(recon_building, "_mesh_viewer_data") and recon_building._mesh_viewer_data:
        threejs["rawMesh"] = recon_building._mesh_viewer_data

    leaflet = prepare_leaflet_data(recon_building, recon_waypoints)

    n_insp_orig = sum(1 for w in waypoints if not w.is_transition)
    n_insp_recon = sum(1 for w in recon_waypoints if not w.is_transition)

    comparison = {
        "original": {
            "facade_count": len(building.facades),
            "dimensions": [building.width, building.depth, building.height],
            "inspection_waypoints": n_insp_orig,
        },
        "reconstructed": {
            "facade_count": len(recon_building.facades),
            "dimensions": [recon_building.width, recon_building.depth, recon_building.height],
            "inspection_waypoints": n_insp_recon,
        },
        "diff": {
            "facade_count": len(recon_building.facades) - len(building.facades),
            "width_m": round(recon_building.width - building.width, 2),
            "depth_m": round(recon_building.depth - building.depth, 2),
            "height_m": round(recon_building.height - building.height, 2),
            "waypoint_diff": n_insp_recon - n_insp_orig,
        },
        "method": used_method,
        "render_scale": render_scale,
        "voxel_size_m": voxel_size,
        "num_photos": len(photos),
    }

    return {
        "viewer_data": {"threejs": threejs, "leaflet": leaflet},
        "summary": {
            "waypoint_count": len(recon_waypoints),
            "inspection_waypoints": n_insp_recon,
            "facade_count": len(recon_building.facades),
            "building_dims": f"{recon_building.width}m × {recon_building.depth}m × {recon_building.height}m",
        },
        "comparison": comparison,
        "photos": photos[:20],
        "photos_total": len(photos),
        "output_dir": str(output_dir),
        "mesh_path": str(mesh_path),
    }


def start_simulation_async(
    building: Building,
    waypoints: list[Waypoint],
    config: MissionConfig,
    algo: AlgorithmConfig,
    task_id: str,
    version_id: str | None = None,
    render_scale: float = 0.1,
    voxel_size: float = 0.03,
    raw_mesh: dict | None = None,
):
    """Launch simulation in a background thread. Poll via get_task()."""
    task = SimulationTask(task_id=task_id, version_id=version_id)
    with _task_lock:
        _tasks[task_id] = task

    def _run():
        try:
            _update_task(task, status="rendering", progress=0.05,
                         message="Starting simulation…")
            result = run_simulation(
                building, waypoints, config, algo, task,
                render_scale=render_scale, voxel_size=voxel_size,
                raw_mesh=raw_mesh,
            )
            _update_task(task, status="complete", progress=1.0,
                         message="Reconstruction complete", result=result)
            # Persist to database
            try:
                _save_to_db(task)
            except Exception:
                logger.warning("Failed to save simulation to DB", exc_info=True)
        except Exception as e:
            logger.exception("Simulation failed")
            _update_task(task, status="error", progress=0.0,
                         message=str(e), error=str(e))

    thread = threading.Thread(target=_run, daemon=True)
    thread.start()
