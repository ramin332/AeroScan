"""Frontend HTML page for the AeroScan debug server."""

from fastapi import APIRouter
from fastapi.responses import HTMLResponse

frontend_router = APIRouter()


@frontend_router.get("/", response_class=HTMLResponse)
def index():
    return FRONTEND_HTML


FRONTEND_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>AeroScan Flight Planner</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
:root {
  --bg: #0f1117; --bg2: #1a1d27; --bg3: #252833;
  --fg: #e1e4ed; --fg2: #9ca3b8; --fg3: #666d80;
  --accent: #3b82f6; --accent2: #60a5fa;
  --border: #2a2d3a; --radius: 6px;
}
* { margin: 0; padding: 0; box-sizing: border-box; }
body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; background: var(--bg); color: var(--fg); display: flex; height: 100vh; overflow: hidden; }

/* Sidebar */
.sidebar {
  width: 320px; min-width: 320px; background: var(--bg2); border-right: 1px solid var(--border);
  display: flex; flex-direction: column; overflow-y: auto;
}
.sidebar h1 { font-size: 16px; padding: 16px; border-bottom: 1px solid var(--border); }
.section { padding: 12px 16px; border-bottom: 1px solid var(--border); }
.section h3 { font-size: 11px; text-transform: uppercase; letter-spacing: 0.5px; color: var(--fg3); margin-bottom: 8px; }
.field { display: flex; align-items: center; gap: 8px; margin-bottom: 6px; }
.field label { font-size: 12px; color: var(--fg2); min-width: 90px; }
.field input, .field select {
  flex: 1; background: var(--bg3); border: 1px solid var(--border); border-radius: var(--radius);
  color: var(--fg); padding: 4px 8px; font-size: 12px; outline: none;
}
.field input:focus, .field select:focus { border-color: var(--accent); }
.field input[type=range] { padding: 0; }
.field .val { font-size: 11px; color: var(--fg3); min-width: 40px; text-align: right; }
select { cursor: pointer; }
.btn-primary {
  width: 100%; padding: 8px; background: var(--accent); color: #fff; border: none;
  border-radius: var(--radius); cursor: pointer; font-size: 13px; font-weight: 600;
}
.btn-primary:hover { background: var(--accent2); }
.btn-primary:disabled { opacity: 0.5; cursor: not-allowed; }
.btn-secondary {
  width: 100%; padding: 6px; background: var(--bg3); color: var(--fg2); border: 1px solid var(--border);
  border-radius: var(--radius); cursor: pointer; font-size: 12px; margin-top: 6px;
}
.btn-secondary:hover { background: var(--border); color: var(--fg); }

/* Summary stats */
.stats { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; margin-top: 8px; }
.stat-box { background: var(--bg3); border-radius: var(--radius); padding: 8px; }
.stat-box .num { font-size: 18px; font-weight: 700; color: var(--accent2); }
.stat-box .lbl { font-size: 10px; color: var(--fg3); text-transform: uppercase; }

/* Version list */
.version-list { max-height: 200px; overflow-y: auto; }
.version-item {
  display: flex; justify-content: space-between; align-items: center;
  padding: 6px 8px; border-radius: var(--radius); cursor: pointer; font-size: 12px;
  margin-bottom: 2px;
}
.version-item:hover { background: var(--bg3); }
.version-item.active { background: var(--accent); color: #fff; }
.version-item .ts { color: var(--fg3); font-size: 10px; }
.version-item .wp { font-size: 11px; }
.version-item .del { color: var(--fg3); cursor: pointer; padding: 2px 4px; border-radius: 3px; }
.version-item .del:hover { background: #dc2626; color: #fff; }

/* Main area */
.main { flex: 1; display: flex; flex-direction: column; }
.tabs {
  display: flex; background: var(--bg2); border-bottom: 1px solid var(--border);
}
.tab {
  padding: 10px 20px; font-size: 13px; color: var(--fg3); cursor: pointer;
  border-bottom: 2px solid transparent;
}
.tab:hover { color: var(--fg); }
.tab.active { color: var(--accent2); border-bottom-color: var(--accent); }

.view-container { flex: 1; position: relative; }
.view { position: absolute; top: 0; left: 0; right: 0; bottom: 0; display: none; }
.view.active { display: block; }
#view3d canvas { width: 100% !important; height: 100% !important; }
#viewMap { z-index: 1; }
#viewMap .leaflet-container { height: 100%; width: 100%; background: var(--bg); }

/* Info popup */
#wpInfo {
  position: absolute; bottom: 12px; left: 12px; z-index: 100;
  background: rgba(15,17,23,0.92); border-radius: var(--radius); color: var(--fg2);
  padding: 10px 14px; font-size: 12px; display: none; backdrop-filter: blur(8px);
  border: 1px solid var(--border);
}
#wpInfo b { color: var(--fg); }

/* Legend in 3D view */
.legend-3d {
  position: absolute; top: 12px; right: 12px; z-index: 100;
  background: rgba(15,17,23,0.85); border-radius: var(--radius); padding: 10px 14px;
  font-size: 12px; backdrop-filter: blur(8px); border: 1px solid var(--border);
  max-height: 50vh; overflow-y: auto;
}
.legend-3d .legend-item { display: flex; align-items: center; gap: 6px; padding: 2px 0; cursor: pointer; }
.legend-3d .legend-dot { width: 8px; height: 8px; border-radius: 50%; }
.legend-3d .legend-item.off span { color: var(--fg3); text-decoration: line-through; }
.legend-3d .legend-item.off .legend-dot { opacity: 0.15; }

/* Empty state */
.empty-state {
  display: flex; align-items: center; justify-content: center; height: 100%;
  color: var(--fg3); font-size: 14px;
}
</style>
</head>
<body>

<div class="sidebar">
  <h1>AeroScan Flight Planner</h1>

  <div class="section" style="padding:10px 16px">
    <h3>Drone: DJI Matrice 4E</h3>
    <div style="font-size:11px; color:var(--fg3); line-height:1.7; margin-top:4px">
      <span style="color:var(--fg2)">Wide</span> 24mm f/2.8 &middot; 20MP &middot; 4/3" CMOS<br>
      <span style="color:var(--fg2)">Tele</span> 70mm f/2.8 &middot; 48MP &middot; 1/1.3"<br>
      <span style="color:var(--fg2)">Zoom</span> 168mm &middot; 48MP &middot; 1/1.5"<br>
      Gimbal: -90&deg; to +35&deg; tilt &middot; &plusmn;60&deg; pan<br>
      Max speed: 21 m/s &middot; Min alt: 2m<br>
      Flight time w/ Manifold 3: ~32 min<br>
      Max waypoints: 65,535
    </div>
  </div>

  <div class="section">
    <h3>Preset</h3>
    <div class="field">
      <select id="preset" style="width:100%">
        <option value="">Custom building</option>
        <option value="simple_box" selected>Simple box (20x10x8m)</option>
        <option value="pitched_roof_house">Pitched roof (30x10x6m)</option>
        <option value="l_shaped_block">L-shaped block</option>
        <option value="large_apartment_block">Large apartment (60x12x18m)</option>
      </select>
    </div>
  </div>

  <div class="section" id="buildingSection">
    <h3>Building</h3>
    <div class="field"><label>Width (m)</label><input type="number" id="bWidth" value="20" min="1" max="200" step="1"><span class="val" id="bWidthVal"></span></div>
    <div class="field"><label>Depth (m)</label><input type="number" id="bDepth" value="10" min="1" max="200" step="1"><span class="val" id="bDepthVal"></span></div>
    <div class="field"><label>Height (m)</label><input type="number" id="bHeight" value="8" min="1" max="100" step="0.5"><span class="val" id="bHeightVal"></span></div>
    <div class="field"><label>Heading</label><input type="range" id="bHeading" value="0" min="0" max="360" step="5"><span class="val" id="bHeadingVal">0</span></div>
    <div class="field"><label>Roof</label><select id="bRoof"><option value="flat">Flat</option><option value="pitched">Pitched</option></select></div>
    <div class="field" id="pitchRow" style="display:none"><label>Pitch angle</label><input type="range" id="bPitch" value="30" min="5" max="60" step="5"><span class="val" id="bPitchVal">30</span></div>
    <div class="field"><label>Lat</label><input type="number" id="bLat" value="53.2012" step="0.0001"></div>
    <div class="field"><label>Lon</label><input type="number" id="bLon" value="5.7999" step="0.0001"></div>
  </div>

  <div class="section">
    <h3>Mission</h3>
    <div class="field"><label>GSD (mm/px)</label><input type="range" id="mGsd" value="2.0" min="0.5" max="10" step="0.5"><span class="val" id="mGsdVal">2.0</span></div>
    <div class="field"><label>Camera</label><select id="mCamera"><option value="wide">Wide 24mm</option><option value="medium_tele">Medium tele 70mm</option><option value="telephoto">Telephoto 168mm</option></select></div>
    <div class="field"><label>Front overlap</label><input type="range" id="mFront" value="0.80" min="0.3" max="0.95" step="0.05"><span class="val" id="mFrontVal">80%</span></div>
    <div class="field"><label>Side overlap</label><input type="range" id="mSide" value="0.70" min="0.3" max="0.95" step="0.05"><span class="val" id="mSideVal">70%</span></div>
    <div class="field"><label>Speed (m/s)</label><input type="range" id="mSpeed" value="3" min="0.5" max="21" step="0.5"><span class="val" id="mSpeedVal">3.0</span></div>
  </div>

  <div class="section">
    <button class="btn-primary" id="btnGenerate">Generate Mission</button>
    <button class="btn-secondary" id="btnDownload" disabled>Download KMZ</button>
    <div class="stats" id="statsArea" style="display:none">
      <div class="stat-box"><div class="num" id="statWp">-</div><div class="lbl">Waypoints</div></div>
      <div class="stat-box"><div class="num" id="statFacades">-</div><div class="lbl">Facades</div></div>
      <div class="stat-box"><div class="num" id="statDist">-</div><div class="lbl">Distance (m)</div></div>
      <div class="stat-box"><div class="num" id="statTime">-</div><div class="lbl">Est. time (s)</div></div>
    </div>
  </div>

  <div class="section">
    <h3>Version History</h3>
    <div class="version-list" id="versionList"></div>
  </div>
</div>

<div class="main">
  <div class="tabs">
    <div class="tab active" data-tab="view3d">3D Viewer</div>
    <div class="tab" data-tab="viewMap">2D Satellite</div>
  </div>
  <div class="view-container">
    <div class="view active" id="view3d">
      <div class="empty-state" id="empty3d">Click "Generate Mission" to start</div>
      <canvas id="c3d" style="display:none"></canvas>
      <div class="legend-3d" id="legend3d" style="display:none"></div>
      <div id="wpInfo"></div>
    </div>
    <div class="view" id="viewMap">
      <div class="empty-state" id="emptyMap">Click "Generate Mission" to start</div>
      <div id="mapContainer" style="height:100%;width:100%;display:none"></div>
    </div>
  </div>
</div>

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

// --- State ---
let currentVersion = null;
let leafletMap = null;
let leafletLayers = [];
let threeScene = null, threeCamera = null, threeRenderer = null, threeControls = null;
let threeObjects = []; // track for cleanup
let animId = null;
let leafletBounds = null;

// --- Slider value displays ---
function wireSlider(id, valId, fmt) {
  var el = document.getElementById(id);
  var valEl = document.getElementById(valId);
  if (!el || !valEl) return;
  function update() { valEl.textContent = fmt ? fmt(el.value) : el.value; }
  el.addEventListener('input', update);
  update();
}
wireSlider('bHeading', 'bHeadingVal');
wireSlider('bPitch', 'bPitchVal');
wireSlider('mGsd', 'mGsdVal');
wireSlider('mFront', 'mFrontVal', function(v) { return Math.round(v*100)+'%'; });
wireSlider('mSide', 'mSideVal', function(v) { return Math.round(v*100)+'%'; });
wireSlider('mSpeed', 'mSpeedVal');

// Roof type toggle
document.getElementById('bRoof').addEventListener('change', function(e) {
  document.getElementById('pitchRow').style.display = e.target.value === 'pitched' ? 'flex' : 'none';
});

// Preset loading
document.getElementById('preset').addEventListener('change', function(e) {
  var presets = {
    simple_box: {w:20,d:10,h:8,heading:0,roof:'flat',pitch:0},
    pitched_roof_house: {w:30,d:10,h:6,heading:45,roof:'pitched',pitch:30},
    l_shaped_block: {w:25,d:10,h:9,heading:0,roof:'flat',pitch:0},
    large_apartment_block: {w:60,d:12,h:18,heading:15,roof:'flat',pitch:0},
  };
  var p = presets[e.target.value];
  if (!p) return;
  document.getElementById('bWidth').value = p.w;
  document.getElementById('bDepth').value = p.d;
  document.getElementById('bHeight').value = p.h;
  document.getElementById('bHeading').value = p.heading;
  document.getElementById('bHeadingVal').textContent = p.heading;
  document.getElementById('bRoof').value = p.roof;
  document.getElementById('bPitch').value = p.pitch;
  document.getElementById('bPitchVal').textContent = p.pitch;
  document.getElementById('pitchRow').style.display = p.roof === 'pitched' ? 'flex' : 'none';
  document.getElementById('buildingSection').style.display = e.target.value === 'l_shaped_block' ? 'none' : 'block';
});

// --- Tab switching ---
document.querySelectorAll('.tab').forEach(function(tab) {
  tab.addEventListener('click', function() {
    document.querySelectorAll('.tab').forEach(function(t) { t.classList.remove('active'); });
    document.querySelectorAll('.view').forEach(function(v) { v.classList.remove('active'); });
    tab.classList.add('active');
    document.getElementById(tab.dataset.tab).classList.add('active');
    if (tab.dataset.tab === 'viewMap' && leafletMap) {
      setTimeout(function() {
        leafletMap.invalidateSize();
        if (leafletBounds) leafletMap.fitBounds(leafletBounds);
      }, 150);
    }
  });
});

// --- Generate ---
document.getElementById('btnGenerate').addEventListener('click', generate);

async function generate() {
  var btn = document.getElementById('btnGenerate');
  btn.disabled = true;
  btn.textContent = 'Generating...';

  var presetVal = document.getElementById('preset').value;
  var body = {
    building: {
      lat: parseFloat(document.getElementById('bLat').value),
      lon: parseFloat(document.getElementById('bLon').value),
      width: parseFloat(document.getElementById('bWidth').value),
      depth: parseFloat(document.getElementById('bDepth').value),
      height: parseFloat(document.getElementById('bHeight').value),
      heading_deg: parseFloat(document.getElementById('bHeading').value),
      roof_type: document.getElementById('bRoof').value,
      roof_pitch_deg: parseFloat(document.getElementById('bPitch').value),
    },
    mission: {
      target_gsd_mm_per_px: parseFloat(document.getElementById('mGsd').value),
      camera: document.getElementById('mCamera').value,
      front_overlap: parseFloat(document.getElementById('mFront').value),
      side_overlap: parseFloat(document.getElementById('mSide').value),
      flight_speed_ms: parseFloat(document.getElementById('mSpeed').value),
    },
  };
  if (presetVal) body.preset = presetVal;

  try {
    var resp = await fetch('/api/generate', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(body),
    });
    var data = await resp.json();
    currentVersion = data;
    updateStats(data.summary);
    render3D(data.viewer_data.threejs);
    renderMap(data.viewer_data.leaflet);
    updateVersionList();
    document.getElementById('btnDownload').disabled = false;
  } catch(err) {
    console.error('Generate failed:', err);
  }

  btn.disabled = false;
  btn.textContent = 'Generate Mission';
}

// --- Stats ---
function updateStats(s) {
  document.getElementById('statsArea').style.display = 'grid';
  document.getElementById('statWp').textContent = s.waypoint_count;
  document.getElementById('statFacades').textContent = s.facade_count;
  document.getElementById('statDist').textContent = s.camera_distance_m;
  document.getElementById('statTime').textContent = s.estimated_flight_time_s;
}

// --- KMZ download ---
document.getElementById('btnDownload').addEventListener('click', function() {
  if (currentVersion) window.location.href = '/api/versions/' + currentVersion.version_id + '/kmz';
});

// --- Version list ---
async function updateVersionList() {
  var resp = await fetch('/api/versions');
  var data = await resp.json();
  var list = document.getElementById('versionList');
  list.textContent = '';
  data.versions.forEach(function(v) {
    var item = document.createElement('div');
    item.className = 'version-item' + (currentVersion && v.version_id === currentVersion.version_id ? ' active' : '');

    var left = document.createElement('span');
    left.className = 'ts';
    left.textContent = v.timestamp.split('T')[1].substring(0,8);

    var mid = document.createElement('span');
    mid.className = 'wp';
    mid.textContent = v.waypoint_count + ' wp';

    var del = document.createElement('span');
    del.className = 'del';
    del.textContent = '\u00d7';
    del.addEventListener('click', function(e) {
      e.stopPropagation();
      fetch('/api/versions/' + v.version_id, {method:'DELETE'}).then(updateVersionList);
    });

    item.appendChild(left);
    item.appendChild(mid);
    item.appendChild(del);
    item.addEventListener('click', function() { loadVersion(v.version_id); });
    list.appendChild(item);
  });
}

async function loadVersion(vid) {
  var resp = await fetch('/api/versions/' + vid);
  var data = await resp.json();
  currentVersion = data;
  updateStats(data.summary);
  render3D(data.viewer_data.threejs);
  renderMap(data.viewer_data.leaflet);
  updateVersionList();
  document.getElementById('btnDownload').disabled = false;
}

// ==================== THREE.JS 3D VIEWER ====================

function enu(x, y, z) { return new THREE.Vector3(x, z, -y); }

function clearThree() {
  threeObjects.forEach(function(obj) {
    if (obj.geometry) obj.geometry.dispose();
    if (obj.material) {
      if (Array.isArray(obj.material)) obj.material.forEach(function(m){m.dispose();});
      else obj.material.dispose();
    }
    if (obj.parent) obj.parent.remove(obj);
  });
  threeObjects = [];
}

function initThree() {
  if (threeRenderer) return;
  var canvas = document.getElementById('c3d');
  threeRenderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
  threeRenderer.setPixelRatio(window.devicePixelRatio);
  threeRenderer.setClearColor(0x0f1117);
  threeCamera = new THREE.PerspectiveCamera(50, 1, 0.1, 500);
  threeCamera.position.set(35, 25, 35);
  threeScene = new THREE.Scene();
  threeScene.fog = new THREE.Fog(0x0f1117, 80, 200);
  threeScene.add(new THREE.AmbientLight(0xffffff, 0.5));
  var dl = new THREE.DirectionalLight(0xffffff, 0.8);
  dl.position.set(20, 40, 30);
  threeScene.add(dl);
  threeScene.add(new THREE.HemisphereLight(0x4488cc, 0x332211, 0.3));
  threeControls = new OrbitControls(threeCamera, canvas);
  threeControls.enableDamping = true;
  threeControls.dampingFactor = 0.08;

  function resizeThree() {
    var container = document.getElementById('view3d');
    var w = container.clientWidth, h = container.clientHeight;
    threeRenderer.setSize(w, h);
    threeCamera.aspect = w / h;
    threeCamera.updateProjectionMatrix();
  }
  window.addEventListener('resize', resizeThree);
  new ResizeObserver(resizeThree).observe(document.getElementById('view3d'));
  resizeThree();

  function animate() { animId = requestAnimationFrame(animate); threeControls.update(); threeRenderer.render(threeScene, threeCamera); }
  animate();
}

function render3D(D) {
  document.getElementById('empty3d').style.display = 'none';
  document.getElementById('c3d').style.display = 'block';
  document.getElementById('legend3d').style.display = 'block';
  initThree();
  clearThree();

  // Ground grid
  var grid = new THREE.GridHelper(100, 50, 0x333355, 0x222244);
  threeScene.add(grid); threeObjects.push(grid);
  var groundGeo = new THREE.PlaneGeometry(100, 100);
  var groundMat = new THREE.MeshStandardMaterial({ color: 0x0f1117, roughness: 1 });
  var ground = new THREE.Mesh(groundGeo, groundMat);
  ground.rotation.x = -Math.PI / 2; ground.position.y = -0.01;
  threeScene.add(ground); threeObjects.push(ground);

  // Facades
  var facadeVis = {};
  D.facades.forEach(function(f) {
    var color = new THREE.Color(f.color);
    var verts = f.vertices;
    if (verts.length < 3) return;
    var positions = [];
    for (var i = 1; i < verts.length - 1; i++) {
      positions.push(verts[0][0], verts[0][2], -verts[0][1]);
      positions.push(verts[i][0], verts[i][2], -verts[i][1]);
      positions.push(verts[i+1][0], verts[i+1][2], -verts[i+1][1]);
    }
    var geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    geo.computeVertexNormals();
    var mesh = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({
      color: color, transparent: true, opacity: 0.35, side: THREE.DoubleSide, roughness: 0.8
    }));
    threeScene.add(mesh); threeObjects.push(mesh);

    var edgePos = [];
    for (var i = 0; i < verts.length; i++) {
      var a = verts[i], b = verts[(i+1)%verts.length];
      edgePos.push(a[0],a[2],-a[1], b[0],b[2],-b[1]);
    }
    var edgeGeo = new THREE.BufferGeometry();
    edgeGeo.setAttribute('position', new THREE.Float32BufferAttribute(edgePos, 3));
    var edges = new THREE.LineSegments(edgeGeo, new THREE.LineBasicMaterial({color:color}));
    threeScene.add(edges); threeObjects.push(edges);

    facadeVis[f.index] = { mesh: mesh, edges: edges, wpMesh: null, arrows: null };
  });

  // Waypoints per facade
  var wpSphereGeo = new THREE.SphereGeometry(0.25, 8, 6);
  var facadeIndices = {};
  D.waypoints.forEach(function(wp) {
    if (!facadeIndices[wp.facade_index]) facadeIndices[wp.facade_index] = [];
    facadeIndices[wp.facade_index].push(wp);
  });

  var wpLookup = [];
  Object.keys(facadeIndices).forEach(function(fi) {
    var wps = facadeIndices[fi];
    var fMeta = D.facades.find(function(f){return f.index===parseInt(fi);});
    var color = fMeta ? new THREE.Color(fMeta.color) : new THREE.Color(0x888888);

    var instMesh = new THREE.InstancedMesh(wpSphereGeo, new THREE.MeshStandardMaterial({
      color:color, roughness:0.4, metalness:0.1
    }), wps.length);
    var dummy = new THREE.Object3D();
    wps.forEach(function(wp, i) {
      dummy.position.copy(enu(wp.x, wp.y, wp.z));
      dummy.updateMatrix();
      instMesh.setMatrixAt(i, dummy.matrix);
      wpLookup.push({pos: enu(wp.x, wp.y, wp.z), wp: wp, fi: fi});
    });
    instMesh.instanceMatrix.needsUpdate = true;
    threeScene.add(instMesh); threeObjects.push(instMesh);

    var arrowGroup = new THREE.Group();
    wps.forEach(function(wp) {
      var origin = enu(wp.x, wp.y, wp.z);
      var hr = wp.heading*Math.PI/180, pr = wp.gimbal_pitch*Math.PI/180;
      var dir = enu(Math.sin(hr)*Math.cos(pr), Math.cos(hr)*Math.cos(pr), Math.sin(pr)).normalize();
      arrowGroup.add(new THREE.ArrowHelper(dir, origin, 2, color.getHex(), 0.3, 0.15));
    });
    threeScene.add(arrowGroup); threeObjects.push(arrowGroup);

    if (facadeVis[fi]) { facadeVis[fi].wpMesh = instMesh; facadeVis[fi].arrows = arrowGroup; }
  });

  // Flight path
  var pathPos = [];
  D.waypoints.forEach(function(wp) { var p=enu(wp.x,wp.y,wp.z); pathPos.push(p.x,p.y,p.z); });
  var pathGeo = new THREE.BufferGeometry();
  pathGeo.setAttribute('position', new THREE.Float32BufferAttribute(pathPos, 3));
  var pathLine = new THREE.Line(pathGeo, new THREE.LineBasicMaterial({color:0xffffff, opacity:0.3, transparent:true}));
  threeScene.add(pathLine); threeObjects.push(pathLine);

  // Camera
  threeControls.target.set(0, D.buildingHeight/2, 0);
  threeCamera.position.set(35, 25, 35);
  threeControls.update();

  // Legend
  var legendDiv = document.getElementById('legend3d');
  legendDiv.textContent = '';
  D.facades.forEach(function(f) {
    var count = (facadeIndices[f.index]||[]).length;
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
      var off = item.classList.toggle('off');
      var vis = facadeVis[f.index];
      if (vis) {
        if (vis.mesh) vis.mesh.visible = !off;
        if (vis.edges) vis.edges.visible = !off;
        if (vis.wpMesh) vis.wpMesh.visible = !off;
        if (vis.arrows) vis.arrows.visible = !off;
      }
    });
    legendDiv.appendChild(item);
  });

  // Click to inspect
  var raycaster = new THREE.Raycaster();
  var mouse = new THREE.Vector2();
  var infoDiv = document.getElementById('wpInfo');
  document.getElementById('c3d').addEventListener('click', function(e) {
    var rect = e.target.getBoundingClientRect();
    mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
    raycaster.setFromCamera(mouse, threeCamera);
    var best = 1.5, bestWp = null, bestFi = null;
    wpLookup.forEach(function(entry) {
      var d = raycaster.ray.distanceToPoint(entry.pos);
      if (d < best) { best = d; bestWp = entry.wp; bestFi = entry.fi; }
    });
    if (bestWp) {
      var fm = D.facades.find(function(f){return f.index===parseInt(bestFi);});
      infoDiv.style.display = 'block';
      infoDiv.textContent = '';
      var b = document.createElement('b');
      b.textContent = 'WP ' + bestWp.index;
      infoDiv.appendChild(b);
      var lines = [
        'Facade: '+(fm?fm.label:'?'), 'Pos: ('+bestWp.x+', '+bestWp.y+', '+bestWp.z+')m',
        'Heading: '+bestWp.heading+'\u00b0', 'Gimbal: '+bestWp.gimbal_pitch+'\u00b0',
        'Component: '+bestWp.component
      ];
      lines.forEach(function(l) { infoDiv.appendChild(document.createElement('br')); infoDiv.appendChild(document.createTextNode(l)); });
    } else { infoDiv.style.display = 'none'; }
  });
}

// ==================== LEAFLET 2D MAP ====================

function clearMap() {
  leafletLayers.forEach(function(l) { leafletMap.removeLayer(l); });
  leafletLayers = [];
}

function ensureMap(center) {
  if (!leafletMap) {
    document.getElementById('emptyMap').style.display = 'none';
    document.getElementById('mapContainer').style.display = 'block';
    leafletMap = L.map('mapContainer', {zoomControl: true}).setView(center, 19);
    L.tileLayer(
      'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
      {attribution:'Esri', maxZoom:21}
    ).addTo(leafletMap);
  }
}

function renderMap(D) {
  ensureMap(D.center);
  clearMap();
  leafletMap.setView(D.center, 19);

  // Building polygon
  if (D.buildingPoly && D.buildingPoly.length > 0) {
    var coords = D.buildingPoly.map(function(c){return [c[1],c[0]];});
    var poly = L.polygon(coords, {color:'#fff',weight:2,fillColor:'#334155',fillOpacity:0.4,dashArray:'4'}).addTo(leafletMap);
    poly.bindPopup(D.buildingLabel+'<br>'+D.buildingDims);
    leafletLayers.push(poly);
  }

  // Waypoints
  Object.keys(D.facadeGroups).forEach(function(fi) {
    var wps = D.facadeGroups[fi];
    var meta = D.facadeMeta[fi] || {label:'?',color:'#888'};
    wps.forEach(function(wp) {
      var c = L.circleMarker([wp.lat, wp.lon], {radius:4,color:meta.color,fillColor:meta.color,fillOpacity:0.8,weight:1}).addTo(leafletMap);
      c.bindPopup('WP '+wp.index+'<br>Facade: '+meta.label+'<br>Alt: '+wp.alt+'m<br>Heading: '+wp.heading+'\u00b0<br>Gimbal: '+wp.gimbal_pitch+'\u00b0');
      leafletLayers.push(c);
    });
  });

  // Flight path
  var path = L.polyline(D.flightPath.map(function(c){return [c[1],c[0]];}), {color:'#fff',weight:1,opacity:0.4,dashArray:'3 6'}).addTo(leafletMap);
  leafletLayers.push(path);

  // Fit bounds
  var pts = [];
  Object.values(D.facadeGroups).forEach(function(wps) { wps.forEach(function(wp){pts.push([wp.lat,wp.lon]);}); });
  if (pts.length) {
    leafletBounds = L.latLngBounds(pts).pad(0.15);
    leafletMap.fitBounds(leafletBounds);
  }

  setTimeout(function() {
    leafletMap.invalidateSize();
    if (leafletBounds) leafletMap.fitBounds(leafletBounds);
  }, 200);
}

// Auto-generate on load
generate();
</script>
</body>
</html>
"""
