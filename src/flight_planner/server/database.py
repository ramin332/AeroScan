"""Database layer for persistent storage.

Uses SQLAlchemy with SQLite by default (zero-config development).
Set DATABASE_URL env var for PostgreSQL in production:
    DATABASE_URL=postgresql+psycopg2://user:pass@host/aeroscan
"""

from __future__ import annotations

import json
import os
import uuid
from datetime import datetime, timezone
from pathlib import Path

from sqlalchemy import Column, Float, Integer, String, Text, create_engine
from sqlalchemy.orm import DeclarativeBase, Session, sessionmaker

_PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent.parent
_DEFAULT_DB = f"sqlite:///{_PROJECT_ROOT / 'aeroscan.db'}"
_DATABASE_URL = os.environ.get("DATABASE_URL", _DEFAULT_DB)

engine = create_engine(_DATABASE_URL, echo=False)
SessionLocal = sessionmaker(bind=engine)


class Base(DeclarativeBase):
    pass


class BuildingRecord(Base):
    __tablename__ = "buildings"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    name = Column(String, nullable=False)
    source_type = Column(String, default="geojson")
    geometry_data = Column(Text, nullable=True)  # raw GeoJSON string
    lat = Column(Float, default=0.0)
    lon = Column(Float, default=0.0)
    height = Column(Float, default=8.0)
    num_stories = Column(Integer, default=1)
    roof_type = Column(String, default="flat")
    roof_pitch_deg = Column(Float, default=0.0)
    heading_deg = Column(Float, default=0.0)
    properties_json = Column(Text, default="{}")
    created_at = Column(String, default=lambda: datetime.now(timezone.utc).isoformat())

    # Keys in properties_json that store multi-MB blobs. Stripped from list
    # responses so `GET /buildings` doesn't ship the whole mesh + point cloud
    # for every imported building.
    _HEAVY_PROP_KEYS = frozenset({
        "mesh_viewer",
        "ply_b64",
        "pc_b64",
        "waypoints_raw",
        "mission_area_wgs84",
        "mission_config_raw",
    })

    def to_dict(self, include_heavy: bool = False) -> dict:
        props = {}
        if self.properties_json:
            try:
                props = json.loads(self.properties_json)
            except (json.JSONDecodeError, TypeError):
                pass
        if not include_heavy:
            props = {k: v for k, v in props.items() if k not in self._HEAVY_PROP_KEYS}
        return {
            "id": self.id,
            "name": self.name,
            "source_type": self.source_type,
            "lat": self.lat,
            "lon": self.lon,
            "height": self.height,
            "width": props.get("width", 0),
            "depth": props.get("depth", 0),
            "num_stories": self.num_stories,
            "roof_type": self.roof_type,
            "roof_pitch_deg": self.roof_pitch_deg,
            "heading_deg": self.heading_deg,
            "properties": props,
            "created_at": self.created_at,
        }


class FacadeCacheRecord(Base):
    """JSON-serialized facade-extraction results keyed by (building_id, params_hash).

    Facade extraction for a large mesh involves HPR with ~48 camera viewpoints
    plus per-region raycasting — expensive enough that we want results to
    survive server restarts. The in-memory dict layered on top of this is still
    the hot path; this table is the cold-start fallback.
    """

    __tablename__ = "facade_cache"

    cache_key = Column(String, primary_key=True)       # "{building_id}:{params_md5}"
    building_id = Column(String, index=True, nullable=False)
    payload_json = Column(Text, nullable=False)        # JSON {"facades":[...], "bbox":[...]}
    created_at = Column(String, default=lambda: datetime.now(timezone.utc).isoformat())


class SimulationRecord(Base):
    __tablename__ = "simulations"

    task_id = Column(String, primary_key=True)
    status = Column(String, default="complete")
    source_version = Column(String, nullable=True)
    result_json = Column(Text, nullable=True)  # full result dict as JSON
    output_dir = Column(String, nullable=True)
    created_at = Column(String, default=lambda: datetime.now(timezone.utc).isoformat())

    def to_summary(self) -> dict:
        result = {}
        if self.result_json:
            try:
                result = json.loads(self.result_json)
            except (json.JSONDecodeError, TypeError):
                pass
        return {
            "task_id": self.task_id,
            "status": self.status,
            "comparison": result.get("comparison"),
            "summary": result.get("summary"),
            "created_at": self.created_at,
        }


def init_db() -> None:
    """Create all tables if they don't exist."""
    Base.metadata.create_all(engine)


def get_db() -> Session:
    """Get a database session. Caller must close it."""
    return SessionLocal()
