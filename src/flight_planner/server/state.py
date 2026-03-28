"""In-memory version store for generated missions."""

from __future__ import annotations

import secrets
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Optional

from ..models import AlgorithmConfig, Building, MissionConfig, Waypoint


@dataclass
class MissionVersion:
    """A snapshot of a generated mission."""

    version_id: str
    timestamp: str  # ISO 8601
    building_params: dict
    mission_params: dict
    building: Building
    waypoints: list[Waypoint]
    config: MissionConfig
    algo: AlgorithmConfig
    summary: dict
    viewer_data: dict  # threejs + leaflet data


def _make_version_id() -> str:
    now = datetime.now(timezone.utc)
    suffix = secrets.token_hex(2)
    return f"v_{now.strftime('%Y%m%d_%H%M%S')}_{suffix}"


class SessionState:
    """In-memory store for mission versions. Cleared on server restart."""

    def __init__(self) -> None:
        self._versions: dict[str, MissionVersion] = {}
        self._order: list[str] = []  # most recent first

    def store(
        self,
        building_params: dict,
        mission_params: dict,
        building: Building,
        waypoints: list[Waypoint],
        config: MissionConfig,
        summary: dict,
        viewer_data: dict,
        algo: AlgorithmConfig | None = None,
    ) -> MissionVersion:
        version_id = _make_version_id()
        version = MissionVersion(
            version_id=version_id,
            timestamp=datetime.now(timezone.utc).isoformat(),
            building_params=building_params,
            mission_params=mission_params,
            building=building,
            waypoints=waypoints,
            config=config,
            algo=algo or AlgorithmConfig(),
            summary=summary,
            viewer_data=viewer_data,
        )
        self._versions[version_id] = version
        self._order.insert(0, version_id)
        return version

    def get(self, version_id: str) -> Optional[MissionVersion]:
        return self._versions.get(version_id)

    def list_versions(self) -> list[dict]:
        result = []
        for vid in self._order:
            v = self._versions[vid]
            result.append({
                "version_id": v.version_id,
                "timestamp": v.timestamp,
                "mission_name": v.mission_params.get("mission_name", ""),
                "waypoint_count": v.summary.get("waypoint_count", 0),
                "config_snapshot": {
                    "building": v.building_params,
                    "mission": v.mission_params,
                },
            })
        return result

    def delete(self, version_id: str) -> bool:
        if version_id in self._versions:
            del self._versions[version_id]
            self._order.remove(version_id)
            return True
        return False

    def clear(self) -> int:
        """Delete all versions. Returns the count deleted."""
        count = len(self._versions)
        self._versions.clear()
        self._order.clear()
        return count


# Module-level singleton
session = SessionState()
