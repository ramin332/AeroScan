"""Entry point: python -m flight_planner"""

import uvicorn


def main():
    uvicorn.run(
        "flight_planner.server:app",
        host="0.0.0.0",
        port=8111,
        reload=True,
        # Restrict the watcher to the source tree. The default scans the whole
        # working directory, which means every write to aeroscan.db,
        # output/kmz_cache/, rendered photos, or node_modules kicks off a
        # full module re-import (costs ~2 s for open3d/trimesh).
        reload_dirs=["src"],
        reload_excludes=[
            "sim_output/*",
            "output/*",
            "*.ply",
            "*.pnts",
            "*.png",
            "*.db",
            "__pycache__",
            "frontend/*",
            "kmz/*",
            "tests/fixtures/*",
        ],
    )


if __name__ == "__main__":
    main()
