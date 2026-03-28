"""AeroScan debug & visualization server."""

from pathlib import Path

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

# Path to the built React frontend
_DIST_DIR = (Path(__file__).resolve().parent.parent.parent.parent / "frontend" / "dist")


def create_app() -> FastAPI:
    from .database import init_db
    init_db()

    application = FastAPI(
        title="AeroScan Debug Server",
        version="0.1.0",
        description="Debug & visualization server for the AeroScan NEN-2767 flight planner",
    )
    application.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_methods=["*"],
        allow_headers=["*"],
    )

    from .api import router
    application.include_router(router, prefix="/api")

    # Serve built React frontend (or show hint if not built)
    if _DIST_DIR.exists() and (_DIST_DIR / "index.html").exists():
        assets_dir = _DIST_DIR / "assets"
        if assets_dir.exists():
            application.mount("/assets", StaticFiles(directory=str(assets_dir)), name="assets")

        @application.get("/{path:path}")
        async def _serve_spa(path: str):
            file_path = _DIST_DIR / path
            if file_path.is_file():
                return FileResponse(str(file_path))
            return FileResponse(str(_DIST_DIR / "index.html"))
    else:
        @application.get("/")
        async def _no_frontend():
            return {
                "message": "Frontend not built. Run: cd frontend && npm run build",
                "dev_hint": "For development, run 'npm run dev' in frontend/ and access http://localhost:5173",
            }

    return application


app = create_app()
