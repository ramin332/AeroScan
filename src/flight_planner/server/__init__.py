"""AeroScan debug & visualization server."""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware


def create_app() -> FastAPI:
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
    from .frontend import frontend_router

    application.include_router(router, prefix="/api")
    application.include_router(frontend_router)

    return application


app = create_app()
