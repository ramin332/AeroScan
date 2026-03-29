"""Entry point: python -m flight_planner"""

import uvicorn


def main():
    uvicorn.run(
        "flight_planner.server:app",
        host="0.0.0.0",
        port=8111,
        reload=True,
        reload_excludes=["sim_output/*", "*.ply", "*.png"],
    )


if __name__ == "__main__":
    main()
