#!/bin/bash
# AeroScan Flight Planner — start backend + frontend dev servers

BACKEND_PORT=8111
FRONTEND_PORT=3847

cd "$(dirname "$0")"

# Recreate venv if broken (e.g. after repo rename/move)
if [ ! -x ".venv/bin/python" ] || ! .venv/bin/python -c "import sys" &>/dev/null; then
    echo "Venv missing or broken — recreating..."
    rm -rf .venv
    python3.12 -m venv .venv
    .venv/bin/pip install -e ".[dev,server]" -q
fi

# Kill anything already on these ports
lsof -ti :$BACKEND_PORT | xargs kill -9 2>/dev/null
lsof -ti :$FRONTEND_PORT | xargs kill -9 2>/dev/null

# Start backend
echo "Starting backend on :$BACKEND_PORT ..."
.venv/bin/python -m flight_planner &
BACKEND_PID=$!

# Start frontend dev server
echo "Starting frontend on :$FRONTEND_PORT ..."
cd frontend
VITE_PORT=$FRONTEND_PORT npx vite --port $FRONTEND_PORT &
FRONTEND_PID=$!
cd ..

echo ""
echo "  Frontend:  http://localhost:$FRONTEND_PORT"
echo "  Backend:   http://localhost:$BACKEND_PORT"
echo "  API docs:  http://localhost:$BACKEND_PORT/docs"
echo ""
echo "  Press Ctrl+C to stop both."
echo ""

# Cleanup on exit
trap "kill $BACKEND_PID $FRONTEND_PID 2>/dev/null; exit" INT TERM
wait
