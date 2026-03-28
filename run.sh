#!/bin/bash
# AeroScan Flight Planner — start backend + frontend dev servers

BACKEND_PORT=8111
FRONTEND_PORT=3847

cd "$(dirname "$0")"

# Activate venv
source .venv/bin/activate

# Start backend
echo "Starting backend on :$BACKEND_PORT ..."
python -m flight_planner &
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
