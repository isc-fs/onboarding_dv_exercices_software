#!/usr/bin/env bash
# Bring up Mission Control for local development:
#   - Backend (FastAPI / uvicorn) on :8000
#   - Frontend (Vite dev server) on :3000
#
# Both run in the foreground; Ctrl-C stops both via the trap. For a
# production-mode build, run `npm run build` in frontend/ and serve
# the dist/ directory via any static file server (nginx, caddy,
# whatever); the backend stays the same.

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BACKEND_DIR="$SCRIPT_DIR/backend"
FRONTEND_DIR="$SCRIPT_DIR/frontend"
VENV_DIR="${IFSSIM_MC_VENV_DIR:-$SCRIPT_DIR/.venv}"

BACKEND_PID=""
FRONTEND_PID=""
VENV_ACTIVATED=0
BACKEND_PYTHON=""

find_python() {
    if command -v python3 >/dev/null 2>&1; then
        command -v python3
        return
    fi
    if command -v python >/dev/null 2>&1; then
        command -v python
        return
    fi
    echo "error: neither python3 nor python was found on PATH" >&2
    exit 1
}

venv_python() {
    if [ -x "$VENV_DIR/bin/python" ]; then
        echo "$VENV_DIR/bin/python"
        return
    fi
    if [ -x "$VENV_DIR/Scripts/python.exe" ]; then
        echo "$VENV_DIR/Scripts/python.exe"
        return
    fi
    if [ -x "$VENV_DIR/Scripts/python" ]; then
        echo "$VENV_DIR/Scripts/python"
        return
    fi
    echo "error: could not find Python inside venv at $VENV_DIR" >&2
    exit 1
}

activate_venv() {
    if [ -f "$VENV_DIR/bin/activate" ]; then
        # shellcheck disable=SC1091
        . "$VENV_DIR/bin/activate"
    elif [ -f "$VENV_DIR/Scripts/activate" ]; then
        # Git Bash / Windows Python venv layout.
        # shellcheck disable=SC1091
        . "$VENV_DIR/Scripts/activate"
    else
        echo "error: could not find activate script inside venv at $VENV_DIR" >&2
        exit 1
    fi
    VENV_ACTIVATED=1
}

cleanup() {
    exit_code=$?
    set +e
    trap - EXIT INT TERM

    if [ -n "$BACKEND_PID" ] || [ -n "$FRONTEND_PID" ]; then
        echo
        echo "→ stopping Mission Control..."
        if [ -n "$BACKEND_PID" ]; then kill "$BACKEND_PID" 2>/dev/null; fi
        if [ -n "$FRONTEND_PID" ]; then kill "$FRONTEND_PID" 2>/dev/null; fi
        wait 2>/dev/null
    fi

    if [ "$VENV_ACTIVATED" -eq 1 ] && command -v deactivate >/dev/null 2>&1; then
        deactivate
        VENV_ACTIVATED=0
    fi

    exit "$exit_code"
}
trap cleanup EXIT INT TERM

# Create/use an isolated backend venv so local runs never install into
# system Python. Operators who manage their own environment can skip the
# venv + install step by exporting IFSSIM_MC_SKIP_INSTALL=1.
setup_backend_python() {
    if [ -n "${IFSSIM_MC_SKIP_INSTALL:-}" ]; then
        BACKEND_PYTHON="$(find_python)"
        return
    fi

    if [ ! -d "$VENV_DIR" ]; then
        echo "→ creating backend venv at $VENV_DIR"
        "$(find_python)" -m venv "$VENV_DIR"
    fi

    activate_venv
    BACKEND_PYTHON="$(venv_python)"

    echo "→ installing backend deps into venv"
    "$BACKEND_PYTHON" -m pip install -r "$BACKEND_DIR/requirements.txt"
}

maybe_install_frontend() {
    if [ -n "${IFSSIM_MC_SKIP_INSTALL:-}" ]; then return; fi
    if [ -d "$FRONTEND_DIR/node_modules" ]; then return; fi
    echo "→ installing frontend deps (one-time, ~1 m)"
    (cd "$FRONTEND_DIR" && npm install)
}

setup_backend_python
maybe_install_frontend

# Start both, hold their PIDs for cleanup.
echo "→ starting backend on :8000"
(cd "$BACKEND_DIR" && "$BACKEND_PYTHON" main.py) &
BACKEND_PID=$!

echo "→ starting frontend on :3000"
(cd "$FRONTEND_DIR" && npm run dev -- --host 0.0.0.0) &
FRONTEND_PID=$!

echo
echo "✓ Mission Control running"
echo "  Backend:  http://localhost:8000"
echo "  Frontend: http://localhost:3000"
echo "  Ctrl-C to stop both."
echo

# Wait for either process to exit; if one dies, take the other down.
wait -n "$BACKEND_PID" "$FRONTEND_PID"
