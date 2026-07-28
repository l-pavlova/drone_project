"""Entry point: `python -m parkdrone_vision.server` (or uvicorn parkdrone_vision.app:app).

Runs the FastAPI monolith on API_PORT (default 4000, matching the web-user vite
proxy). uvicorn's default ping keeps WebSocket clients alive; no manual
heartbeat needed.
"""
import uvicorn

from .config import API_PORT


def main() -> None:
    uvicorn.run(
        "parkdrone_vision.app:app",
        host="0.0.0.0",
        port=API_PORT,
        log_level="info",
    )


if __name__ == "__main__":
    main()
