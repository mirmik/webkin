#!/usr/bin/env python3
"""
WebKin launcher script with command-line arguments.

Usage:
    python run.py              # Use MQTT (default)
    python run.py --crow       # Use Crow protocol
    python run.py --host 0.0.0.0 --port 8080
"""

import argparse
import os
import uvicorn


def main():
    parser = argparse.ArgumentParser(
        description="WebKin - Kinematic Tree Visualizer Server"
    )
    parser.add_argument(
        "--crow",
        action="store_true",
        help="Use Crow protocol instead of MQTT",
    )
    parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="Host to bind to (default: 0.0.0.0)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8000,
        help="Port to bind to (default: 8000)",
    )
    parser.add_argument(
        "--reload",
        action="store_true",
        help="Enable auto-reload for development",
    )

    args = parser.parse_args()

    # Set transport type based on --crow flag
    if args.crow:
        os.environ["TRANSPORT_TYPE"] = "crow"

    uvicorn.run(
        "server.main:app",
        host=args.host,
        port=args.port,
        reload=args.reload,
    )


if __name__ == "__main__":
    main()
