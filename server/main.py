"""
WebKin - Kinematic Tree Visualizer Server
"""

import asyncio
import json
import os
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse

from .kinematic import KinematicTree


# Configuration
MQTT_BROKER = os.environ.get("MQTT_BROKER", "localhost")
MQTT_PORT = int(os.environ.get("MQTT_PORT", "1883"))
MQTT_TOPIC = os.environ.get("MQTT_TOPIC", "robot/joints")

# Paths
BASE_DIR = Path(__file__).parent.parent
STATIC_DIR = BASE_DIR / "static"

# Global state
tree = KinematicTree()
clients: list[WebSocket] = []
mqtt_task = None


async def broadcast_scene():
    """Send current scene state to all connected clients"""
    if not clients:
        return

    scene_data = tree.get_scene_data()
    message = json.dumps({"type": "scene_update", "nodes": scene_data})

    disconnected = []
    for client in clients:
        try:
            await client.send_text(message)
        except Exception:
            disconnected.append(client)

    for client in disconnected:
        clients.remove(client)


async def mqtt_listener():
    """Listen for MQTT messages and update joint positions"""
    try:
        import aiomqtt
    except ImportError:
        print("aiomqtt not installed, MQTT disabled")
        return

    while True:
        try:
            async with aiomqtt.Client(MQTT_BROKER, MQTT_PORT) as client:
                print(f"Connected to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
                await client.subscribe(MQTT_TOPIC)

                async for message in client.messages:
                    try:
                        payload = json.loads(message.payload.decode())
                        joints = payload.get("joints", {})

                        if joints:
                            tree.set_joint_coords(joints)
                            tree.update()
                            await broadcast_scene()

                    except json.JSONDecodeError:
                        print(f"Invalid JSON: {message.payload}")
                    except Exception as e:
                        print(f"Error processing MQTT message: {e}")

        except Exception as e:
            print(f"MQTT connection error: {e}, reconnecting in 5s...")
            await asyncio.sleep(5)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Startup and shutdown events"""
    global mqtt_task

    # Load tree on startup
    tree_file = STATIC_DIR / "example_tree.json"
    if tree_file.exists():
        tree.load(json.loads(tree_file.read_text()))
        print(f"Loaded tree with joints: {tree.get_joint_names()}")

    # Start MQTT listener
    mqtt_task = asyncio.create_task(mqtt_listener())

    yield

    # Cleanup
    if mqtt_task:
        mqtt_task.cancel()
        try:
            await mqtt_task
        except asyncio.CancelledError:
            pass


app = FastAPI(title="WebKin", description="Kinematic Tree Visualizer", lifespan=lifespan)


@app.get("/")
async def root():
    """Serve the main page"""
    return FileResponse(STATIC_DIR / "index.html")


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket endpoint for real-time scene updates"""
    await websocket.accept()
    clients.append(websocket)
    print(f"Client connected. Total: {len(clients)}")

    # Send initial scene state
    scene_data = tree.get_scene_data()
    await websocket.send_text(json.dumps({
        "type": "scene_init",
        "nodes": scene_data,
        "joints": tree.get_joint_names()
    }))

    try:
        while True:
            data = await websocket.receive_text()
            message = json.loads(data)

            # Handle joint updates from UI sliders
            if message.get("type") == "joint_update":
                joints = message.get("joints", {})
                tree.set_joint_coords(joints)
                tree.update()
                await broadcast_scene()

    except WebSocketDisconnect:
        clients.remove(websocket)
        print(f"Client disconnected. Total: {len(clients)}")


@app.get("/api/tree")
async def get_tree():
    """Return the kinematic tree structure"""
    tree_file = STATIC_DIR / "example_tree.json"
    if tree_file.exists():
        return json.loads(tree_file.read_text())
    return {"error": "No tree file found"}


@app.get("/api/scene")
async def get_scene():
    """Return current scene state (all computed poses)"""
    return tree.get_scene_data()


@app.post("/api/joints")
async def set_joints(joints: dict[str, float]):
    """Set joint coordinates via REST API"""
    tree.set_joint_coords(joints)
    tree.update()
    await broadcast_scene()
    return {"status": "ok"}


# Mount static files
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
