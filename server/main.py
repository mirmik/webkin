"""
WebKin - Kinematic Tree Visualizer Server

Receives kinematic tree configuration and joint updates from ralgo-cnc-emulator via MQTT.
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
TRANSPORT_TYPE = os.environ.get("TRANSPORT_TYPE", "mqtt")  # "mqtt" or "crow"

# MQTT configuration
MQTT_BROKER = os.environ.get("MQTT_BROKER", "localhost")
MQTT_PORT = int(os.environ.get("MQTT_PORT", "1883"))
MQTT_TOPIC = os.environ.get("MQTT_TOPIC", "robot/joints")
MQTT_TREE_TOPIC = os.environ.get("MQTT_TREE_TOPIC", "robot/joints/tree")

# Paths
BASE_DIR = Path(__file__).parent.parent
STATIC_DIR = BASE_DIR / "static"

# Global state
tree = KinematicTree()
tree_data_json: dict = {}  # Store raw tree data for /api/tree endpoint
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


async def broadcast_scene_init():
    """Send scene_init to all connected clients (after tree reload)"""
    if not clients:
        return

    scene_data = tree.get_scene_data()
    message = json.dumps({
        "type": "scene_init",
        "nodes": scene_data,
        "joints": tree.get_joint_names()
    })

    disconnected = []
    for client in clients:
        try:
            await client.send_text(message)
        except Exception:
            disconnected.append(client)

    for client in disconnected:
        clients.remove(client)


async def mqtt_listener():
    """Listen for MQTT messages: joint updates and tree configuration"""
    global tree_data_json

    try:
        import aiomqtt
    except ImportError:
        print("aiomqtt not installed, MQTT disabled")
        return

    while True:
        try:
            async with aiomqtt.Client(MQTT_BROKER, MQTT_PORT) as client:
                print(f"Connected to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")

                # Subscribe to joint updates
                await client.subscribe(MQTT_TOPIC)
                print(f"Subscribed to: {MQTT_TOPIC}")

                # Subscribe to tree configuration (with retain flag, we get it immediately)
                await client.subscribe(MQTT_TREE_TOPIC)
                print(f"Subscribed to: {MQTT_TREE_TOPIC}")

                async for message in client.messages:
                    topic = str(message.topic)

                    try:
                        payload = json.loads(message.payload.decode())

                        if topic == MQTT_TREE_TOPIC:
                            # Received kinematic tree from ralgo-cnc-emulator
                            print(f"Received kinematic tree: {payload.get('name', 'unnamed')}")
                            tree_data_json = payload
                            tree.load(payload)
                            print(f"Loaded tree with joints: {tree.get_joint_names()}")
                            # Notify all clients about new scene structure
                            await broadcast_scene_init()

                        elif topic == MQTT_TOPIC:
                            # Received joint position update
                            joints = payload.get("joints", {})
                            if joints:
                                tree.set_joint_coords(joints)
                                tree.update()
                                await broadcast_scene()

                    except json.JSONDecodeError:
                        print(f"Invalid JSON on {topic}: {message.payload}")
                    except Exception as e:
                        print(f"Error processing MQTT message on {topic}: {e}")

        except Exception as e:
            print(f"MQTT connection error: {e}, reconnecting in 5s...")
            await asyncio.sleep(5)


async def handle_tree_update(payload: dict):
    """Handle incoming kinematic tree from transport"""
    global tree_data_json
    print(f"Received kinematic tree: {payload.get('name', 'unnamed')}")
    tree_data_json = payload
    tree.load(payload)
    print(f"Loaded tree with joints: {tree.get_joint_names()}")
    await broadcast_scene_init()


async def handle_joints_update(payload: dict):
    """Handle incoming joint positions from transport"""
    joints = payload.get("joints", {})
    if joints:
        tree.set_joint_coords(joints)
        tree.update()
        await broadcast_scene()


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Startup and shutdown events"""
    global mqtt_task, tree_data_json

    # Load fallback tree from file (will be replaced by transport tree if available)
    tree_file = STATIC_DIR / "example_tree.json"
    if tree_file.exists():
        tree_data_json = json.loads(tree_file.read_text())
        tree.load(tree_data_json)
        print(f"Loaded fallback tree with joints: {tree.get_joint_names()}")

    # Start transport listener based on configuration
    if TRANSPORT_TYPE == "crow":
        print("Using Crow transport")
        print("Waiting for kinematic tree from Crow (robot/joints/tree)...")
        from .crow_listener import crow_listener
        mqtt_task = asyncio.create_task(
            crow_listener(
                on_tree=lambda p: asyncio.create_task(handle_tree_update(p)),
                on_joints=lambda p: asyncio.create_task(handle_joints_update(p)),
                joints_topic=MQTT_TOPIC,
                tree_topic=MQTT_TREE_TOPIC,
            )
        )
    else:
        print("Using MQTT transport")
        print("Waiting for kinematic tree from MQTT (robot/joints/tree)...")
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
    """Return the current kinematic tree structure"""
    if tree_data_json:
        return tree_data_json
    return {"error": "No tree loaded"}


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


@app.post("/api/tree")
async def load_tree(tree_json: dict):
    """Load a new kinematic tree via REST API (alternative to MQTT)"""
    global tree_data_json

    tree_data_json = tree_json
    tree.load(tree_json)
    print(f"Loaded tree via REST: {tree_json.get('name', 'unnamed')}")
    print(f"Joints: {tree.get_joint_names()}")

    await broadcast_scene_init()
    return {"status": "ok", "joints": tree.get_joint_names()}


# Mount static files
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
