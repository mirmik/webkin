"""
Crow protocol listener for WebKin.

Receives kinematic tree configuration and joint updates from ralgo-cnc-emulator via Crow pub/sub.
"""

import asyncio
import json
import threading
from typing import Callable


async def crow_listener(
    on_tree: Callable[[dict], None],
    on_joints: Callable[[dict], None],
    joints_topic: str = "robot/joints",
    tree_topic: str = "robot/joints/tree",
):
    """
    Listen for Crow pub/sub messages.

    Args:
        on_tree: Callback when kinematic tree is received
        on_joints: Callback when joint update is received
        joints_topic: Topic for joint updates
        tree_topic: Topic for kinematic tree
    """
    try:
        import pycrow
        from pycrow import pubsub
    except ImportError:
        print("pycrow not installed, Crow support disabled")
        return

    # pycrow uses a synchronous API with background threading
    # We need to bridge it to asyncio

    loop = asyncio.get_event_loop()

    def handle_tree(data: bytes):
        """Handle incoming tree message"""
        try:
            payload = json.loads(data.decode() if isinstance(data, bytes) else data)
            # Schedule callback in asyncio event loop
            loop.call_soon_threadsafe(lambda: asyncio.create_task(_async_on_tree(payload)))
        except json.JSONDecodeError as e:
            print(f"Crow: Invalid JSON on {tree_topic}: {e}")
        except Exception as e:
            print(f"Crow: Error processing tree message: {e}")

    def handle_joints(data: bytes):
        """Handle incoming joints message"""
        try:
            payload = json.loads(data.decode() if isinstance(data, bytes) else data)
            # Schedule callback in asyncio event loop
            loop.call_soon_threadsafe(lambda: asyncio.create_task(_async_on_joints(payload)))
        except json.JSONDecodeError as e:
            print(f"Crow: Invalid JSON on {joints_topic}: {e}")
        except Exception as e:
            print(f"Crow: Error processing joints message: {e}")

    async def _async_on_tree(payload):
        on_tree(payload)

    async def _async_on_joints(payload):
        on_joints(payload)

    # Start pycrow client
    print("Starting Crow client...")
    pycrow.start_client()

    # Subscribe to topics
    print(f"Crow: Subscribing to {tree_topic}")
    pubsub.subscribe(tree_topic, handle_tree)

    print(f"Crow: Subscribing to {joints_topic}")
    pubsub.subscribe(joints_topic, handle_joints)

    print("Crow: Listener started")

    try:
        # Keep running until cancelled
        while True:
            await asyncio.sleep(1)
    except asyncio.CancelledError:
        print("Crow: Listener cancelled")
    finally:
        print("Crow: Stopping client...")
        pycrow.stop_client()
        print("Crow: Client stopped")
