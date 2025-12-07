#!/usr/bin/env python3
"""
Demo script: rotates joints via MQTT
"""

import json
import math
import time
import argparse

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("Install paho-mqtt: pip install paho-mqtt")
    exit(1)


def main():
    parser = argparse.ArgumentParser(description="Rotate robot joints demo")
    parser.add_argument("--broker", default="localhost", help="MQTT broker host")
    parser.add_argument("--port", type=int, default=1883, help="MQTT broker port")
    parser.add_argument("--topic", default="robot/joints", help="MQTT topic")
    parser.add_argument("--speed", type=float, default=1.0, help="Rotation speed multiplier")
    args = parser.parse_args()

    client = mqtt.Client()

    try:
        client.connect(args.broker, args.port, 60)
        client.loop_start()
        print(f"Connected to {args.broker}:{args.port}")
        print(f"Publishing to topic: {args.topic}")
        print("Press Ctrl+C to stop\n")

        t = 0
        while True:
            # Animate joints with different frequencies
            joints = {
                "shoulder_yaw": math.sin(t * 0.5) * 1.5,           # slow base rotation
                "shoulder_pitch": math.sin(t * 0.7) * 0.8,         # shoulder up/down
                "elbow": math.sin(t * 1.0 + 1) * 1.2,              # elbow bend
                "wrist": math.sin(t * 2.0) * 2.0,                  # fast wrist rotation
            }

            message = json.dumps({"joints": joints})
            client.publish(args.topic, message)

            # Print current values
            values = " | ".join(f"{k}: {math.degrees(v):+6.1f}°" for k, v in joints.items())
            print(f"\r{values}", end="", flush=True)

            t += 0.05 * args.speed
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\nStopped")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
