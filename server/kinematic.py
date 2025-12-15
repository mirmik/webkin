"""
Kinematic Tree - Server-side calculations
"""

import math
import json
from dataclasses import dataclass, field
from typing import Optional, List, Dict


@dataclass
class Vec3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __add__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __mul__(self, scalar: float) -> "Vec3":
        return Vec3(self.x * scalar, self.y * scalar, self.z * scalar)

    def to_list(self) -> list:
        return [self.x, self.y, self.z]

    @staticmethod
    def from_list(lst: list) -> "Vec3":
        return Vec3(lst[0], lst[1], lst[2]) if lst else Vec3()


@dataclass
class Quat:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0

    def __mul__(self, other: "Quat") -> "Quat":
        return Quat(
            self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
            self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
        )

    def rotate_vec(self, v: Vec3) -> Vec3:
        """Rotate vector by quaternion"""
        qv = Quat(v.x, v.y, v.z, 0)
        conj = Quat(-self.x, -self.y, -self.z, self.w)
        result = self * qv * conj
        return Vec3(result.x, result.y, result.z)

    def to_list(self) -> list:
        return [self.x, self.y, self.z, self.w]

    @staticmethod
    def from_list(lst: list) -> "Quat":
        return Quat(lst[0], lst[1], lst[2], lst[3]) if lst else Quat()

    @staticmethod
    def from_axis_angle(axis: Vec3, angle: float) -> "Quat":
        """Create quaternion from axis and angle (radians)"""
        half = angle / 2
        s = math.sin(half)
        return Quat(axis.x * s, axis.y * s, axis.z * s, math.cos(half))


@dataclass
class Pose:
    position: Vec3 = field(default_factory=Vec3)
    orientation: Quat = field(default_factory=Quat)

    def __mul__(self, other: "Pose") -> "Pose":
        """Compose two poses: self * other"""
        return Pose(
            position=self.position + self.orientation.rotate_vec(other.position),
            orientation=self.orientation * other.orientation,
        )

    def to_dict(self) -> dict:
        return {
            "position": self.position.to_list(),
            "orientation": self.orientation.to_list(),
        }


class KinematicNode:
    def __init__(self, data: dict, parent: Optional["KinematicNode"] = None):
        self.name = data.get("name", "unnamed")
        self.type = data.get("type", "transform")
        self.parent = parent
        self.children: List["KinematicNode"] = []

        # Local pose (from JSON)
        pose_data = data.get("pose", {})
        self.local_pose = Pose(
            position=Vec3.from_list(pose_data.get("position", [0, 0, 0])),
            orientation=Quat.from_list(pose_data.get("orientation", [0, 0, 0, 1])),
        )

        # Axis for rotator/actuator
        axis_data = data.get("axis", [0, 0, 1])
        self.axis = Vec3.from_list(axis_data)

        # Current joint coordinate
        self.coord = 0.0

        # Model data (pass through to client)
        self.model = data.get("model", {})

        # Computed global pose
        self.global_pose = Pose()

        # Parse children
        for child_data in data.get("children", []):
            child = KinematicNode(child_data, self)
            self.children.append(child)

    def set_coord(self, value: float):
        """Set joint coordinate (radians for rotator, units for actuator)"""
        self.coord = value

    def get_joint_transform(self) -> Pose:
        """Get transform introduced by joint movement"""
        if self.type == "rotator":
            q = Quat.from_axis_angle(self.axis, self.coord)
            return Pose(orientation=q)
        elif self.type == "actuator":
            offset = self.axis * self.coord
            return Pose(position=offset)
        return Pose()

    def compute_global_poses(self, parent_pose: Pose = None):
        """Recursively compute global poses for all nodes"""
        if parent_pose is None:
            parent_pose = Pose()

        # Global = parent * local * joint
        # Joint transform is included so the node's visual moves/rotates with the joint
        self.global_pose = parent_pose * self.local_pose * self.get_joint_transform()

        for child in self.children:
            child.compute_global_poses(self.global_pose)

    def get_all_poses(self) -> dict:
        """Get dictionary of all node poses {name: pose}"""
        result = {self.name: self.global_pose.to_dict()}
        for child in self.children:
            result.update(child.get_all_poses())
        return result

    def get_scene_data(self) -> dict:
        """Get scene data for client (poses + model info)"""
        result = {
            self.name: {
                "pose": self.global_pose.to_dict(),
                "model": self.model,
            }
        }
        for child in self.children:
            result.update(child.get_scene_data())
        return result

    def find_by_name(self, name: str) -> Optional["KinematicNode"]:
        if self.name == name:
            return self
        for child in self.children:
            found = child.find_by_name(name)
            if found:
                return found
        return None

    def get_all_joints(self) -> List["KinematicNode"]:
        """Get all joint nodes (rotator/actuator)"""
        joints = []
        if self.type in ("rotator", "actuator"):
            joints.append(self)
        for child in self.children:
            joints.extend(child.get_all_joints())
        return joints


class KinematicTree:
    def __init__(self):
        self.root: Optional[KinematicNode] = None
        self.joints: Dict[str, KinematicNode] = {}

    def load(self, data: dict):
        """Load tree from JSON data"""
        self.root = KinematicNode(data)
        self.joints = {j.name: j for j in self.root.get_all_joints()}
        self.update()

    def set_joint_coords(self, coords: Dict[str, float]):
        """Set multiple joint coordinates"""
        for name, value in coords.items():
            if name in self.joints:
                self.joints[name].set_coord(value)

    def update(self):
        """Recompute all global poses"""
        if self.root:
            self.root.compute_global_poses()

    def get_scene_data(self) -> dict:
        """Get all scene data for rendering"""
        if self.root:
            return self.root.get_scene_data()
        return {}

    def get_joint_names(self) -> List[str]:
        """Get list of joint names"""
        return list(self.joints.keys())
