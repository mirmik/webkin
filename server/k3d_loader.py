"""
K3D file loader - loads kinematic tree from .k3d files (zip archives with k3d.json and STL models)
"""

import json
import zipfile
import tempfile
import shutil
from pathlib import Path
from typing import Optional, Union


class K3DLoader:
    """Loader for .k3d files (zip archives containing k3d.json and STL models)"""

    def __init__(self):
        self.models_dir: Optional[Path] = None
        self.tree_data: Optional[dict] = None
        self.scale_dict: dict = {}
        self.camera_pose: Optional[dict] = None
        self._temp_dir: Optional[Path] = None

    def load_file(self, k3d_path: Union[str, Path]) -> dict:
        """
        Load a .k3d file and extract its contents.
        Returns the kinematic tree in webkin format.

        Path can be absolute or relative, ~ is expanded.
        """
        k3d_path = Path(k3d_path).expanduser().resolve()

        if not k3d_path.exists():
            raise FileNotFoundError(f"K3D file not found: {k3d_path}")

        # Clean up previous temp dir if exists
        self.cleanup()

        # Create temp directory for extracted files
        self._temp_dir = Path(tempfile.mkdtemp(prefix="webkin_k3d_"))
        self.models_dir = self._temp_dir / "models"
        self.models_dir.mkdir()

        # Extract zip contents
        with zipfile.ZipFile(k3d_path, 'r') as zf:
            for member in zf.namelist():
                filename = Path(member).name
                if filename == 'k3d.json':
                    # Extract and parse k3d.json
                    with zf.open(member) as f:
                        raw_data = json.load(f)
                        self._parse_k3d_json(raw_data)
                elif filename.lower().endswith('.stl'):
                    # Extract STL file to models directory
                    with zf.open(member) as src:
                        dst_path = self.models_dir / filename
                        with open(dst_path, 'wb') as dst:
                            dst.write(src.read())
                        print(f"  Extracted: {filename}")

        if self.tree_data is None:
            raise ValueError("k3d.json not found in archive")

        return self.tree_data

    def load_directory(self, dir_path: Union[str, Path]) -> dict:
        """
        Load from an already extracted directory containing k3d.json and STL files.

        Path can be absolute or relative, ~ is expanded.
        """
        dir_path = Path(dir_path).expanduser().resolve()

        if not dir_path.exists():
            raise FileNotFoundError(f"Directory not found: {dir_path}")

        k3d_json_path = dir_path / "k3d.json"
        if not k3d_json_path.exists():
            raise FileNotFoundError(f"k3d.json not found in {dir_path}")

        # Clean up previous temp dir
        self.cleanup()

        # Use the directory directly for models
        self.models_dir = dir_path
        self._temp_dir = None  # No temp dir to clean up

        # Parse k3d.json
        with open(k3d_json_path, 'r') as f:
            raw_data = json.load(f)
            self._parse_k3d_json(raw_data)

        return self.tree_data

    def _parse_k3d_json(self, raw_data: dict):
        """Parse k3d.json and convert to webkin format"""
        # Extract scale dict and camera pose
        self.scale_dict = raw_data.get("scaleDict", {})
        self.camera_pose = raw_data.get("cameraPose", None)

        # Get the kinematic tree (it's nested under "k3d" key)
        k3d_tree = raw_data.get("k3d", raw_data)

        # Convert to webkin format
        self.tree_data = self._convert_node(k3d_tree)

    def _convert_node(self, node: dict) -> dict:
        """Convert a k3d node to webkin format"""
        result = {
            "name": node.get("name", "unnamed"),
            "type": node.get("type", "transform"),
            "children": [],
        }

        # Convert pose (numbers may be strings with comma as decimal separator)
        if "pose" in node:
            pose = node["pose"]
            result["pose"] = {
                "position": self._convert_vec3(pose.get("position", [0, 0, 0])),
                "orientation": self._convert_quat(pose.get("orientation", [0, 0, 0, 1])),
            }

        # Convert axis for rotator/actuator
        if "axis" in node:
            result["axis"] = self._convert_vec3(node["axis"])

        # Convert model
        if "model" in node:
            model = node["model"]
            model_type = model.get("type", "none")

            if model_type == "file":
                stl_path = model.get("path", "")
                scale = self.scale_dict.get(stl_path, 1.0)
                result["model"] = {
                    "type": "stl",
                    "path": f"/k3d/models/{stl_path}",
                    "scale": scale,
                }
            elif model_type == "none":
                result["model"] = {"type": "none"}
            else:
                result["model"] = model

        # Convert children
        for child in node.get("children", []):
            result["children"].append(self._convert_node(child))

        return result

    def _convert_vec3(self, vec: list) -> list:
        """Convert vec3, handling string values with comma decimal separator"""
        return [self._parse_number(v) for v in vec]

    def _convert_quat(self, quat: list) -> list:
        """Convert quaternion, handling string values with comma decimal separator"""
        return [self._parse_number(v) for v in quat]

    def _parse_number(self, value) -> float:
        """Parse a number that may be a string with comma as decimal separator"""
        if isinstance(value, (int, float)):
            return float(value)
        if isinstance(value, str):
            # Replace comma with dot for decimal separator
            return float(value.replace(",", "."))
        return 0.0

    def get_model_path(self, filename: str) -> Optional[Path]:
        """Get the full path to a model file"""
        if self.models_dir is None:
            return None
        path = self.models_dir / filename
        return path if path.exists() else None

    def cleanup(self):
        """Clean up temporary files"""
        if self._temp_dir and self._temp_dir.exists():
            shutil.rmtree(self._temp_dir)
            self._temp_dir = None
            self.models_dir = None

    def __del__(self):
        self.cleanup()
