#pragma once

/**
 * K3D file loader - loads kinematic tree from .k3d files
 * (zip archives with k3d.json and STL models)
 */

#include <string>
#include <map>
#include <filesystem>
#include <nos/trent/trent.h>

namespace fs = std::filesystem;

namespace webkin
{

class K3DLoader
{
public:
    K3DLoader() = default;
    ~K3DLoader();

    /**
     * Load a .k3d file (zip archive) and extract its contents.
     * Returns the kinematic tree in webkin format.
     */
    nos::trent load_file(const fs::path &k3d_path);

    /**
     * Load from an already extracted directory containing k3d.json and STL files.
     */
    nos::trent load_directory(const fs::path &dir_path);

    /**
     * Get the full path to a model file
     */
    fs::path get_model_path(const std::string &filename) const;

    /**
     * Check if models directory is set
     */
    bool has_models() const { return !_models_dir.empty(); }

    /**
     * Get models directory
     */
    const fs::path &models_dir() const { return _models_dir; }

    /**
     * Get camera pose (if present in k3d.json)
     */
    const nos::trent &camera_pose() const { return _camera_pose; }

    /**
     * Clean up temporary files
     */
    void cleanup();

private:
    fs::path _models_dir;
    fs::path _temp_dir;
    nos::trent _tree_data;
    nos::trent _camera_pose;
    std::map<std::string, double> _scale_dict;

    void parse_k3d_json(const nos::trent &raw_data);
    nos::trent convert_node(const nos::trent &node);
    nos::trent convert_vec3(const nos::trent &vec);
    nos::trent convert_quat(const nos::trent &quat);
    double parse_number(const nos::trent &value);
};

} // namespace webkin
