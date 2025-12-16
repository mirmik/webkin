/**
 * K3D file loader implementation
 */

#include "k3d_loader.hpp"
#include <nos/trent/json.h>
#include <nos/print.h>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cstring>

// Simple ZIP reading (central directory at end of file)
// For production, consider using libzip or miniz
#include <vector>
#include <cstdint>

namespace webkin
{

namespace
{
    // ZIP local file header signature
    constexpr uint32_t ZIP_LOCAL_HEADER_SIG = 0x04034b50;
    constexpr uint32_t ZIP_CENTRAL_DIR_SIG = 0x02014b50;
    constexpr uint32_t ZIP_END_CENTRAL_SIG = 0x06054b50;

    struct ZipLocalHeader
    {
        uint32_t signature;
        uint16_t version;
        uint16_t flags;
        uint16_t compression;
        uint16_t mod_time;
        uint16_t mod_date;
        uint32_t crc32;
        uint32_t compressed_size;
        uint32_t uncompressed_size;
        uint16_t filename_len;
        uint16_t extra_len;
    } __attribute__((packed));

    bool extract_zip_file(const fs::path &zip_path, const fs::path &dest_dir,
                          std::string &k3d_json_content)
    {
        std::ifstream file(zip_path, std::ios::binary);
        if (!file)
            return false;

        while (file)
        {
            ZipLocalHeader header;
            file.read(reinterpret_cast<char *>(&header), sizeof(header));

            if (!file || header.signature != ZIP_LOCAL_HEADER_SIG)
                break;

            // Read filename
            std::string filename(header.filename_len, '\0');
            file.read(filename.data(), header.filename_len);

            // Skip extra field
            file.seekg(header.extra_len, std::ios::cur);

            // Read file data (assuming no compression for now, compression=0)
            std::vector<char> data(header.uncompressed_size);
            if (header.compression == 0)
            {
                file.read(data.data(), header.uncompressed_size);
            }
            else
            {
                // Skip compressed files we can't handle
                file.seekg(header.compressed_size, std::ios::cur);
                continue;
            }

            // Get just the filename without path
            std::string basename = fs::path(filename).filename().string();

            if (basename == "k3d.json")
            {
                k3d_json_content = std::string(data.begin(), data.end());
            }
            else if (basename.size() > 4 &&
                     (basename.substr(basename.size() - 4) == ".stl" ||
                      basename.substr(basename.size() - 4) == ".STL"))
            {
                fs::path out_path = dest_dir / basename;
                std::ofstream out(out_path, std::ios::binary);
                out.write(data.data(), data.size());
                nos::println("  Extracted: ", basename);
            }
        }

        return !k3d_json_content.empty();
    }

    std::string generate_temp_dir()
    {
        const char *tmpdir = std::getenv("TMPDIR");
        if (!tmpdir)
            tmpdir = "/tmp";

        std::string path = std::string(tmpdir) + "/webkin_k3d_XXXXXX";
        std::vector<char> buf(path.begin(), path.end());
        buf.push_back('\0');

        if (mkdtemp(buf.data()) == nullptr)
        {
            return "";
        }
        return std::string(buf.data());
    }
}

K3DLoader::~K3DLoader()
{
    cleanup();
}

nos::trent K3DLoader::load_file(const fs::path &k3d_path)
{
    fs::path resolved = fs::weakly_canonical(k3d_path);

    if (!fs::exists(resolved))
    {
        throw std::runtime_error("K3D file not found: " + resolved.string());
    }

    // Clean up previous temp dir
    cleanup();

    // Create temp directory
    std::string temp_str = generate_temp_dir();
    if (temp_str.empty())
    {
        throw std::runtime_error("Failed to create temp directory");
    }
    _temp_dir = temp_str;
    _models_dir = _temp_dir / "models";
    fs::create_directories(_models_dir);

    // Extract zip
    std::string k3d_json_content;
    if (!extract_zip_file(resolved, _models_dir, k3d_json_content))
    {
        throw std::runtime_error("k3d.json not found in archive");
    }

    // Parse JSON
    nos::trent raw_data = nos::json::parse(k3d_json_content);
    parse_k3d_json(raw_data);

    return _tree_data;
}

nos::trent K3DLoader::load_directory(const fs::path &dir_path)
{
    fs::path resolved = fs::weakly_canonical(dir_path);

    if (!fs::exists(resolved))
    {
        throw std::runtime_error("Directory not found: " + resolved.string());
    }

    fs::path k3d_json_path = resolved / "k3d.json";
    if (!fs::exists(k3d_json_path))
    {
        throw std::runtime_error("k3d.json not found in " + resolved.string());
    }

    // Clean up previous temp dir
    cleanup();

    // Use directory directly
    _models_dir = resolved;
    _temp_dir.clear();

    // Read and parse k3d.json
    std::ifstream file(k3d_json_path);
    std::stringstream buffer;
    buffer << file.rdbuf();

    nos::trent raw_data = nos::json::parse(buffer.str());
    parse_k3d_json(raw_data);

    return _tree_data;
}

void K3DLoader::parse_k3d_json(const nos::trent &raw_data)
{
    // Extract scale dict
    _scale_dict.clear();
    const auto &scale_dict = raw_data["scaleDict"];
    if (scale_dict.is_dict())
    {
        for (const auto &[key, value] : scale_dict.as_dict())
        {
            _scale_dict[key] = value.as_numer_default(1.0);
        }
    }

    // Extract camera pose
    _camera_pose = raw_data["cameraPose"];

    // Get kinematic tree (may be nested under "k3d" key)
    const auto &k3d_tree = raw_data["k3d"].is_nil() ? raw_data : raw_data["k3d"];

    // Convert to webkin format
    _tree_data = convert_node(k3d_tree);
}

nos::trent K3DLoader::convert_node(const nos::trent &node)
{
    nos::trent result;
    result.init(nos::trent::type::dict);

    result["name"] = node["name"].as_string_default("unnamed");
    result["type"] = node["type"].as_string_default("transform");

    // Convert pose
    const auto &pose = node["pose"];
    if (!pose.is_nil())
    {
        nos::trent pose_out;
        pose_out.init(nos::trent::type::dict);
        pose_out["position"] = convert_vec3(pose["position"]);
        pose_out["orientation"] = convert_quat(pose["orientation"]);
        result["pose"] = std::move(pose_out);
    }

    // Convert axis
    const auto &axis = node["axis"];
    if (!axis.is_nil())
    {
        result["axis"] = convert_vec3(axis);
    }

    // Convert model
    const auto &model = node["model"];
    if (!model.is_nil())
    {
        std::string model_type = model["type"].as_string_default("none");

        nos::trent model_out;
        model_out.init(nos::trent::type::dict);

        if (model_type == "file")
        {
            std::string stl_path = model["path"].as_string_default("");
            double scale = 1.0;
            auto it = _scale_dict.find(stl_path);
            if (it != _scale_dict.end())
            {
                scale = it->second;
            }

            model_out["type"] = "stl";
            model_out["path"] = "/k3d/models/" + stl_path;
            model_out["scale"] = scale;
        }
        else if (model_type == "none")
        {
            model_out["type"] = "none";
        }
        else
        {
            // Pass through other model types
            model_out = model;
        }

        result["model"] = std::move(model_out);
    }

    // Convert children
    nos::trent children;
    children.init(nos::trent::type::list);

    const auto &node_children = node["children"];
    if (node_children.is_list())
    {
        for (const auto &child : node_children.as_list())
        {
            children.push_back(convert_node(child));
        }
    }
    result["children"] = std::move(children);

    return result;
}

nos::trent K3DLoader::convert_vec3(const nos::trent &vec)
{
    nos::trent result;
    result.init(nos::trent::type::list);

    if (vec.is_list() && vec.as_list().size() >= 3)
    {
        result.push_back(parse_number(vec[0]));
        result.push_back(parse_number(vec[1]));
        result.push_back(parse_number(vec[2]));
    }
    else
    {
        result.push_back(0.0);
        result.push_back(0.0);
        result.push_back(0.0);
    }

    return result;
}

nos::trent K3DLoader::convert_quat(const nos::trent &quat)
{
    nos::trent result;
    result.init(nos::trent::type::list);

    if (quat.is_list() && quat.as_list().size() >= 4)
    {
        result.push_back(parse_number(quat[0]));
        result.push_back(parse_number(quat[1]));
        result.push_back(parse_number(quat[2]));
        result.push_back(parse_number(quat[3]));
    }
    else
    {
        result.push_back(0.0);
        result.push_back(0.0);
        result.push_back(0.0);
        result.push_back(1.0);
    }

    return result;
}

double K3DLoader::parse_number(const nos::trent &value)
{
    if (value.is_numer())
    {
        return value.as_numer();
    }
    if (value.is_string())
    {
        // Handle comma as decimal separator
        std::string s = value.as_string();
        for (char &c : s)
        {
            if (c == ',')
                c = '.';
        }
        return std::stod(s);
    }
    return 0.0;
}

fs::path K3DLoader::get_model_path(const std::string &filename) const
{
    if (_models_dir.empty())
        return {};

    fs::path path = _models_dir / filename;
    return fs::exists(path) ? path : fs::path{};
}

void K3DLoader::cleanup()
{
    if (!_temp_dir.empty() && fs::exists(_temp_dir))
    {
        fs::remove_all(_temp_dir);
        _temp_dir.clear();
        _models_dir.clear();
    }
}

} // namespace webkin
