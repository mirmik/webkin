/**
 * WebKin - Kinematic Tree Visualizer Server (C++ version)
 *
 * HTTP server with WebSocket support for real-time kinematic tree visualization.
 * Supports MQTT and Crow protocol transports for receiving robot data.
 */

#include "kinematic.hpp"
#include "mqtt_listener.hpp"
#include "crow_listener.hpp"
#include "k3d_loader.hpp"

#include <crowhttp.h>

#include <nos/trent/json.h>
#include <nos/trent/json_print.h>
#include <nos/print.h>

#include <mutex>
#include <set>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <csignal>
#include <atomic>
#include <memory>
#include <cstdlib>
#include <vector>

namespace fs = std::filesystem;

// IRCC embedded resources
extern std::string ircc_string(const std::string &key);
extern std::vector<std::string> ircc_keys();

// Global state
webkin::KinematicTree g_tree;
nos::trent g_tree_data_json;
std::mutex g_mutex;
std::set<crowhttp::websocket::connection *> g_clients;
bool g_z_up = false;
bool g_debug = false;
std::atomic<bool> g_running{true};
bool g_use_embedded_resources = true;  // Use embedded resources by default

// Paths
fs::path g_base_dir;
fs::path g_static_dir;
fs::path g_config_dir;
fs::path g_axis_overrides_file;

// K3D loader
std::unique_ptr<webkin::K3DLoader> g_k3d_loader;

// Axis overrides: {axis_name: {axis_offset, axis_scale, slider_min, slider_max}}
std::map<std::string, std::map<std::string, double>> g_axis_overrides;

// Transport type
enum class TransportType
{
    NONE,
    MQTT,
    CROW
};

// Forward declarations
std::string read_file(const fs::path &path);

// Read static resource (from embedded or file system)
std::string read_static_resource(const std::string &resource_path)
{
    if (g_use_embedded_resources)
    {
        // Try embedded resource first
        std::string key = "/static/" + resource_path;
        std::string content = ircc_string(key.c_str());
        if (!content.empty())
        {
            return content;
        }
    }
    // Fall back to file system
    return read_file(g_static_dir / resource_path);
}

std::string trent_to_json(const nos::trent &t)
{
    return nos::json::to_string(t);
}

void load_axis_overrides()
{
    g_axis_overrides.clear();
    if (fs::exists(g_axis_overrides_file))
    {
        try
        {
            std::string content = read_file(g_axis_overrides_file);
            nos::trent data = nos::json::parse(content);
            if (data.is_dict())
            {
                for (const auto &[name, params] : data.as_dict())
                {
                    if (params.is_dict())
                    {
                        for (const auto &[key, value] : params.as_dict())
                        {
                            g_axis_overrides[name][key] = value.as_numer_default(0.0);
                        }
                    }
                }
            }
            nos::println("Loaded axis overrides: ", g_axis_overrides.size(), " entries");
        }
        catch (const std::exception &e)
        {
            nos::println("Failed to load axis overrides: ", e.what());
        }
    }
}

void save_axis_overrides()
{
    try
    {
        fs::create_directories(g_config_dir);
        nos::trent data;
        data.init(nos::trent::type::dict);
        for (const auto &[name, params] : g_axis_overrides)
        {
            nos::trent joint_data;
            joint_data.init(nos::trent::type::dict);
            for (const auto &[key, value] : params)
            {
                joint_data[key] = value;
            }
            data[name] = std::move(joint_data);
        }
        std::ofstream file(g_axis_overrides_file);
        file << nos::json::to_string(data);
        nos::println("Saved axis overrides: ", g_axis_overrides.size(), " entries");
    }
    catch (const std::exception &e)
    {
        nos::println("Failed to save axis overrides: ", e.what());
    }
}

void apply_axis_overrides()
{
    for (const auto &[name, params] : g_axis_overrides)
    {
        auto it = g_tree.joints.find(name);
        if (it != g_tree.joints.end())
        {
            auto *joint = it->second;
            auto offset_it = params.find("axis_offset");
            if (offset_it != params.end())
                joint->axis_offset = offset_it->second;

            auto scale_it = params.find("axis_scale");
            if (scale_it != params.end())
                joint->axis_scale = scale_it->second;

            auto min_it = params.find("slider_min");
            if (min_it != params.end())
                joint->slider_min = min_it->second;

            auto max_it = params.find("slider_max");
            if (max_it != params.end())
                joint->slider_max = max_it->second;
        }
    }
}

nos::trent find_original_axis_params(const nos::trent &node, const std::string &joint_name)
{
    if (node["name"].as_string_default("") == joint_name)
    {
        nos::trent result;
        result.init(nos::trent::type::dict);
        std::string jtype = node["type"].as_string_default("transform");
        double default_min = -180.0, default_max = 180.0;
        if (jtype == "actuator")
        {
            default_min = -1000.0;
            default_max = 1000.0;
        }
        result["axis_offset"] = node["axis_offset"].as_numer_default(0.0);
        result["axis_scale"] = node["axis_scale"].as_numer_default(1.0);
        result["slider_min"] = node["slider_min"].as_numer_default(default_min);
        result["slider_max"] = node["slider_max"].as_numer_default(default_max);
        return result;
    }
    const auto &children = node["children"];
    if (children.is_list())
    {
        for (const auto &child : children.as_list())
        {
            nos::trent result = find_original_axis_params(child, joint_name);
            if (!result.is_nil())
            {
                return result;
            }
        }
    }
    nos::trent empty;
    return empty;
}

nos::trent make_scene_init_message()
{
    nos::trent msg;
    msg.init(nos::trent::type::dict);
    msg["type"] = "scene_init";
    msg["nodes"] = g_tree.get_scene_data();
    msg["joints"] = g_tree.get_joint_names_trent();
    msg["jointsInfo"] = g_tree.get_joints_info();
    msg["zUp"] = g_z_up;
    return msg;
}

nos::trent make_scene_update_message()
{
    nos::trent msg;
    msg.init(nos::trent::type::dict);
    msg["type"] = "scene_update";
    msg["nodes"] = g_tree.get_scene_data();
    msg["jointsInfo"] = g_tree.get_joints_info();
    return msg;
}

void broadcast_to_clients(const std::string &message)
{
    for (auto *conn : g_clients)
    {
        conn->send_text(message);
    }
}

void broadcast_scene_update()
{
    if (g_clients.empty())
    {
        if (g_debug)
        {
            nos::println("[DEBUG] broadcast_scene_update: no clients");
        }
        return;
    }
    std::string msg = trent_to_json(make_scene_update_message());
    if (g_debug)
    {
        nos::println("[DEBUG] broadcast_scene_update: sending to ", g_clients.size(), " clients, msg_len=", msg.size());
    }
    broadcast_to_clients(msg);
}

void broadcast_scene_init()
{
    if (g_clients.empty())
        return;
    std::string msg = trent_to_json(make_scene_init_message());
    broadcast_to_clients(msg);
}

// Callbacks for transport listeners
void on_tree_received(const nos::trent &data)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    g_tree_data_json = data;
    g_tree.load(data);
    apply_axis_overrides();
    g_tree.update();
    nos::println("Loaded kinematic tree: ", data["name"].as_string_default("unnamed"));
    nos::println("Joints: ");
    for (const auto &name : g_tree.get_joint_names())
    {
        nos::println("  - ", name);
    }
    broadcast_scene_init();
}

void on_joints_received(const nos::trent &data)
{
    if (g_debug)
    {
        nos::println("[DEBUG] on_joints_received called");
    }

    std::lock_guard<std::mutex> lock(g_mutex);
    const auto &joints_data = data["joints"];
    if (joints_data.is_dict())
    {
        std::map<std::string, double> joints;
        for (const auto &[name, value] : joints_data.as_dict())
        {
            joints[name] = value.as_numer_default(0);
        }
        g_tree.set_joint_coords(joints);
        g_tree.update();

        if (g_debug)
        {
            nos::println("[DEBUG] joints updated, clients=", g_clients.size());
        }
        broadcast_scene_update();
    }
    else if (g_debug)
    {
        nos::println("[DEBUG] on_joints_received: data[joints] is not dict");
    }
}

std::string read_file(const fs::path &path)
{
    std::ifstream file(path, std::ios::binary);
    if (!file)
        return "";
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

std::string get_mime_type(const std::string &filename)
{
    size_t dot_pos = filename.rfind('.');
    if (dot_pos == std::string::npos)
        return "application/octet-stream";
    
    std::string ext = filename.substr(dot_pos);
    if (ext == ".html")
        return "text/html";
    if (ext == ".js")
        return "application/javascript";
    if (ext == ".css")
        return "text/css";
    if (ext == ".json")
        return "application/json";
    if (ext == ".png")
        return "image/png";
    if (ext == ".jpg" || ext == ".jpeg")
        return "image/jpeg";
    if (ext == ".svg")
        return "image/svg+xml";
    if (ext == ".stl")
        return "application/octet-stream";
    return "application/octet-stream";
}

std::string get_mime_type(const fs::path &path)
{
    return get_mime_type(path.string());
}

void signal_handler(int sig)
{
    (void)sig;
    nos::println("\nReceived signal, shutting down...");
    g_running = false;
}

int main(int argc, char *argv[])
{
    nos::println("=== WebKin C++ Server ===");

    // Parse arguments
    std::string host = "0.0.0.0";
    int port = 8000;
    TransportType transport = TransportType::NONE;
    std::string mqtt_broker = "localhost";
    int mqtt_port = 1883;
    std::string mqtt_topic = "robot/joints";
    std::string crowker_addr = ".12.127.0.0.1:10009";
    std::string k3d_file;

    // Check K3D_FILE environment variable
    if (const char *env_k3d = std::getenv("K3D_FILE"))
    {
        k3d_file = env_k3d;
    }

    // Check Z_UP environment variable
    if (const char *env_zup = std::getenv("Z_UP"))
    {
        std::string val = env_zup;
        g_z_up = (val == "1" || val == "true" || val == "yes");
    }

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--host" && i + 1 < argc)
        {
            host = argv[++i];
        }
        else if (arg == "--port" && i + 1 < argc)
        {
            port = std::stoi(argv[++i]);
        }
        else if (arg == "--z-up")
        {
            g_z_up = true;
        }
        else if (arg == "--mqtt")
        {
            transport = TransportType::MQTT;
        }
        else if (arg == "--crow")
        {
            transport = TransportType::CROW;
        }
        else if (arg == "--mqtt-broker" && i + 1 < argc)
        {
            mqtt_broker = argv[++i];
        }
        else if (arg == "--mqtt-port" && i + 1 < argc)
        {
            mqtt_port = std::stoi(argv[++i]);
        }
        else if (arg == "--mqtt-topic" && i + 1 < argc)
        {
            mqtt_topic = argv[++i];
        }
        else if (arg == "--crowker" && i + 1 < argc)
        {
            crowker_addr = argv[++i];
        }
        else if (arg == "--debug" || arg == "-d")
        {
            g_debug = true;
        }
        else if (arg == "--k3d" && i + 1 < argc)
        {
            k3d_file = argv[++i];
        }
        else if (arg == "--static-dir" && i + 1 < argc)
        {
            g_static_dir = argv[++i];
            g_use_embedded_resources = false;
        }
        else if (arg == "--help" || arg == "-h")
        {
            nos::println("Usage: webkin [options]");
            nos::println("Options:");
            nos::println("  --host HOST        Host to bind (default: 0.0.0.0)");
            nos::println("  --port PORT        Port to bind (default: 8000)");
            nos::println("  --z-up             Convert Z-up to Y-up");
            nos::println("  --k3d PATH         Load K3D file or directory (env: K3D_FILE)");
            nos::println("  --static-dir DIR   Use external static files directory");
            nos::println("  --debug, -d        Enable debug output");
            nos::println("");
            nos::println("Transport options:");
            nos::println("  --mqtt             Use MQTT transport");
            nos::println("  --crow             Use Crow protocol transport");
            nos::println("");
            nos::println("MQTT options:");
            nos::println("  --mqtt-broker HOST MQTT broker host (default: localhost)");
            nos::println("  --mqtt-port PORT   MQTT broker port (default: 1883)");
            nos::println("  --mqtt-topic TOPIC MQTT topic prefix (default: robot/joints)");
            nos::println("");
            nos::println("Crow options:");
            nos::println("  --crowker ADDR     Crowker address (default: .12.127.0.0.1:10009)");
            return 0;
        }
    }

    // Setup signal handlers
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Setup paths (only if not using embedded resources or --static-dir was not specified)
    if (g_static_dir.empty())
    {
        g_base_dir = fs::path(argv[0]).parent_path().parent_path();
        if (!fs::exists(g_base_dir / "static"))
        {
            // Try current directory
            g_base_dir = fs::current_path();
        }
        g_static_dir = g_base_dir / "static";
    }

    // Setup config directory (XDG Base Directory Specification)
    const char *xdg_config = std::getenv("XDG_CONFIG_HOME");
    if (xdg_config)
    {
        g_config_dir = fs::path(xdg_config) / "webkin";
    }
    else
    {
        const char *home = std::getenv("HOME");
        if (home)
        {
            g_config_dir = fs::path(home) / ".config" / "webkin";
        }
        else
        {
            g_config_dir = "/tmp/webkin";
        }
    }
    g_axis_overrides_file = g_config_dir / "axis_overrides.json";

    // Load axis overrides
    load_axis_overrides();

    if (g_use_embedded_resources)
    {
        nos::println("Using embedded resources");
    }
    else
    {
        nos::println("Static dir: ", g_static_dir.string());
    }
    nos::println("Config dir: ", g_config_dir.string());

    // Try to load K3D file if specified
    if (!k3d_file.empty())
    {
        fs::path k3d_path = fs::path(k3d_file);
        // Expand ~ to home directory
        if (!k3d_path.empty() && k3d_path.string()[0] == '~')
        {
            const char *home = std::getenv("HOME");
            if (home)
            {
                k3d_path = fs::path(home) / k3d_path.string().substr(2);
            }
        }

        if (fs::exists(k3d_path))
        {
            try
            {
                g_k3d_loader = std::make_unique<webkin::K3DLoader>();
                if (fs::is_directory(k3d_path))
                {
                    g_tree_data_json = g_k3d_loader->load_directory(k3d_path);
                }
                else
                {
                    g_tree_data_json = g_k3d_loader->load_file(k3d_path);
                }
                g_tree.load(g_tree_data_json);
                apply_axis_overrides();
                g_tree.update();
                nos::println("Loaded K3D: ", k3d_file);
                nos::println("Joints: ");
                for (const auto &name : g_tree.get_joint_names())
                {
                    nos::println("  - ", name);
                }
                if (g_k3d_loader->has_models())
                {
                    nos::println("Models dir: ", g_k3d_loader->models_dir().string());
                }
            }
            catch (const std::exception &e)
            {
                nos::println("Failed to load K3D file ", k3d_file, ": ", e.what());
            }
        }
        else
        {
            nos::println("K3D file not found: ", k3d_file);
        }
    }

    // Load fallback tree if no K3D loaded
    if (g_tree_data_json.is_nil())
    {
        fs::path tree_file = g_static_dir / "example_tree.json";
        if (fs::exists(tree_file))
        {
            std::string content = read_file(tree_file);
            g_tree_data_json = nos::json::parse(content);
            g_tree.load(g_tree_data_json);
            nos::println("Loaded fallback tree with joints: ");
            for (const auto &name : g_tree.get_joint_names())
            {
                nos::println("  - ", name);
            }
        }
    }

    // Setup transport
    webkin::mqtt_listener mqtt;
    webkin::crow_listener crow;

    switch (transport)
    {
    case TransportType::MQTT:
    {
        nos::println("Using MQTT transport");
        webkin::mqtt_config cfg;
        cfg.broker_host = mqtt_broker;
        cfg.broker_port = mqtt_port;
        cfg.joints_topic = mqtt_topic;
        cfg.tree_topic = mqtt_topic + "/tree";

        mqtt.set_tree_callback(on_tree_received);
        mqtt.set_joints_callback(on_joints_received);

        if (mqtt.init(cfg))
        {
            if (!mqtt.connect())
            {
                nos::println("Warning: MQTT connection failed, continuing without transport");
            }
        }
        break;
    }
    case TransportType::CROW:
    {
        nos::println("Using Crow protocol transport");
        webkin::crow_config cfg;
        cfg.crowker_addr = crowker_addr;
        cfg.joints_topic = mqtt_topic; // reuse same topic names
        cfg.tree_topic = mqtt_topic + "/tree";

        crow.set_tree_callback(on_tree_received);
        crow.set_joints_callback(on_joints_received);

        if (crow.init(cfg))
        {
            if (!crow.connect())
            {
                nos::println("Warning: Crow connection failed, continuing without transport");
            }
        }
        break;
    }
    case TransportType::NONE:
        nos::println("No transport configured (use --mqtt or --crow to enable)");
        break;
    }

    // Create Crow HTTP app
    crowhttp::SimpleApp app;

    // Main page
    CROW_ROUTE(app, "/")
    ([]()
     {
        std::string content = read_static_resource("index.html");
        if (content.empty()) {
            return crowhttp::response(404, "Not found");
        }
        crowhttp::response res(200, content);
        res.set_header("Content-Type", "text/html");
        return res; });

    // Static files
    CROW_ROUTE(app, "/static/<path>")
    ([](const std::string &path)
     {
        // Security: prevent directory traversal
        if (path.find("..") != std::string::npos) {
            return crowhttp::response(403, "Forbidden");
        }

        std::string content = read_static_resource(path);
        if (content.empty()) {
            return crowhttp::response(404, "Not found");
        }

        crowhttp::response res(200, content);
        res.set_header("Content-Type", get_mime_type(path));
        return res; });

    // K3D model files
    CROW_ROUTE(app, "/k3d/models/<path>")
    ([](const std::string &filename)
     {
        if (!g_k3d_loader || !g_k3d_loader->has_models())
        {
            return crowhttp::response(404, "No K3D file loaded");
        }

        fs::path model_path = g_k3d_loader->get_model_path(filename);
        if (model_path.empty())
        {
            return crowhttp::response(404, "Model not found: " + filename);
        }

        std::string content = read_file(model_path);
        if (content.empty())
        {
            return crowhttp::response(404, "Failed to read model");
        }

        crowhttp::response res(200, content);
        res.set_header("Content-Type", "application/octet-stream");
        return res; });

    // REST API: Get tree
    CROW_ROUTE(app, "/api/tree")
    ([]()
     {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (g_tree_data_json.is_nil()) {
            return crowhttp::response(200, R"({"error": "No tree loaded"})");
        }
        crowhttp::response res(200, trent_to_json(g_tree_data_json));
        res.set_header("Content-Type", "application/json");
        return res; });

    // REST API: Get scene
    CROW_ROUTE(app, "/api/scene")
    ([]()
     {
        std::lock_guard<std::mutex> lock(g_mutex);
        nos::trent scene = g_tree.get_scene_data();
        crowhttp::response res(200, trent_to_json(scene));
        res.set_header("Content-Type", "application/json");
        return res; });

    // REST API: Set joints
    CROW_ROUTE(app, "/api/joints").methods("POST"_method)([](const crowhttp::request &req)
                                                          {
        std::lock_guard<std::mutex> lock(g_mutex);

        nos::trent body = nos::json::parse(req.body);
        std::map<std::string, double> joints;

        if (body.is_dict()) {
            for (const auto& [name, value] : body.as_dict()) {
                joints[name] = value.as_numer_default(0);
            }
        }

        g_tree.set_joint_coords(joints);
        g_tree.update();
        broadcast_scene_update();

        crowhttp::response res(200, R"({"status": "ok"})");
        res.set_header("Content-Type", "application/json");
        return res; });

    // REST API: Load tree
    CROW_ROUTE(app, "/api/tree").methods("POST"_method)([](const crowhttp::request &req)
                                                        {
        std::lock_guard<std::mutex> lock(g_mutex);

        g_tree_data_json = nos::json::parse(req.body);
        g_tree.load(g_tree_data_json);

        nos::println("Loaded tree via REST: ", g_tree_data_json["name"].as_string_default("unnamed"));
        nos::println("Joints: ");
        for (const auto& name : g_tree.get_joint_names()) {
            nos::println("  - ", name);
        }

        broadcast_scene_init();

        nos::trent response;
        response.init(nos::trent::type::dict);
        response["status"] = "ok";
        response["joints"] = g_tree.get_joint_names_trent();

        crowhttp::response res(200, trent_to_json(response));
        res.set_header("Content-Type", "application/json");
        return res; });

    // REST API: Set zero offset for a joint
    CROW_ROUTE(app, "/api/offset/set_zero").methods("POST"_method)([](const crowhttp::request &req)
                                                                   {
        std::lock_guard<std::mutex> lock(g_mutex);

        nos::trent body = nos::json::parse(req.body);
        std::string joint_name = body["joint_name"].as_string_default("");

        if (joint_name.empty())
        {
            crowhttp::response res(400, R"({"error": "joint_name is required"})");
            res.set_header("Content-Type", "application/json");
            return res;
        }

        auto it = g_tree.joints.find(joint_name);
        if (it == g_tree.joints.end())
        {
            crowhttp::response res(404, R"({"error": "Joint not found"})");
            res.set_header("Content-Type", "application/json");
            return res;
        }

        // Set offset so that current position becomes zero
        double new_offset = -it->second->coord;
        g_axis_overrides[joint_name]["axis_offset"] = new_offset;
        it->second->axis_offset = new_offset;

        save_axis_overrides();
        g_tree.update();
        broadcast_scene_update();

        nos::trent response;
        response.init(nos::trent::type::dict);
        response["status"] = "ok";
        response["joint"] = joint_name;
        response["offset"] = new_offset;

        crowhttp::response res(200, trent_to_json(response));
        res.set_header("Content-Type", "application/json");
        return res; });

    // REST API: Set axis override (POST /api/axis/override)
    CROW_ROUTE(app, "/api/axis/override").methods("POST"_method)([](const crowhttp::request &req)
                                                                 {
        std::lock_guard<std::mutex> lock(g_mutex);

        nos::trent body = nos::json::parse(req.body);
        std::string joint_name = body["joint_name"].as_string_default("");

        if (joint_name.empty())
        {
            crowhttp::response res(400, R"({"error": "joint_name is required"})");
            res.set_header("Content-Type", "application/json");
            return res;
        }

        auto it = g_tree.joints.find(joint_name);
        if (it == g_tree.joints.end())
        {
            crowhttp::response res(404, R"({"error": "Joint not found"})");
            res.set_header("Content-Type", "application/json");
            return res;
        }

        // Update overrides from body
        if (body["axis_offset"].is_numer())
        {
            double val = body["axis_offset"].as_numer();
            g_axis_overrides[joint_name]["axis_offset"] = val;
            it->second->axis_offset = val;
            nos::println("Set axis_offset for ", joint_name, " = ", val);
        }
        if (body["axis_scale"].is_numer())
        {
            double val = body["axis_scale"].as_numer();
            g_axis_overrides[joint_name]["axis_scale"] = val;
            it->second->axis_scale = val;
            nos::println("Set axis_scale for ", joint_name, " = ", val);
        }
        if (body["slider_min"].is_numer())
        {
            double val = body["slider_min"].as_numer();
            g_axis_overrides[joint_name]["slider_min"] = val;
            it->second->slider_min = val;
            nos::println("Set slider_min for ", joint_name, " = ", val);
        }
        if (body["slider_max"].is_numer())
        {
            double val = body["slider_max"].as_numer();
            g_axis_overrides[joint_name]["slider_max"] = val;
            it->second->slider_max = val;
            nos::println("Set slider_max for ", joint_name, " = ", val);
        }

        save_axis_overrides();
        g_tree.update();
        broadcast_scene_update();
        nos::println("Applied axis override for ", joint_name, ", broadcasted update");

        nos::trent response;
        response.init(nos::trent::type::dict);
        response["status"] = "ok";
        response["joint"] = joint_name;

        crowhttp::response res(200, trent_to_json(response));
        res.set_header("Content-Type", "application/json");
        return res; });

    // REST API: Get axis overrides
    CROW_ROUTE(app, "/api/axis/overrides")
    ([]()
     {
        std::lock_guard<std::mutex> lock(g_mutex);

        nos::trent overrides;
        overrides.init(nos::trent::type::dict);
        for (const auto &[name, params] : g_axis_overrides)
        {
            nos::trent joint_params;
            joint_params.init(nos::trent::type::dict);
            for (const auto &[key, value] : params)
            {
                joint_params[key] = value;
            }
            overrides[name] = std::move(joint_params);
        }

        nos::trent response;
        response.init(nos::trent::type::dict);
        response["overrides"] = std::move(overrides);

        crowhttp::response res(200, trent_to_json(response));
        res.set_header("Content-Type", "application/json");
        return res; });

    // REST API: Clear all axis overrides
    CROW_ROUTE(app, "/api/axis/overrides").methods("DELETE"_method)([]()
                                                                    {
        std::lock_guard<std::mutex> lock(g_mutex);

        g_axis_overrides.clear();
        save_axis_overrides();

        // Reload tree to restore original values
        if (!g_tree_data_json.is_nil())
        {
            g_tree.load(g_tree_data_json);
            g_tree.update();
            broadcast_scene_update();
        }

        crowhttp::response res(200, R"({"status": "ok"})");
        res.set_header("Content-Type", "application/json");
        return res; });

    // REST API: Clear axis overrides for specific joint
    CROW_ROUTE(app, "/api/axis/overrides/<string>").methods("DELETE"_method)([](const std::string &joint_name)
                                                                              {
        std::lock_guard<std::mutex> lock(g_mutex);

        auto it = g_axis_overrides.find(joint_name);
        if (it != g_axis_overrides.end())
        {
            g_axis_overrides.erase(it);
            save_axis_overrides();

            // Restore original values
            if (!g_tree_data_json.is_nil())
            {
                auto joint_it = g_tree.joints.find(joint_name);
                if (joint_it != g_tree.joints.end())
                {
                    nos::trent original = find_original_axis_params(g_tree_data_json, joint_name);
                    if (!original.is_nil())
                    {
                        joint_it->second->axis_offset = original["axis_offset"].as_numer_default(0.0);
                        joint_it->second->axis_scale = original["axis_scale"].as_numer_default(1.0);
                        std::string jtype = joint_it->second->type;
                        double default_min = (jtype == "actuator") ? -1000.0 : -180.0;
                        double default_max = (jtype == "actuator") ? 1000.0 : 180.0;
                        joint_it->second->slider_min = original["slider_min"].as_numer_default(default_min);
                        joint_it->second->slider_max = original["slider_max"].as_numer_default(default_max);
                        g_tree.update();
                        broadcast_scene_update();
                    }
                }
            }
        }

        crowhttp::response res(200, R"({"status": "ok"})");
        res.set_header("Content-Type", "application/json");
        return res; });

    // WebSocket endpoint
    CROW_WEBSOCKET_ROUTE(app, "/ws")
        .onopen([](crowhttp::websocket::connection &conn)
                {
            std::lock_guard<std::mutex> lock(g_mutex);
            g_clients.insert(&conn);
            nos::println("Client connected. Total: ", g_clients.size());

            // Send initial scene state
            std::string msg = trent_to_json(make_scene_init_message());
            conn.send_text(msg); })
        .onclose([](crowhttp::websocket::connection &conn, const std::string &reason, uint16_t code)
                 {
            (void)reason;
            (void)code;
            std::lock_guard<std::mutex> lock(g_mutex);
            g_clients.erase(&conn);
            nos::println("Client disconnected. Total: ", g_clients.size()); })
        .onmessage([](crowhttp::websocket::connection &conn, const std::string &data, bool is_binary)
                   {
            (void)conn;
            if (is_binary) return;

            std::lock_guard<std::mutex> lock(g_mutex);

            nos::trent message = nos::json::parse(data);
            std::string msg_type = message["type"].as_string_default("");

            if (msg_type == "joint_update") {
                const auto& joints_data = message["joints"];
                if (joints_data.is_dict()) {
                    std::map<std::string, double> joints;
                    for (const auto& [name, value] : joints_data.as_dict()) {
                        joints[name] = value.as_numer_default(0);
                    }
                    g_tree.set_joint_coords(joints);
                    g_tree.update();
                    broadcast_scene_update();
                }
            } });

    nos::println("");
    nos::println("Server starting on http://", host, ":", port);
    nos::println("Press Ctrl+C to stop.");
    nos::println("");

    app.bindaddr(host).port(port).multithreaded().run();

    // Cleanup
    mqtt.disconnect();
    crow.disconnect();

    nos::println("Goodbye!");
    return 0;
}
