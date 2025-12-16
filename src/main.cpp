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

namespace fs = std::filesystem;

// Global state
webkin::KinematicTree g_tree;
nos::trent g_tree_data_json;
std::mutex g_mutex;
std::set<crowhttp::websocket::connection *> g_clients;
bool g_z_up = false;
std::atomic<bool> g_running{true};

// Paths
fs::path g_base_dir;
fs::path g_static_dir;

// K3D loader
std::unique_ptr<webkin::K3DLoader> g_k3d_loader;

// Transport type
enum class TransportType
{
    NONE,
    MQTT,
    CROW
};

std::string trent_to_json(const nos::trent &t)
{
    return nos::json::to_string(t);
}

nos::trent make_scene_init_message()
{
    nos::trent msg;
    msg.init(nos::trent::type::dict);
    msg["type"] = "scene_init";
    msg["nodes"] = g_tree.get_scene_data();
    msg["joints"] = g_tree.get_joint_names_trent();
    msg["zUp"] = g_z_up;
    return msg;
}

nos::trent make_scene_update_message()
{
    nos::trent msg;
    msg.init(nos::trent::type::dict);
    msg["type"] = "scene_update";
    msg["nodes"] = g_tree.get_scene_data();
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
        return;
    std::string msg = trent_to_json(make_scene_update_message());
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
        broadcast_scene_update();
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

std::string get_mime_type(const fs::path &path)
{
    std::string ext = path.extension().string();
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
        else if (arg == "--k3d" && i + 1 < argc)
        {
            k3d_file = argv[++i];
        }
        else if (arg == "--help" || arg == "-h")
        {
            nos::println("Usage: webkin [options]");
            nos::println("Options:");
            nos::println("  --host HOST        Host to bind (default: 0.0.0.0)");
            nos::println("  --port PORT        Port to bind (default: 8000)");
            nos::println("  --z-up             Convert Z-up to Y-up");
            nos::println("  --k3d PATH         Load K3D file or directory (env: K3D_FILE)");
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

    // Setup paths
    g_base_dir = fs::path(argv[0]).parent_path().parent_path();
    if (!fs::exists(g_base_dir / "static"))
    {
        // Try current directory
        g_base_dir = fs::current_path();
    }
    g_static_dir = g_base_dir / "static";

    nos::println("Static dir: ", g_static_dir.string());

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
        std::string content = read_file(g_static_dir / "index.html");
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
        fs::path file_path = g_static_dir / path;

        // Security: prevent directory traversal
        fs::path canonical = fs::weakly_canonical(file_path);
        if (canonical.string().find(fs::weakly_canonical(g_static_dir).string()) != 0) {
            return crowhttp::response(403, "Forbidden");
        }

        std::string content = read_file(file_path);
        if (content.empty()) {
            return crowhttp::response(404, "Not found");
        }

        crowhttp::response res(200, content);
        res.set_header("Content-Type", get_mime_type(file_path));
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
