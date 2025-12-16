#pragma once

/**
 * MQTT Listener for WebKin
 *
 * Receives kinematic tree configuration and joint updates via MQTT.
 */

#include <string>
#include <functional>
#include <atomic>
#include <thread>
#include <nos/trent/trent.h>

#ifdef HAVE_MOSQUITTO
#include <mosquitto.h>
#endif

namespace webkin
{

struct mqtt_config
{
    bool enabled = true;
    std::string broker_host = "localhost";
    int broker_port = 1883;
    std::string joints_topic = "robot/joints";
    std::string tree_topic = "robot/joints/tree";
};

class mqtt_listener
{
public:
    using tree_callback_t = std::function<void(const nos::trent &)>;
    using joints_callback_t = std::function<void(const nos::trent &)>;

    mqtt_listener() = default;
    ~mqtt_listener();

    bool init(const mqtt_config &config);
    bool connect();
    void disconnect();

    void set_tree_callback(tree_callback_t cb) { _on_tree = std::move(cb); }
    void set_joints_callback(joints_callback_t cb) { _on_joints = std::move(cb); }

    bool is_connected() const { return _connected; }

private:
    mqtt_config _config;
    void *_mosq = nullptr;
    std::atomic<bool> _connected{false};
    std::atomic<bool> _running{false};
    std::thread _loop_thread;

    tree_callback_t _on_tree;
    joints_callback_t _on_joints;

#ifdef HAVE_MOSQUITTO
    static void on_connect(struct mosquitto *mosq, void *userdata, int rc);
    static void on_message(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *msg);
    static void on_disconnect(struct mosquitto *mosq, void *userdata, int rc);
#endif
};

} // namespace webkin
