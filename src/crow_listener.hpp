#pragma once

/**
 * Crow Protocol Listener for WebKin
 *
 * Receives kinematic tree configuration and joint updates via Crow pub/sub.
 */

#include <string>
#include <functional>
#include <atomic>
#include <memory>
#include <nos/trent/trent.h>

#ifdef HAVE_CROW
#include <crow/tower.h>
#include <crow/tower_cls.h>
#include <crow/tower_thread_executor.h>
#include <crow/gates/udpgate.h>
#include <crow/nodes/subscriber_node.h>
#include <crow/hostaddr.h>
#endif

namespace webkin
{

struct crow_config
{
    bool enabled = true;
    std::string crowker_addr = ".12.127.0.0.1:10009";
    std::string joints_topic = "robot/joints";
    std::string tree_topic = "robot/joints/tree";
};

class crow_listener
{
public:
    using tree_callback_t = std::function<void(const nos::trent &)>;
    using joints_callback_t = std::function<void(const nos::trent &)>;

    crow_listener() = default;
    ~crow_listener();

    bool init(const crow_config &config);
    bool connect();
    void disconnect();

    void set_tree_callback(tree_callback_t cb) { _on_tree = std::move(cb); }
    void set_joints_callback(joints_callback_t cb) { _on_joints = std::move(cb); }

    bool is_connected() const { return _connected; }

private:
    crow_config _config;
    std::atomic<bool> _connected{false};
    std::atomic<bool> _spin_started{false};

    tree_callback_t _on_tree;
    joints_callback_t _on_joints;

#ifdef HAVE_CROW
    crow::Tower _tower;
    std::unique_ptr<crow::TowerThreadExecutor> _executor;
    std::shared_ptr<crow::udpgate> _udpgate;
    std::unique_ptr<crow::subscriber_node> _tree_subscriber;
    std::unique_ptr<crow::subscriber_node> _joints_subscriber;

    void handle_tree_message(nos::buffer data);
    void handle_joints_message(nos::buffer data);
#endif
};

} // namespace webkin
