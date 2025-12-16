/**
 * Crow Protocol Listener implementation for WebKin
 */

#include "crow_listener.hpp"
#include <nos/print.h>
#include <nos/trent/json.h>

namespace webkin
{

crow_listener::~crow_listener()
{
    disconnect();
}

bool crow_listener::init(const crow_config &config)
{
    _config = config;

    if (!_config.enabled)
    {
        nos::println("Crow: disabled by configuration");
        return true;
    }

#ifdef HAVE_CROW
    // Create UDP gate for crow communication
    _udpgate = std::make_shared<crow::udpgate>();
    if (_udpgate->open(0) != 0)
    {
        nos::println("Crow: failed to open UDP gate");
        return false;
    }
    _udpgate->bind(_tower, CROW_UDPGATE_NO);
    nos::println("Crow: initialized with crowker address ", _config.crowker_addr);
    return true;
#else
    nos::println("Crow: not compiled with crow support");
    return false;
#endif
}

bool crow_listener::connect()
{
#ifdef HAVE_CROW
    if (!_config.enabled)
        return false;

    // Start tower executor
    if (!_spin_started)
    {
        _executor = std::make_unique<crow::TowerThreadExecutor>(_tower);
        _executor->start();
        _spin_started = true;
    }

    crow::hostaddr addr(_config.crowker_addr);

    // Create tree subscriber
    _tree_subscriber = std::make_unique<crow::subscriber>();
    _tree_subscriber->init(
        igris::make_delegate(&crow_listener::handle_tree_message, this));
    _tree_subscriber->subscribe(
        _tower,
        addr.view(),
        _config.tree_topic.c_str(),
        1,   // qos
        100, // ackquant
        0,   // rqos
        50   // rackquant
    );
    nos::println("Crow: subscribed to ", _config.tree_topic);

    // Create joints subscriber
    _joints_subscriber = std::make_unique<crow::subscriber>();
    _joints_subscriber->init(
        igris::make_delegate(&crow_listener::handle_joints_message, this));
    _joints_subscriber->subscribe(
        _tower,
        addr.view(),
        _config.joints_topic.c_str(),
        0,  // qos - unreliable for frequent updates
        50, // ackquant
        0,  // rqos
        50  // rackquant
    );
    nos::println("Crow: subscribed to ", _config.joints_topic);

    _connected = true;
    nos::println("Crow: connected to crowker at ", _config.crowker_addr);
    return true;
#else
    return false;
#endif
}

void crow_listener::disconnect()
{
#ifdef HAVE_CROW
    if (_spin_started)
    {
        if (_executor)
        {
            _executor->stop(true);
            _executor.reset();
        }
        _spin_started = false;
    }

    _tree_subscriber.reset();
    _joints_subscriber.reset();

    _connected = false;
    nos::println("Crow: disconnected");
#endif
}

#ifdef HAVE_CROW

void crow_listener::handle_tree_message(crow::pubsub_packet_ptr pack)
{
    nos::buffer data = pack.message();
    std::string payload(data.data(), data.size());

    try
    {
        nos::trent tree_data = nos::json::parse(payload);
        nos::println("Crow: received kinematic tree");
        if (_on_tree)
        {
            _on_tree(tree_data);
        }
    }
    catch (const std::exception &e)
    {
        nos::println("Crow: failed to parse tree JSON: ", e.what());
    }
}

void crow_listener::handle_joints_message(crow::pubsub_packet_ptr pack)
{
    nos::buffer data = pack.message();
    std::string payload(data.data(), data.size());

    try
    {
        nos::trent joints_data = nos::json::parse(payload);
        if (_on_joints)
        {
            _on_joints(joints_data);
        }
    }
    catch (const std::exception &e)
    {
        nos::println("Crow: failed to parse joints JSON: ", e.what());
    }
}

#endif // HAVE_CROW

} // namespace webkin
