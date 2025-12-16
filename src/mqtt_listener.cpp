/**
 * MQTT Listener implementation for WebKin
 */

#include "mqtt_listener.hpp"
#include <nos/print.h>
#include <nos/trent/json.h>

namespace webkin
{

mqtt_listener::~mqtt_listener()
{
    disconnect();
#ifdef HAVE_MOSQUITTO
    mosquitto_lib_cleanup();
#endif
}

bool mqtt_listener::init(const mqtt_config &config)
{
    _config = config;

    if (!_config.enabled)
    {
        nos::println("MQTT: disabled by configuration");
        return true;
    }

#ifdef HAVE_MOSQUITTO
    mosquitto_lib_init();
    _mosq = mosquitto_new("webkin", true, this);
    if (!_mosq)
    {
        nos::println("MQTT: failed to create mosquitto client");
        return false;
    }

    // Set callbacks
    mosquitto_connect_callback_set(static_cast<mosquitto *>(_mosq), on_connect);
    mosquitto_message_callback_set(static_cast<mosquitto *>(_mosq), on_message);
    mosquitto_disconnect_callback_set(static_cast<mosquitto *>(_mosq), on_disconnect);

    nos::println("MQTT: initialized");
    return true;
#else
    nos::println("MQTT: not compiled with mosquitto support");
    return false;
#endif
}

bool mqtt_listener::connect()
{
    if (!_config.enabled || !_mosq)
        return false;

#ifdef HAVE_MOSQUITTO
    int rc = mosquitto_connect(
        static_cast<mosquitto *>(_mosq),
        _config.broker_host.c_str(),
        _config.broker_port,
        60 // keepalive
    );

    if (rc != MOSQ_ERR_SUCCESS)
    {
        nos::println("MQTT: connection failed: ", mosquitto_strerror(rc));
        return false;
    }

    _connected = true;
    nos::println("MQTT: connected to ", _config.broker_host, ":", _config.broker_port);

    // Start network loop in background thread
    _running = true;
    _loop_thread = std::thread([this]()
                               {
        while (_running) {
            mosquitto_loop(static_cast<mosquitto*>(_mosq), 100, 1);
        } });

    return true;
#else
    return false;
#endif
}

void mqtt_listener::disconnect()
{
    if (!_connected || !_mosq)
        return;

#ifdef HAVE_MOSQUITTO
    _running = false;
    if (_loop_thread.joinable())
    {
        _loop_thread.join();
    }

    mosquitto_disconnect(static_cast<mosquitto *>(_mosq));
    mosquitto_destroy(static_cast<mosquitto *>(_mosq));
    _mosq = nullptr;
    _connected = false;
    nos::println("MQTT: disconnected");
#endif
}

#ifdef HAVE_MOSQUITTO

void mqtt_listener::on_connect(struct mosquitto *mosq, void *userdata, int rc)
{
    auto *self = static_cast<mqtt_listener *>(userdata);

    if (rc == 0)
    {
        nos::println("MQTT: connected, subscribing to topics...");

        // Subscribe to tree topic
        mosquitto_subscribe(mosq, nullptr, self->_config.tree_topic.c_str(), 0);
        nos::println("MQTT: subscribed to ", self->_config.tree_topic);

        // Subscribe to joints topic
        mosquitto_subscribe(mosq, nullptr, self->_config.joints_topic.c_str(), 0);
        nos::println("MQTT: subscribed to ", self->_config.joints_topic);
    }
    else
    {
        nos::println("MQTT: connect failed: ", mosquitto_connack_string(rc));
    }
}

void mqtt_listener::on_message(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *msg)
{
    (void)mosq;
    auto *self = static_cast<mqtt_listener *>(userdata);

    if (!msg->payload || msg->payloadlen == 0)
        return;

    std::string topic(msg->topic);
    std::string payload(static_cast<char *>(msg->payload), msg->payloadlen);

    try
    {
        nos::trent data = nos::json::parse(payload);

        if (topic == self->_config.tree_topic)
        {
            nos::println("MQTT: received kinematic tree");
            if (self->_on_tree)
            {
                self->_on_tree(data);
            }
        }
        else if (topic == self->_config.joints_topic)
        {
            if (self->_on_joints)
            {
                self->_on_joints(data);
            }
        }
    }
    catch (const std::exception &e)
    {
        nos::println("MQTT: failed to parse JSON: ", e.what());
    }
}

void mqtt_listener::on_disconnect(struct mosquitto *mosq, void *userdata, int rc)
{
    (void)mosq;
    auto *self = static_cast<mqtt_listener *>(userdata);
    self->_connected = false;

    if (rc != 0)
    {
        nos::println("MQTT: unexpected disconnect, will try to reconnect...");
    }
}

#endif // HAVE_MOSQUITTO

} // namespace webkin
