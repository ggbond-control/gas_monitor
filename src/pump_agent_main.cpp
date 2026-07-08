#include <csignal>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "gas_monitor/pump_agent.hpp"
#include "gas_monitor/pump_protocol.hpp"

namespace
{

    std::shared_ptr<gas_monitor::PumpAgent> g_agent;

    void signal_handler(int)
    {
        if (g_agent)
            g_agent->request_shutdown();
    }

} // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto loader = std::make_shared<rclcpp::Node>(
        "serial_gas_node",
        rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(true)
            .arguments(argc > 1 ? std::vector<std::string>{"--ros-args", "--params-file", argv[1]} : std::vector<std::string>{}));

    std::string socket_path = gas_monitor::kDefaultPumpSocketPath;
    if (loader->has_parameter("pump_relay_socket_path"))
        socket_path = loader->get_parameter("pump_relay_socket_path").as_string();

    gas_monitor::PumpConfig config;
    config.relay_gpio = loader->has_parameter("pump_relay_gpio") ? static_cast<int>(loader->get_parameter("pump_relay_gpio").as_int()) : -1;
    config.active_high = loader->has_parameter("pump_relay_active_high") ? loader->get_parameter("pump_relay_active_high").as_bool() : true;

    gas_monitor::PumpStateCommand state;
    state.enable = false;

    g_agent = std::make_shared<gas_monitor::PumpAgent>(socket_path, config, state);
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::string error;
    const bool ok = g_agent->run(&error);
    if (!ok)
        RCLCPP_ERROR(rclcpp::get_logger("gas_monitor_pump_agent"), "%s", error.c_str());

    g_agent.reset();
    rclcpp::shutdown();
    return ok ? 0 : 1;
}
