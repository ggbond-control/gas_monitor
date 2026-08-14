#include <csignal>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "gas_monitor/pump_agent.hpp"
#include "gas_monitor/pump_config_loader.hpp"

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

    const std::string config_path = argc > 1 ? argv[1] : "config/gas_params_default.yaml";
    gas_monitor::PumpAgentStartupConfig startup_config;
    std::string error;
    if (!gas_monitor::load_pump_agent_config_from_yaml(config_path, &startup_config, &error))
    {
        RCLCPP_ERROR(rclcpp::get_logger("gas_monitor_pump_agent"), "%s", error.c_str());
        rclcpp::shutdown();
        return 1;
    }

    g_agent = std::make_shared<gas_monitor::PumpAgent>(startup_config.socket_path, startup_config.pump_config, startup_config.initial_state);
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    const bool ok = g_agent->run(&error);
    if (!ok)
        RCLCPP_ERROR(rclcpp::get_logger("gas_monitor_pump_agent"), "%s", error.c_str());

    g_agent.reset();
    rclcpp::shutdown();
    return ok ? 0 : 1;
}
