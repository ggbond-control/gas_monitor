#include "gas_monitor/pump_config_loader.hpp"

#include <exception>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace gas_monitor
{
    namespace
    {

        YAML::Node load_parameters(const std::string &path)
        {
            const YAML::Node root = YAML::LoadFile(path);
            const YAML::Node node = root["serial_gas_node"];
            if (!node || !node.IsMap())
            {
                throw YAML::Exception(root.Mark(), "missing serial_gas_node section");
            }

            const YAML::Node parameters = node["ros__parameters"];
            if (!parameters || !parameters.IsMap())
            {
                throw YAML::Exception(node.Mark(), "missing serial_gas_node.ros__parameters section");
            }
            return parameters;
        }

    } // namespace

    bool load_pump_agent_config_from_yaml(const std::string &path, PumpAgentStartupConfig *config, std::string *error)
    {
        if (config == nullptr)
        {
            if (error != nullptr)
            {
                *error = "气体泵 agent 配置输出参数为空";
            }
            return false;
        }

        try
        {
            const YAML::Node parameters = load_parameters(path);
            PumpAgentStartupConfig loaded;

            if (parameters["pump_relay_socket_path"])
                loaded.socket_path = parameters["pump_relay_socket_path"].as<std::string>();
            if (parameters["pump_relay_gpio"])
                loaded.pump_config.relay_gpio = parameters["pump_relay_gpio"].as<int>();
            if (parameters["pump_relay_active_high"])
                loaded.pump_config.active_high = parameters["pump_relay_active_high"].as<bool>();

            *config = std::move(loaded);
            return true;
        }
        catch (const std::exception &exception)
        {
            if (error != nullptr)
            {
                *error = "加载气体泵 agent 配置失败(" + path + "): " + exception.what();
            }
            return false;
        }
    }

} // namespace gas_monitor
