#pragma once

#include <string>

#include "gas_monitor/pump_protocol.hpp"

namespace gas_monitor
{

    struct PumpAgentStartupConfig
    {
        std::string socket_path{kDefaultPumpSocketPath};
        PumpConfig pump_config;
        PumpStateCommand initial_state;
    };

    bool load_pump_agent_config_from_yaml(const std::string &path, PumpAgentStartupConfig *config, std::string *error);

} // namespace gas_monitor
