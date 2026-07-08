#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace gas_monitor
{

    constexpr const char kPumpProtocolMagic[4] = {'G', 'P', 'M', '1'};
    constexpr std::uint16_t kPumpProtocolVersion = 1;
    constexpr const char kDefaultPumpSocketPath[] = "/run/gas_monitor/pump_relay.sock";

    enum class PumpMessageType : std::uint16_t
    {
        kConfigureRequest = 1,
        kConfigureResponse = 2,
        kSetStateRequest = 3,
        kSetStateResponse = 4,
        kGetStatusRequest = 5,
        kGetStatusResponse = 6,
    };

    struct PumpMessageHeader
    {
        char magic[4];
        std::uint16_t version;
        std::uint16_t type;
        std::uint32_t payload_size;
    };

    struct PumpConfig
    {
        int32_t relay_gpio{-1};
        bool active_high{true};
    };

    struct PumpStateCommand
    {
        bool enable{false};
    };

    struct PumpStatusResponse
    {
        bool success{false};
        bool agent_ready{false};
        bool configured{false};
        bool enable{false};
        bool active_high{true};
        int32_t relay_gpio{-1};
        std::string message;
    };

    std::vector<std::uint8_t> serialize_pump_config(const PumpConfig &config);
    bool deserialize_pump_config(const std::vector<std::uint8_t> &payload, PumpConfig *config, std::string *error);

    std::vector<std::uint8_t> serialize_pump_state_command(const PumpStateCommand &command);
    bool deserialize_pump_state_command(const std::vector<std::uint8_t> &payload, PumpStateCommand *command, std::string *error);

    std::vector<std::uint8_t> serialize_pump_status_response(const PumpStatusResponse &response);
    bool deserialize_pump_status_response(const std::vector<std::uint8_t> &payload, PumpStatusResponse *response, std::string *error);

    bool write_pump_message(int fd, PumpMessageType type, const std::vector<std::uint8_t> &payload, std::string *error);
    bool read_pump_message(int fd, PumpMessageType *type, std::vector<std::uint8_t> *payload, std::string *error);

} // namespace gas_monitor
