#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include "gas_monitor/pump_protocol.hpp"

namespace gas_monitor
{

    class PumpAgent
    {
    public:
        PumpAgent(std::string socket_path, PumpConfig initial_config, PumpStateCommand initial_state);
        ~PumpAgent();

        bool run(std::string *error);
        void request_shutdown();

    private:
        struct RuntimeState
        {
            PumpConfig config;
            PumpStateCommand state;
            bool configured{false};
        };

        bool ensure_runtime_dir(std::string *error) const;
        void accept_loop();
        void handle_client(int client_fd);
        bool process_request(PumpMessageType request_type,
                             const std::vector<std::uint8_t> &payload,
                             PumpMessageType *response_type,
                             std::vector<std::uint8_t> *response_payload);

        PumpStatusResponse handle_configure(const PumpConfig &config);
        PumpStatusResponse handle_set_state(const PumpStateCommand &command);
        PumpStatusResponse handle_get_status();
        PumpStatusResponse build_status_locked() const;

        bool validate_config(const PumpConfig &config, std::string *error) const;
        bool init_gpio_locked(int pin, std::string *error);
        bool write_gpio_locked(int pin, bool on, std::string *error) const;
        void set_output_locked(bool on);

        std::string socket_path_;
        std::atomic<bool> shutdown_requested_{false};
        int server_fd_{-1};

        mutable std::mutex state_mutex_;
        RuntimeState runtime_state_;
        std::unordered_set<int> initialized_gpios_;
    };

} // namespace gas_monitor
