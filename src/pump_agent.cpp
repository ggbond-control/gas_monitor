#include "gas_monitor/pump_agent.hpp"

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>

#include <rclcpp/rclcpp.hpp>

namespace gas_monitor
{
    namespace
    {

        std::string gpio_dir(int pin)
        {
            return "/sys/class/gpio/gpio" + std::to_string(pin);
        }

        std::string gpio_value_path(int pin)
        {
            return gpio_dir(pin) + "/value";
        }

        std::string gpio_direction_path(int pin)
        {
            return gpio_dir(pin) + "/direction";
        }

        bool write_text_file(const std::string &path, const std::string &value, std::string *error)
        {
            std::ofstream stream(path);
            if (!stream.is_open())
            {
                if (error)
                    *error = "open failed: " + path + " errno=" + std::to_string(errno);
                return false;
            }
            stream << value;
            if (!stream.good())
            {
                if (error)
                    *error = "write failed: " + path;
                return false;
            }
            return true;
        }

    } // namespace

    PumpAgent::PumpAgent(std::string socket_path, PumpConfig initial_config, PumpStateCommand initial_state)
        : socket_path_(std::move(socket_path))
    {
        runtime_state_.config = std::move(initial_config);
        runtime_state_.state = std::move(initial_state);
    }

    PumpAgent::~PumpAgent()
    {
        request_shutdown();
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            set_output_locked(false);
        }
        if (server_fd_ >= 0)
        {
            ::close(server_fd_);
            server_fd_ = -1;
        }
        std::error_code ec;
        std::filesystem::remove(socket_path_, ec);
    }

    bool PumpAgent::run(std::string *error)
    {
        if (!ensure_runtime_dir(error))
            return false;

        std::error_code ec;
        std::filesystem::remove(socket_path_, ec);

        server_fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
        if (server_fd_ < 0)
        {
            if (error)
                *error = "创建 Unix socket 失败";
            return false;
        }

        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", socket_path_.c_str());
        if (::bind(server_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
        {
            if (error)
                *error = "绑定 Unix socket 失败: " + std::string(std::strerror(errno));
            return false;
        }
        ::chmod(socket_path_.c_str(), 0666);

        if (::listen(server_fd_, 4) < 0)
        {
            if (error)
                *error = "监听 Unix socket 失败";
            return false;
        }

        RCLCPP_INFO(rclcpp::get_logger("gas_monitor_pump_agent"), "气体泵继电器root权限进程代理已启动，socket=%s", socket_path_.c_str());
        accept_loop();
        return true;
    }

    void PumpAgent::request_shutdown()
    {
        shutdown_requested_ = true;
        if (server_fd_ >= 0)
            ::shutdown(server_fd_, SHUT_RDWR);
    }

    bool PumpAgent::ensure_runtime_dir(std::string *error) const
    {
        const std::filesystem::path socket_path(socket_path_);
        const auto dir = socket_path.parent_path();
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        if (ec)
        {
            if (error)
                *error = "创建 socket 目录失败: " + ec.message();
            return false;
        }
        ::chmod(dir.c_str(), 0775);
        return true;
    }

    void PumpAgent::accept_loop()
    {
        while (!shutdown_requested_)
        {
            const int client_fd = ::accept(server_fd_, nullptr, nullptr);
            if (client_fd < 0)
            {
                if (shutdown_requested_)
                    return;
                continue;
            }
            handle_client(client_fd);
            ::close(client_fd);
        }
    }

    void PumpAgent::handle_client(int client_fd)
    {
        while (!shutdown_requested_)
        {
            PumpMessageType request_type{};
            std::vector<std::uint8_t> payload;
            std::string error;
            if (!read_pump_message(client_fd, &request_type, &payload, &error))
                return;

            PumpMessageType response_type{};
            std::vector<std::uint8_t> response_payload;
            if (!process_request(request_type, payload, &response_type, &response_payload))
            {
                PumpStatusResponse response;
                response.success = false;
                response.agent_ready = true;
                response.message = "unknown request";
                response_payload = serialize_pump_status_response(response);
                response_type = PumpMessageType::kGetStatusResponse;
            }

            if (!write_pump_message(client_fd, response_type, response_payload, &error))
                return;
        }
    }

    bool PumpAgent::process_request(PumpMessageType request_type,
                                    const std::vector<std::uint8_t> &payload,
                                    PumpMessageType *response_type,
                                    std::vector<std::uint8_t> *response_payload)
    {
        PumpStatusResponse response;
        std::string error;

        if (request_type == PumpMessageType::kConfigureRequest)
        {
            PumpConfig config;
            if (!deserialize_pump_config(payload, &config, &error))
            {
                response.success = false;
                response.agent_ready = true;
                response.message = "配置反序列化失败: " + error;
            }
            else
            {
                response = handle_configure(config);
            }
            *response_type = PumpMessageType::kConfigureResponse;
            *response_payload = serialize_pump_status_response(response);
            return true;
        }

        if (request_type == PumpMessageType::kSetStateRequest)
        {
            PumpStateCommand command;
            if (!deserialize_pump_state_command(payload, &command, &error))
            {
                response.success = false;
                response.agent_ready = true;
                response.message = "状态反序列化失败: " + error;
            }
            else
            {
                response = handle_set_state(command);
            }
            *response_type = PumpMessageType::kSetStateResponse;
            *response_payload = serialize_pump_status_response(response);
            return true;
        }

        if (request_type == PumpMessageType::kGetStatusRequest)
        {
            response = handle_get_status();
            *response_type = PumpMessageType::kGetStatusResponse;
            *response_payload = serialize_pump_status_response(response);
            return true;
        }

        return false;
    }

    PumpStatusResponse PumpAgent::handle_configure(const PumpConfig &config)
    {
        PumpStatusResponse response;
        response.agent_ready = true;

        std::string error;
        if (!validate_config(config, &error))
        {
            response.success = false;
            response.message = error;
            return response;
        }

        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!init_gpio_locked(config.relay_gpio, &error))
        {
            response.success = false;
            response.message = error;
            return response;
        }

        runtime_state_.config = config;
        runtime_state_.configured = true;
        set_output_locked(false);
        response = build_status_locked();
        response.success = true;
        response.message = "配置成功";
        return response;
    }

    PumpStatusResponse PumpAgent::handle_set_state(const PumpStateCommand &command)
    {
        PumpStatusResponse response;
        response.agent_ready = true;

        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!runtime_state_.configured)
        {
            response.success = false;
            response.message = "GPIO 尚未配置";
            return response;
        }

        std::string error;
        if (!write_gpio_locked(runtime_state_.config.relay_gpio, command.enable, &error))
        {
            response.success = false;
            response.message = error;
            return response;
        }

        runtime_state_.state = command;
        response = build_status_locked();
        response.success = true;
        response.message = command.enable ? "泵已开启" : "泵已关闭";
        return response;
    }

    PumpStatusResponse PumpAgent::handle_get_status()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return build_status_locked();
    }

    PumpStatusResponse PumpAgent::build_status_locked() const
    {
        PumpStatusResponse response;
        response.success = true;
        response.agent_ready = true;
        response.configured = runtime_state_.configured;
        response.enable = runtime_state_.state.enable;
        response.active_high = runtime_state_.config.active_high;
        response.relay_gpio = runtime_state_.config.relay_gpio;
        response.message = runtime_state_.configured ? "agent ready" : "agent not configured";
        return response;
    }

    bool PumpAgent::validate_config(const PumpConfig &config, std::string *error) const
    {
        if (config.relay_gpio < 0)
        {
            if (error)
                *error = "relay_gpio 非法";
            return false;
        }
        return true;
    }

    bool PumpAgent::init_gpio_locked(int pin, std::string *error)
    {
        if (initialized_gpios_.count(pin) > 0)
            return true;

        const auto dir = gpio_dir(pin);
        if (!std::filesystem::exists(dir))
        {
            if (!write_text_file("/sys/class/gpio/export", std::to_string(pin), error))
                return false;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!write_text_file(gpio_direction_path(pin), "out", error))
            return false;

        initialized_gpios_.insert(pin);
        return true;
    }

    bool PumpAgent::write_gpio_locked(int pin, bool on, std::string *error) const
    {
        if (pin < 0)
            return false;
        const bool value = runtime_state_.config.active_high ? on : !on;
        return write_text_file(gpio_value_path(pin), value ? "1" : "0", error);
    }

    void PumpAgent::set_output_locked(bool on)
    {
        std::string ignored_error;
        write_gpio_locked(runtime_state_.config.relay_gpio, on, &ignored_error);
        runtime_state_.state.enable = on;
    }

} // namespace gas_monitor
