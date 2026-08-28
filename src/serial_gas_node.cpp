#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "gas_monitor/pump_protocol.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class SerialGasNode : public rclcpp::Node
{
public:
    struct AlarmThreshold
    {
        double low{};
        double high{};
    };

    struct PumpRelayState
    {
        bool configured{false};
        bool enabled{false};
        std::string message;
    };

    SerialGasNode() : Node("serial_gas_node")
    {
        serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        baud_rate_ = declare_parameter<int>("baud_rate", 9600);
        parity_ = declare_parameter<std::string>("parity", "none");
        poll_interval_ms_ = declare_parameter<int>("poll_interval_ms", 1000);
        startup_timeout_seconds_ = declare_parameter<int>("startup_timeout_seconds", 120);
        fail_on_startup_timeout_ = declare_parameter<bool>("fail_on_startup_timeout", false);
        inter_request_delay_ms_ = declare_parameter<int>("inter_request_delay_ms", 500);
        response_timeout_ms_ = declare_parameter<int>("response_timeout_ms", 1000);
        alarm_repeat_seconds_ = declare_parameter<int>("alarm_repeat_seconds", 10);
        test_alarm_hold_seconds_ = declare_parameter<int>("test_alarm_hold_seconds", 5);
        max_retries_per_slave_ = declare_parameter<int>("max_retries_per_slave", 3);
        use_config_alarm_thresholds_ = declare_parameter<bool>("use_config_alarm_thresholds", false);
        pump_relay_enable_ = declare_parameter<bool>("pump_relay_enable", false);
        pump_relay_gpio_ = declare_parameter<int>("pump_relay_gpio", -1);
        pump_relay_active_high_ = declare_parameter<bool>("pump_relay_active_high", true);
        pump_relay_socket_path_ = declare_parameter<std::string>("pump_relay_socket_path", gas_monitor::kDefaultPumpSocketPath);

        const auto slave_ids_raw = declare_parameter<std::vector<int64_t>>("slave_ids", {1});
        for (const auto sid : slave_ids_raw)
            slave_ids_.push_back(static_cast<int>(sid));

        const auto threshold_ids_raw = declare_parameter<std::vector<int64_t>>("alarm_threshold_slave_ids", std::vector<int64_t>{});
        const auto low_alarm_raw = declare_parameter<std::vector<double>>("low_alarm_overrides", std::vector<double>{});
        const auto high_alarm_raw = declare_parameter<std::vector<double>>("high_alarm_overrides", std::vector<double>{});
        const auto gas_type_names_raw = declare_parameter<std::vector<std::string>>("gas_type_overrides", std::vector<std::string>{});
        alarm_threshold_slave_ids_.reserve(threshold_ids_raw.size());
        for (const auto sid : threshold_ids_raw)
            alarm_threshold_slave_ids_.push_back(static_cast<int>(sid));
        low_alarm_overrides_ = low_alarm_raw;
        high_alarm_overrides_ = high_alarm_raw;
        gas_type_overrides_names_ = gas_type_names_raw;
        if (threshold_ids_raw.size() != low_alarm_raw.size() || threshold_ids_raw.size() != high_alarm_raw.size())
        {
            RCLCPP_WARN(get_logger(), "报警阈值覆盖参数长度不一致，将忽略配置阈值。");
            use_config_alarm_thresholds_ = false;
        }
        else
        {
            for (size_t i = 0; i < threshold_ids_raw.size(); ++i)
                threshold_overrides_[static_cast<int>(threshold_ids_raw[i])] = AlarmThreshold{low_alarm_raw[i], high_alarm_raw[i]};
        }

        if (!gas_type_names_raw.empty() && gas_type_names_raw.size() != threshold_ids_raw.size())
        {
            RCLCPP_WARN(get_logger(), "气体类型覆盖参数长度与alarm_threshold_slave_ids不一致，将忽略气体类型覆盖。");
        }
        else
        {
            for (size_t i = 0; i < gas_type_names_raw.size(); ++i)
            {
                const int gas_type_code = gas_code_from_name(gas_type_names_raw[i]);
                if (gas_type_code < 0)
                {
                    RCLCPP_WARN(get_logger(), "未知气体类型覆盖配置：slave_id=%ld gas=%s，将忽略该项。",
                                static_cast<long>(threshold_ids_raw[i]), gas_type_names_raw[i].c_str());
                    continue;
                }
                gas_type_overrides_[static_cast<int>(threshold_ids_raw[i])] = gas_type_code;
            }
        }

        status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/monitor/gas/status", 10);
        start_srv_ = create_service<std_srvs::srv::Trigger>("/monitor/gas/start", std::bind(&SerialGasNode::on_start, this, std::placeholders::_1, std::placeholders::_2));
        stop_srv_ = create_service<std_srvs::srv::Trigger>("/monitor/gas/stop", std::bind(&SerialGasNode::on_stop, this, std::placeholders::_1, std::placeholders::_2));
        test_alarm_srv_ = create_service<std_srvs::srv::Trigger>("/monitor/gas/test_alarm", std::bind(&SerialGasNode::on_test_alarm, this, std::placeholders::_1, std::placeholders::_2));
        set_parameters_srv_ = create_service<rcl_interfaces::srv::SetParameters>("/monitor/gas/set_parameters", std::bind(&SerialGasNode::on_set_parameters, this, std::placeholders::_1, std::placeholders::_2));

        publish_status(default_readings(), diagnostic_msgs::msg::DiagnosticStatus::STALE, "气体传感器未启动");
        RCLCPP_INFO(get_logger(), "气体传感器服务已就绪：start=/monitor/gas/start stop=/monitor/gas/stop set_parameters=/monitor/gas/set_parameters status=/monitor/gas/status 串口=%s 站号=%s 阈值覆盖=%s 气体类型覆盖=%s 泵继电器=%s",
                    serial_port_.c_str(),
                    join_ints(slave_ids_).c_str(),
                    use_config_alarm_thresholds_ ? "开启" : "关闭",
                    use_config_alarm_thresholds_ && !gas_type_overrides_.empty() ? "开启" : "关闭",
                    pump_relay_enable_ ? "开启" : "关闭");
    }

    ~SerialGasNode() override
    {
        monitoring_active_ = false;
        if (polling_thread_.joinable())
            polling_thread_.join();
        disconnect_pump_socket();
    }

private:
    struct GasSensorReading
    {
        int id{};
        bool valid{};
        std::string gas;
        int gas_type_code{};
        double concentration{};
        std::string unit;
        double low_alarm{};
        double high_alarm{};
        double full_scale{};
        int status_code{};
        std::string status;
        int ad_value{};
        double temp{};
        double humidity{};
        int unit_code{-1};
        int decimal_code{-1};
        int decimal_places{-1};
        std::vector<uint16_t> registers;
        std::string raw_frame_hex;
        std::string error;
    };

    struct AlarmSummary
    {
        bool active{false};
        std::string type;
        std::string detail;
    };

    static constexpr const char *kLogRed = "\033[31m";
    static constexpr const char *kLogYellow = "\033[33m";
    static constexpr const char *kLogGreen = "\033[32m";
    static constexpr const char *kLogReset = "\033[0m";
    static constexpr int kRegisterCount = 10;
    static constexpr size_t kResponseSize = 25;

    void on_start(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(thread_mutex_);
        if (monitoring_active_)
        {
            response->success = true;
            response->message = "气体传感器已在运行";
            return;
        }

        if (polling_thread_.joinable())
            polling_thread_.join();

        if (pump_relay_enable_)
        {
            const auto pump_config = configure_pump_relay();
            if (!pump_config.success)
            {
                response->success = false;
                response->message = "泵继电器配置失败: " + pump_config.message;
                publish_status(default_readings(response->message), diagnostic_msgs::msg::DiagnosticStatus::ERROR, response->message);
                return;
            }
            const auto pump_on = set_pump_enabled(true);
            if (!pump_on.success)
            {
                response->success = false;
                response->message = "泵继电器开启失败: " + pump_on.message;
                publish_status(default_readings(response->message), diagnostic_msgs::msg::DiagnosticStatus::ERROR, response->message);
                return;
            }
        }

        std::string probe_message;
        std::vector<GasSensorReading> probe_readings;
        if (!probe_sensors(probe_readings, probe_message))
        {
            if (pump_relay_enable_)
            {
                const auto pump_off = set_pump_enabled(false);
                if (!pump_off.success)
                    RCLCPP_WARN(get_logger(), "探测失败后关闭泵继电器失败：%s", pump_off.message.c_str());
            }
            response->success = false;
            response->message = probe_message;
            publish_status(probe_readings.empty() ? default_readings(probe_message) : probe_readings,
                           diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                           probe_message);
            return;
        }

        monitoring_active_ = true;
        last_status_codes_.clear();
        last_alarm_times_.clear();
        polling_thread_ = std::thread(&SerialGasNode::polling_loop, this);

        response->success = true;
        response->message = "气体传感器已启动";
    }

    void on_stop(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        {
            std::lock_guard<std::mutex> lock(thread_mutex_);
            if (!monitoring_active_)
            {
                bool pump_ok = true;
                std::string pump_message;
                if (pump_relay_enable_)
                {
                    const auto pump_off = set_pump_enabled(false);
                    pump_ok = pump_off.success;
                    pump_message = pump_off.message;
                }
                response->success = pump_ok;
                response->message = pump_ok ? "气体传感器已停止" : "气体传感器已停止，但关闭泵继电器失败: " + pump_message;
                publish_status(default_readings(), diagnostic_msgs::msg::DiagnosticStatus::STALE, "气体传感器已停止");
                return;
            }
            monitoring_active_ = false;
        }

        if (polling_thread_.joinable())
            polling_thread_.join();

        bool pump_ok = true;
        std::string pump_message;
        if (pump_relay_enable_)
        {
            const auto pump_off = set_pump_enabled(false);
            pump_ok = pump_off.success;
            pump_message = pump_off.message;
        }

        response->success = pump_ok;
        response->message = pump_ok ? "气体传感器已停止" : "气体传感器已停止，但关闭泵继电器失败: " + pump_message;
        publish_status(default_readings(), diagnostic_msgs::msg::DiagnosticStatus::STALE, "气体传感器已停止");
    }

    void on_test_alarm(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!monitoring_active_)
        {
            response->success = false;
            response->message = "气体传感器未启动，无法触发测试报警";
            return;
        }

        std::vector<GasSensorReading> readings;
        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            last_manual_test_alarm_until_ = std::chrono::steady_clock::now() + std::chrono::seconds(std::max(1, test_alarm_hold_seconds_));
            readings = last_readings_.empty() ? default_readings("测试报警") : last_readings_;
        }

        AlarmSummary alarm;
        apply_manual_test_alarm(alarm);
        publish_status(readings, diagnostic_msgs::msg::DiagnosticStatus::ERROR, alarm.detail);
        RCLCPP_WARN(get_logger(), "%s[气体] 已触发测试报警：level=ERROR(2) 持续=%d秒%s",
                    kLogRed, std::max(1, test_alarm_hold_seconds_), kLogReset);

        response->success = true;
        response->message = "已触发气体传感器测试报警";
    }

    void on_set_parameters(const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request,
                           std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> response)
    {
        std::lock_guard<std::mutex> update_lock(parameter_update_mutex_);
        auto fail = [&](const std::string &reason)
        {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = false;
            result.reason = reason;
            response->results.assign(request->parameters.size(), result);
        };

        std::lock_guard<std::mutex> monitoring_lock(thread_mutex_);
        if (!monitoring_active_)
        {
            fail("气体传感器未启动，无法修改参数，请先调用 /monitor/gas/start");
            return;
        }

        if (request->parameters.empty())
        {
            fail("至少需要提供一个参数");
            return;
        }

        bool use_config_alarm_thresholds = false;
        std::vector<int> threshold_ids;
        std::vector<double> low_alarms;
        std::vector<double> high_alarms;
        std::vector<std::string> gas_names;
        {
            std::lock_guard<std::mutex> lock(config_mutex_);
            use_config_alarm_thresholds = use_config_alarm_thresholds_;
            threshold_ids = alarm_threshold_slave_ids_;
            low_alarms = low_alarm_overrides_;
            high_alarms = high_alarm_overrides_;
            gas_names = gas_type_overrides_names_;
        }

        bool has_use_config = false;
        bool has_ids = false;
        bool has_low = false;
        bool has_high = false;
        bool has_gas_names = false;
        for (const auto &parameter : request->parameters)
        {
            if (parameter.name == "use_config_alarm_thresholds")
            {
                if (has_use_config)
                {
                    fail("参数 use_config_alarm_thresholds 重复提供");
                    return;
                }
                if (parameter.value.type != rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
                {
                    fail("参数 use_config_alarm_thresholds 必须为布尔值");
                    return;
                }
                has_use_config = true;
                use_config_alarm_thresholds = parameter.value.bool_value;
            }
            else if (parameter.name == "alarm_threshold_slave_ids")
            {
                if (has_ids)
                {
                    fail("参数 alarm_threshold_slave_ids 重复提供");
                    return;
                }
                if (parameter.value.type != rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY)
                {
                    fail("参数 alarm_threshold_slave_ids 必须为整数数组");
                    return;
                }
                has_ids = true;
                threshold_ids.clear();
                for (const auto value : parameter.value.integer_array_value)
                {
                    if (value < 1 || value > 247)
                    {
                        fail("参数 alarm_threshold_slave_ids 中的站号必须在 1 到 247 之间");
                        return;
                    }
                    threshold_ids.push_back(static_cast<int>(value));
                }
            }
            else if (parameter.name == "low_alarm_overrides")
            {
                if (has_low)
                {
                    fail("参数 low_alarm_overrides 重复提供");
                    return;
                }
                if (parameter.value.type != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY)
                {
                    fail("参数 low_alarm_overrides 必须为浮点数组");
                    return;
                }
                has_low = true;
                low_alarms.assign(parameter.value.double_array_value.begin(), parameter.value.double_array_value.end());
            }
            else if (parameter.name == "high_alarm_overrides")
            {
                if (has_high)
                {
                    fail("参数 high_alarm_overrides 重复提供");
                    return;
                }
                if (parameter.value.type != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY)
                {
                    fail("参数 high_alarm_overrides 必须为浮点数组");
                    return;
                }
                has_high = true;
                high_alarms.assign(parameter.value.double_array_value.begin(), parameter.value.double_array_value.end());
            }
            else if (parameter.name == "gas_type_overrides")
            {
                if (has_gas_names)
                {
                    fail("参数 gas_type_overrides 重复提供");
                    return;
                }
                if (parameter.value.type != rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY)
                {
                    fail("参数 gas_type_overrides 必须为字符串数组");
                    return;
                }
                has_gas_names = true;
                gas_names.assign(parameter.value.string_array_value.begin(), parameter.value.string_array_value.end());
            }
            else
            {
                fail("不支持的参数：" + parameter.name);
                return;
            }
        }

        if (threshold_ids.size() != low_alarms.size() || threshold_ids.size() != high_alarms.size())
        {
            fail("alarm_threshold_slave_ids、low_alarm_overrides、high_alarm_overrides 的长度必须一致");
            return;
        }
        if (!gas_names.empty() && gas_names.size() != threshold_ids.size())
        {
            fail("gas_type_overrides 非空时，长度必须与 alarm_threshold_slave_ids 一致");
            return;
        }

        std::map<int, AlarmThreshold> threshold_overrides;
        std::map<int, int> gas_type_overrides;
        for (size_t i = 0; i < threshold_ids.size(); ++i)
        {
            if (threshold_overrides.find(threshold_ids[i]) != threshold_overrides.end())
            {
                fail("alarm_threshold_slave_ids 中存在重复站号：" + std::to_string(threshold_ids[i]));
                return;
            }
            if (!std::isfinite(low_alarms[i]) || !std::isfinite(high_alarms[i]) || low_alarms[i] < -1.0 || high_alarms[i] < -1.0)
            {
                fail("low_alarm_overrides 和 high_alarm_overrides 必须为有限数值，且不能小于 -1（-1 表示禁用该级报警）");
                return;
            }
            if (low_alarms[i] >= 0.0 && high_alarms[i] >= 0.0 && high_alarms[i] < low_alarms[i])
            {
                fail("每个传感器的 high_alarm_overrides 必须大于或等于 low_alarm_overrides");
                return;
            }
            threshold_overrides[threshold_ids[i]] = AlarmThreshold{low_alarms[i], high_alarms[i]};

            if (!gas_names.empty())
            {
                const int gas_type_code = gas_code_from_name(gas_names[i]);
                if (gas_type_code < 0)
                {
                    fail("未知气体类型：" + gas_names[i]);
                    return;
                }
                gas_type_overrides[threshold_ids[i]] = gas_type_code;
            }
        }

        {
            std::lock_guard<std::mutex> lock(config_mutex_);
            use_config_alarm_thresholds_ = use_config_alarm_thresholds;
            alarm_threshold_slave_ids_ = std::move(threshold_ids);
            low_alarm_overrides_ = std::move(low_alarms);
            high_alarm_overrides_ = std::move(high_alarms);
            gas_type_overrides_names_ = std::move(gas_names);
            threshold_overrides_ = std::move(threshold_overrides);
            gas_type_overrides_ = std::move(gas_type_overrides);
        }

        rcl_interfaces::msg::SetParametersResult success;
        success.successful = true;
        success.reason = "气体传感器参数设置成功";
        response->results.assign(request->parameters.size(), success);
    }

    static uint16_t calculate_crc16(const uint8_t *data, size_t length)
    {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < length; ++i)
        {
            crc ^= data[i];
            for (int bit = 0; bit < 8; ++bit)
                crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001) : static_cast<uint16_t>(crc >> 1);
        }
        return crc;
    }

    static std::vector<uint8_t> build_modbus_request(int slave_id)
    {
        std::vector<uint8_t> packet{static_cast<uint8_t>(slave_id), 0x03, 0x00, 0x00, 0x00, static_cast<uint8_t>(kRegisterCount)};
        const uint16_t crc = calculate_crc16(packet.data(), packet.size());
        packet.push_back(static_cast<uint8_t>(crc & 0xFF));
        packet.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
        return packet;
    }

    static std::string to_hex(const uint8_t *data, size_t length)
    {
        std::ostringstream oss;
        for (size_t i = 0; i < length; ++i)
        {
            if (i)
                oss << ' ';
            oss << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(data[i]);
        }
        return oss.str();
    }

    static std::string to_hex(const std::vector<uint8_t> &data)
    {
        return data.empty() ? "" : to_hex(data.data(), data.size());
    }

    static uint16_t register_value(const std::vector<uint8_t> &frame, size_t index)
    {
        const size_t offset = 3 + index * 2;
        return (static_cast<uint16_t>(frame[offset]) << 8) | frame[offset + 1];
    }

    speed_t to_speed_t(int baud_rate) const
    {
        switch (baud_rate)
        {
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        default:
            RCLCPP_WARN(get_logger(), "不支持波特率 %d，将使用 9600。", baud_rate);
            return B9600;
        }
    }

    int open_serial_port() const
    {
        const int fd = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd == -1)
        {
            RCLCPP_ERROR(get_logger(), "无法打开串口 %s：%s", serial_port_.c_str(), strerror(errno));
            return -1;
        }
        if (fcntl(fd, F_SETFL, 0) == -1)
        {
            RCLCPP_ERROR(get_logger(), "无法将串口 %s 设置为阻塞模式：%s", serial_port_.c_str(), strerror(errno));
            close(fd);
            return -1;
        }

        struct termios options{};
        if (tcgetattr(fd, &options) != 0)
        {
            RCLCPP_ERROR(get_logger(), "读取串口 %s 参数失败：%s", serial_port_.c_str(), strerror(errno));
            close(fd);
            return -1;
        }

        cfmakeraw(&options);
        const auto speed = to_speed_t(baud_rate_);
        cfsetispeed(&options, speed);
        cfsetospeed(&options, speed);
        options.c_cflag |= CLOCAL | CREAD;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag &= ~CRTSCTS;
        options.c_cflag &= ~CSTOPB;

        if (parity_ == "even")
        {
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
        }
        else if (parity_ == "odd")
        {
            options.c_cflag |= PARENB;
            options.c_cflag |= PARODD;
        }
        else
        {
            options.c_cflag &= ~PARENB;
        }

        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 1;
        if (tcsetattr(fd, TCSANOW, &options) != 0)
        {
            RCLCPP_ERROR(get_logger(), "设置串口 %s 参数失败：%s", serial_port_.c_str(), strerror(errno));
            close(fd);
            return -1;
        }
        tcflush(fd, TCIOFLUSH);
        return fd;
    }

    std::vector<uint8_t> read_response_frame(int fd, bool require_active = true)
    {
        std::vector<uint8_t> frame;
        frame.reserve(kResponseSize);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(response_timeout_ms_);

        while (rclcpp::ok() && (!require_active || monitoring_active_) && std::chrono::steady_clock::now() < deadline)
        {
            uint8_t buffer[256]{};
            const ssize_t bytes_read = read(fd, buffer, sizeof(buffer));
            if (bytes_read > 0)
            {
                frame.insert(frame.end(), buffer, buffer + bytes_read);
                if (frame.size() >= kResponseSize)
                    break;
            }
            else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
            {
                RCLCPP_WARN(get_logger(), "[串口] 读取失败：%s", strerror(errno));
                break;
            }
        }
        return frame;
    }

    static std::string frame_error(int slave_id, const std::vector<uint8_t> &frame)
    {
        if (frame.empty())
            return "未收到数据";
        if (frame.size() < 5)
            return "应答长度不足";
        if (frame[0] != static_cast<uint8_t>(slave_id))
            return "站号不匹配";
        if (frame[1] & 0x80)
            return "设备返回Modbus异常码=" + std::to_string(frame[2]);
        if (frame[1] != 0x03)
            return "功能码不匹配";
        if (frame[2] != kRegisterCount * 2)
            return "有效字节数不匹配";
        if (frame.size() < kResponseSize)
            return "应答长度不足";

        const uint16_t received_crc = static_cast<uint16_t>(frame[kResponseSize - 2]) | (static_cast<uint16_t>(frame[kResponseSize - 1]) << 8);
        if (calculate_crc16(frame.data(), kResponseSize - 2) != received_crc)
            return "CRC校验失败";
        return "";
    }

    std::string gas_name(int code) const
    {
        const auto it = gas_type_map_.find(code);
        return it == gas_type_map_.end() ? "未知" : it->second;
    }

    int gas_code_from_name(const std::string &name) const
    {
        for (const auto &[code, gas_name_value] : gas_type_map_)
        {
            if (gas_name_value == name)
                return code;
        }
        return -1;
    }

    std::string sensor_status(int code) const
    {
        const auto it = sensor_status_map_.find(code);
        return it == sensor_status_map_.end() ? "未知状态" : it->second;
    }

    static bool audible_status(int status_code)
    {
        return status_code == 4 || status_code == 5 || status_code == 6;
    }

    static std::string alarm_level(int status_code)
    {
        if (status_code == 4)
            return "warning";
        if (status_code == 5)
            return "low_alarm";
        if (status_code == 6)
            return "high_alarm";
        if (status_code == 1)
            return "normal";
        return "fault";
    }

    static int alarm_priority(int status_code)
    {
        switch (status_code)
        {
        case 6:
            return 4;
        case 5:
            return 3;
        case 4:
            return 2;
        case 1:
            return 0;
        default:
            return 1;
        }
    }

    GasSensorReading default_sensor_reading(int slave_id, const std::string &error = "通信失败") const
    {
        GasSensorReading msg;
        msg.id = slave_id;
        msg.valid = false;
        msg.gas = "未知";
        msg.gas_type_code = -1;
        msg.concentration = -1.0;
        msg.unit = "N/A";
        msg.low_alarm = -1.0;
        msg.high_alarm = -1.0;
        msg.full_scale = -1.0;
        msg.status_code = -1;
        msg.status = error;
        msg.ad_value = -1;
        msg.temp = -1.0;
        msg.humidity = -1.0;
        msg.error = error;
        return msg;
    }

    std::vector<GasSensorReading> default_readings(const std::string &error = "通信失败") const
    {
        std::vector<GasSensorReading> readings;
        for (const int sid : slave_ids_)
            readings.push_back(default_sensor_reading(sid, error));
        return readings;
    }

    bool parse_register_payload(int slave_id, const std::vector<uint8_t> &frame, GasSensorReading &msg)
    {
        std::lock_guard<std::mutex> config_lock(config_mutex_);
        const std::string error = frame_error(slave_id, frame);
        msg.raw_frame_hex = to_hex(frame);
        if (!error.empty())
        {
            msg.error = error;
            return false;
        }

        msg.registers.resize(kRegisterCount);
        for (size_t i = 0; i < kRegisterCount; ++i)
            msg.registers[i] = register_value(frame, i);

        msg.unit_code = (msg.registers[0] >> 12) & 0x0F;
        msg.decimal_code = (msg.registers[0] >> 8) & 0x0F;
        msg.decimal_places = msg.decimal_code == 4 ? 1 : msg.decimal_code == 8 ? 2
                                                     : msg.decimal_code == 12  ? 3
                                                                               : 0;
        const double divisor = std::pow(10, msg.decimal_places);
        msg.unit = msg.unit_code == 2 ? "%LEL" : msg.unit_code == 4 ? "%VOL"
                                             : msg.unit_code == 6   ? "mg/m3"
                                             : msg.unit_code == 8   ? "ppb"
                                                                    : "ppm";
        msg.id = slave_id;
        msg.valid = true;
        msg.gas_type_code = (msg.registers[8] >> 8) & 0xFF;
        apply_config_gas_type_override(msg);
        msg.status_code = msg.registers[5] & 0xFF;
        msg.gas = gas_name(msg.gas_type_code);
        msg.concentration = static_cast<double>(msg.registers[1]) / divisor;
        msg.low_alarm = static_cast<double>(msg.registers[2]) / divisor;
        msg.high_alarm = static_cast<double>(msg.registers[3]) / divisor;
        apply_config_threshold_override(msg);
        msg.full_scale = static_cast<double>(msg.registers[4]) / divisor;
        msg.status = sensor_status(msg.status_code);
        msg.ad_value = msg.registers[6];
        msg.temp = static_cast<double>(msg.registers[7]) / 10.0;
        msg.humidity = static_cast<double>(msg.registers[9]) / 10.0;
        msg.error.clear();

        // RCLCPP_INFO(get_logger(), "[气体] 地址=%d 气体=%s(%d) 浓度=%.3f%s 低报=%.3f 高报=%.3f 状态=%s(0x%02X) AD=%d 温度=%.1f°C 湿度=%.1f%%RH",
        //             slave_id, msg.gas.c_str(), msg.gas_type_code, msg.concentration, msg.unit.c_str(),
        //             msg.low_alarm, msg.high_alarm, msg.status.c_str(), msg.status_code, msg.ad_value, msg.temp, msg.humidity);
        return true;
    }

    void apply_config_threshold_override(GasSensorReading &reading) const
    {
        if (!use_config_alarm_thresholds_)
            return;
        const auto it = threshold_overrides_.find(reading.id);
        if (it == threshold_overrides_.end())
            return;
        reading.low_alarm = it->second.low;
        reading.high_alarm = it->second.high;
        if (reading.high_alarm >= 0.0 && reading.concentration >= reading.high_alarm)
            reading.status_code = 6;
        else if (reading.low_alarm >= 0.0 && reading.concentration >= reading.low_alarm)
            reading.status_code = 5;
        else
            reading.status_code = 1;
        reading.status = sensor_status(reading.status_code);
    }

    void apply_config_gas_type_override(GasSensorReading &reading) const
    {
        if (!use_config_alarm_thresholds_)
            return;
        const auto it = gas_type_overrides_.find(reading.id);
        if (it == gas_type_overrides_.end())
            return;
        reading.gas_type_code = it->second;
    }

    bool read_and_parse_sensor(int fd, int slave_id, GasSensorReading &msg, bool require_active = true)
    {
        const auto request = build_modbus_request(slave_id);
        for (int attempt = 1; attempt <= std::max(1, max_retries_per_slave_); ++attempt)
        {
            tcflush(fd, TCIFLUSH);
            const ssize_t written = write(fd, request.data(), request.size());
            if (written != static_cast<ssize_t>(request.size()))
            {
                RCLCPP_WARN(get_logger(), "[串口] 地址=%d 第%d次发送失败，已写入=%zd，错误=%s", slave_id, attempt, written, strerror(errno));
                msg = default_sensor_reading(slave_id, "发送失败");
                msg.error = strerror(errno);
                continue;
            }
            if (tcdrain(fd) != 0)
            {
                RCLCPP_WARN(get_logger(), "[串口] 地址=%d 第%d次等待发送完成失败：%s", slave_id, attempt, strerror(errno));
                msg = default_sensor_reading(slave_id, "等待发送完成失败");
                msg.error = strerror(errno);
                continue;
            }
            const auto frame = read_response_frame(fd, require_active);
            msg = default_sensor_reading(slave_id);
            msg.raw_frame_hex = to_hex(frame);
            const std::string error = frame_error(slave_id, frame);
            if (error.empty() && parse_register_payload(slave_id, frame, msg))
                return true;

            msg.error = error;
            RCLCPP_WARN(get_logger(), "[串口] 地址=%d 第%d次应答无效：%s，长度=%zu，数据=%s", slave_id, attempt, error.c_str(), frame.size(), to_hex(frame).c_str());
            std::this_thread::sleep_for(std::chrono::milliseconds(inter_request_delay_ms_));
        }
        return false;
    }

    bool probe_sensors(std::vector<GasSensorReading> &readings, std::string &message)
    {
        readings.clear();

        const int fd = open_serial_port();
        if (fd == -1)
        {
            message = "无法打开气体传感器串口";
            return false;
        }

        int valid_count = 0;
        for (const int sid : slave_ids_)
        {
            GasSensorReading reading;
            if (read_and_parse_sensor(fd, sid, reading, false))
                ++valid_count;
            readings.push_back(reading);
            std::this_thread::sleep_for(std::chrono::milliseconds(inter_request_delay_ms_));
        }
        close(fd);

        if (valid_count == 0)
        {
            message = "未读取到任何有效气体传感器数据";
            return false;
        }

        if (valid_count < static_cast<int>(slave_ids_.size()))
        {
            message = "部分气体传感器可用，已启动轮询";
        }
        else
        {
            message = "气体传感器已启动";
        }
        return true;
    }

    gas_monitor::PumpStatusResponse configure_pump_relay()
    {
        gas_monitor::PumpConfig config;
        config.relay_gpio = pump_relay_gpio_;
        config.active_high = pump_relay_active_high_;
        return send_pump_request(gas_monitor::PumpMessageType::kConfigureRequest,
                                 gas_monitor::serialize_pump_config(config),
                                 gas_monitor::PumpMessageType::kConfigureResponse);
    }

    gas_monitor::PumpStatusResponse set_pump_enabled(bool enable)
    {
        gas_monitor::PumpStateCommand command;
        command.enable = enable;
        auto response = send_pump_request(gas_monitor::PumpMessageType::kSetStateRequest,
                                          gas_monitor::serialize_pump_state_command(command),
                                          gas_monitor::PumpMessageType::kSetStateResponse);
        if (response.success)
        {
            pump_relay_state_.configured = response.configured;
            pump_relay_state_.enabled = response.enable;
            pump_relay_state_.message = response.message;
            RCLCPP_INFO(get_logger(), "泵继电器%s：gpio=%d active_high=%s socket=%s",
                        enable ? "已开启" : "已关闭",
                        pump_relay_gpio_,
                        pump_relay_active_high_ ? "true" : "false",
                        pump_relay_socket_path_.c_str());
        }
        return response;
    }

    gas_monitor::PumpStatusResponse send_pump_request(gas_monitor::PumpMessageType request_type,
                                                      const std::vector<std::uint8_t> &payload,
                                                      gas_monitor::PumpMessageType expected_response)
    {
        gas_monitor::PumpStatusResponse failure;
        failure.success = false;
        failure.agent_ready = false;
        failure.relay_gpio = pump_relay_gpio_;
        failure.active_high = pump_relay_active_high_;

        std::lock_guard<std::mutex> lock(pump_socket_mutex_);
        const int fd = ensure_pump_connected_locked();
        if (fd < 0)
        {
            failure.message = "未连接到 gas_monitor pump agent，请先启动 systemd 守护进程";
            return failure;
        }

        std::string error;
        if (!gas_monitor::write_pump_message(fd, request_type, payload, &error))
        {
            disconnect_pump_socket_locked();
            failure.message = "发送 IPC 请求失败: " + error;
            return failure;
        }

        gas_monitor::PumpMessageType response_type{};
        std::vector<std::uint8_t> response_payload;
        if (!gas_monitor::read_pump_message(fd, &response_type, &response_payload, &error))
        {
            disconnect_pump_socket_locked();
            failure.message = "读取 IPC 响应失败: " + error;
            return failure;
        }
        if (response_type != expected_response)
        {
            failure.message = "收到意外响应类型";
            return failure;
        }

        gas_monitor::PumpStatusResponse response;
        if (!gas_monitor::deserialize_pump_status_response(response_payload, &response, &error))
        {
            failure.message = "解析 IPC 响应失败: " + error;
            return failure;
        }
        return response;
    }

    int ensure_pump_connected_locked()
    {
        if (pump_socket_fd_ >= 0)
            return pump_socket_fd_;

        const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
        if (fd < 0)
            return -1;

        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", pump_relay_socket_path_.c_str());
        if (::connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
        {
            ::close(fd);
            return -1;
        }

        pump_socket_fd_ = fd;
        RCLCPP_INFO(get_logger(), "已连接到 gas_monitor pump agent: %s", pump_relay_socket_path_.c_str());
        return pump_socket_fd_;
    }

    void disconnect_pump_socket()
    {
        std::lock_guard<std::mutex> lock(pump_socket_mutex_);
        disconnect_pump_socket_locked();
    }

    void disconnect_pump_socket_locked()
    {
        if (pump_socket_fd_ >= 0)
        {
            ::close(pump_socket_fd_);
            pump_socket_fd_ = -1;
        }
    }

    bool manual_test_alarm_active() const
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        return std::chrono::steady_clock::now() < last_manual_test_alarm_until_;
    }

    void apply_manual_test_alarm(AlarmSummary &alarm) const
    {
        alarm.active = true;
        alarm.type = "gas_test_alarm";
        alarm.detail = "气体传感器测试报警";
    }

    void update_alarm_state(const GasSensorReading &reading, AlarmSummary &summary)
    {
        const auto now_tp = std::chrono::steady_clock::now();
        const auto status_it = last_status_codes_.find(reading.id);
        const bool status_changed = status_it == last_status_codes_.end() || status_it->second != reading.status_code;
        const auto repeat_it = last_alarm_times_.find(reading.id);
        const bool repeat_due = repeat_it == last_alarm_times_.end() || std::chrono::duration_cast<std::chrono::seconds>(now_tp - repeat_it->second).count() >= alarm_repeat_seconds_;

        const std::string level = alarm_level(reading.status_code);
        if (reading.status_code != 1)
        {
            const bool should_replace = !summary.active || alarm_priority(reading.status_code) >= alarm_priority(last_summary_status_code_);
            if (should_replace)
            {
                summary.active = true;
                summary.type = "gas_" + level;
                summary.detail = "气体传感器异常：地址=" + std::to_string(reading.id) + " 气体=" + reading.gas +
                                 " 状态=" + reading.status + " 浓度=" + number(reading.concentration) + reading.unit;
                last_summary_status_code_ = reading.status_code;
            }
        }

        if ((status_changed && (reading.status_code != 1 || status_it != last_status_codes_.end())) || (audible_status(reading.status_code) && repeat_due))
        {
            last_alarm_times_[reading.id] = now_tp;
            RCLCPP_INFO(get_logger(), "%s[气体] 地址=%d 气体=%s 状态=%s(0x%02X) 级别=%s 音频=%s%s",
                        summary.active ? (audible_status(reading.status_code) ? kLogRed : kLogYellow) : kLogGreen,
                        reading.id, reading.gas.c_str(), reading.status.c_str(), reading.status_code, level.c_str(), audible_status(reading.status_code) ? "是" : "否", kLogReset);
        }
        last_status_codes_[reading.id] = reading.status_code;
    }

    static diagnostic_msgs::msg::KeyValue kv(const std::string &key, const std::string &value)
    {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = value;
        return item;
    }

    static std::string number(double value)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3) << value;
        return oss.str();
    }

    static std::string join_ints(const std::vector<int> &items)
    {
        std::ostringstream oss;
        for (size_t i = 0; i < items.size(); ++i)
        {
            if (i)
                oss << ",";
            oss << items[i];
        }
        return oss.str();
    }

    uint8_t aggregate_level(const std::vector<GasSensorReading> &readings) const
    {
        uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        for (const auto &reading : readings)
        {
            if (!reading.valid || reading.status_code < 0 || reading.status_code == 3 || reading.status_code == 5 || reading.status_code == 6 ||
                reading.status_code == 7 || reading.status_code == 8 || reading.status_code == 10 || reading.status_code == 11 ||
                reading.status_code == 12 || reading.status_code == 15)
            {
                return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            }
            if (reading.status_code != 1)
                level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        }
        return level;
    }

    std::vector<diagnostic_msgs::msg::KeyValue> build_values(const std::vector<GasSensorReading> &readings) const
    {
        std::vector<diagnostic_msgs::msg::KeyValue> values;
        values.push_back(kv("sensor_count", std::to_string(readings.size())));
        values.push_back(kv("sensor_ids", join_ints(slave_ids_)));

        for (const auto &reading : readings)
        {
            const std::string prefix = "sensor_" + std::to_string(reading.id) + ".";
            values.push_back(kv(prefix + "id", std::to_string(reading.id)));
            values.push_back(kv(prefix + "valid", reading.valid ? "true" : "false"));
            values.push_back(kv(prefix + "gas", reading.gas));
            values.push_back(kv(prefix + "gas_type_code", std::to_string(reading.gas_type_code)));
            values.push_back(kv(prefix + "concentration", number(reading.concentration)));
            values.push_back(kv(prefix + "unit", reading.unit));
            values.push_back(kv(prefix + "low_alarm", number(reading.low_alarm)));
            values.push_back(kv(prefix + "high_alarm", number(reading.high_alarm)));
            values.push_back(kv(prefix + "full_scale", number(reading.full_scale)));
            values.push_back(kv(prefix + "status_code", std::to_string(reading.status_code)));
            values.push_back(kv(prefix + "status", reading.status));
            values.push_back(kv(prefix + "temp", number(reading.temp)));
            values.push_back(kv(prefix + "humidity", number(reading.humidity)));
            values.push_back(kv(prefix + "error", reading.error));
        }
        return values;
    }

    void publish_status(const std::vector<GasSensorReading> &readings, uint8_t level, const std::string &message)
    {
        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            last_readings_ = readings;
        }

        diagnostic_msgs::msg::DiagnosticStatus status;
        status.level = level;
        status.name = "gas_sensor";
        status.message = message;
        status.hardware_id = serial_port_;
        status.values = build_values(readings);
        status_pub_->publish(status);
    }

    void polling_loop()
    {
        const auto start_time = std::chrono::steady_clock::now();
        bool startup_check_done = false;
        bool has_valid_data = false;

        while (rclcpp::ok() && monitoring_active_)
        {
            const int fd = open_serial_port();
            if (fd == -1)
            {
                publish_status(default_readings("串口打开失败"), diagnostic_msgs::msg::DiagnosticStatus::ERROR, "无法打开气体传感器串口");
                std::this_thread::sleep_for(2s);
                continue;
            }

            while (rclcpp::ok() && monitoring_active_)
            {
                std::vector<GasSensorReading> readings;
                AlarmSummary alarm;
                last_summary_status_code_ = 1;
                for (const int sid : slave_ids_)
                {
                    GasSensorReading reading;
                    if (read_and_parse_sensor(fd, sid, reading))
                    {
                        has_valid_data = true;
                        update_alarm_state(reading, alarm);
                    }
                    readings.push_back(reading);
                    std::this_thread::sleep_for(std::chrono::milliseconds(inter_request_delay_ms_));
                }

                std::string message = alarm.active ? alarm.detail : "气体传感器运行中";
                uint8_t level = aggregate_level(readings);
                if (manual_test_alarm_active())
                {
                    apply_manual_test_alarm(alarm);
                    message = alarm.detail;
                    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                }
                if (!startup_check_done)
                {
                    const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count();
                    if (has_valid_data)
                    {
                        startup_check_done = true;
                    }
                    else if (elapsed >= startup_timeout_seconds_)
                    {
                        message = "启动等待超时，未读取到有效气体数据";
                        level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                        if (fail_on_startup_timeout_)
                            monitoring_active_ = false;
                        startup_check_done = true;
                    }
                }

                publish_status(readings, level, message);
                std::this_thread::sleep_for(std::chrono::milliseconds(poll_interval_ms_));
            }
            close(fd);
        }
    }

    std::string serial_port_;
    int baud_rate_{};
    std::string parity_;
    int poll_interval_ms_{};
    int startup_timeout_seconds_{};
    bool fail_on_startup_timeout_{};
    int inter_request_delay_ms_{};
    int response_timeout_ms_{};
    int alarm_repeat_seconds_{};
    int test_alarm_hold_seconds_{};
    int max_retries_per_slave_{};
    bool use_config_alarm_thresholds_{};
    bool pump_relay_enable_{};
    int pump_relay_gpio_{-1};
    bool pump_relay_active_high_{true};
    std::string pump_relay_socket_path_{gas_monitor::kDefaultPumpSocketPath};
    std::vector<int> slave_ids_;
    std::map<int, AlarmThreshold> threshold_overrides_;
    std::map<int, int> gas_type_overrides_;
    std::vector<int> alarm_threshold_slave_ids_;
    std::vector<double> low_alarm_overrides_;
    std::vector<double> high_alarm_overrides_;
    std::vector<std::string> gas_type_overrides_names_;
    mutable std::mutex config_mutex_;
    std::mutex parameter_update_mutex_;

    std::atomic<bool> monitoring_active_{false};
    std::thread polling_thread_;
    std::mutex thread_mutex_;
    mutable std::mutex status_mutex_;
    std::mutex pump_socket_mutex_;
    std::map<int, int> last_status_codes_;
    std::map<int, std::chrono::steady_clock::time_point> last_alarm_times_;
    int last_summary_status_code_{1};
    std::chrono::steady_clock::time_point last_manual_test_alarm_until_{};
    std::vector<GasSensorReading> last_readings_;
    int pump_socket_fd_{-1};
    PumpRelayState pump_relay_state_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr test_alarm_srv_;
    rclcpp::Service<rcl_interfaces::srv::SetParameters>::SharedPtr set_parameters_srv_;

    const std::map<int, std::string> gas_type_map_ = {{0, "NULL"}, {1, "AR"}, {2, "ASH3"}, {3, "B2H6"}, {4, "BR2"}, {5, "CO"}, {6, "CO2"}, {7, "COCL2"}, {8, "CH2O"}, {9, "CH2O2"}, {10, "CH3BR"}, {11, "CH4"}, {12, "CH4O"}, {13, "CH4S"}, {14, "CH5N"}, {15, "CH6O"}, {16, "CIC"}, {17, "CL2"}, {18, "CLO2"}, {19, "C2CL4"}, {20, "C2HCL3"}, {21, "C2H2"}, {22, "C2H3CL"}, {23, "C2H"}, {24, "C2H4O"}, {25, "C2H6O"}, {26, "C3H3N"}, {27, "C3H6O"}, {28, "C3H8"}, {29, "C3H8O"}, {30, "C4H8O2"}, {31, "C4H8S"}, {32, "C4H10"}, {33, "C4H10O"}, {34, "C5H12"}, {35, "C6H6"}, {36, "C6H6S"}, {37, "C6H12"}, {38, "C6H14"}, {39, "C7H8"}, {40, "C7H16"}, {41, "C8H8"}, {42, "C8H10"}, {43, "C8H18"}, {44, "CS2"}, {45, "EX"}, {46, "ETO"}, {47, "F2"}, {48, "FX"}, {49, "GEH4"}, {50, "H2"}, {51, "H2O2"}, {52, "H2S"}, {53, "HCL"}, {54, "HCN"}, {55, "HBR"}, {56, "HE"}, {57, "HF"}, {58, "I2"}, {59, "NO"}, {60, "NO2"}, {61, "NOX"}, {62, "NF3"}, {63, "NH3"}, {64, "N2"}, {65, "N2O"}, {66, "N2H4"}, {67, "O2"}, {68, "O3"}, {69, "PH3"}, {70, "PID"}, {71, "P2O5"}, {72, "SO2"}, {73, "SO2F2"}, {74, "SIH4"}, {75, "SIF4"}, {76, "SF6"}, {77, "THT"}, {78, "TVOC"}, {79, "VOC"}, {80, "VOCS"}, {81, "SO3"}, {82, "NMHC"}, {83, "温度"}, {84, "湿度"}, {85, "风速"}, {86, "风向"}, {87, "降雨量"}, {88, "噪音"}};
    const std::map<int, std::string> sensor_status_map_ = {{0, "预热"}, {1, "正常"}, {2, "数据错误"}, {3, "传感器故障"}, {4, "预警"}, {5, "低报"}, {6, "高报"}, {7, "访问故障"}, {8, "超量程"}, {9, "需要标定"}, {10, "超时"}, {11, "STEL报警"}, {12, "TWA报警"}, {13, "保留"}, {14, "保留"}, {15, "通信故障"}};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialGasNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
