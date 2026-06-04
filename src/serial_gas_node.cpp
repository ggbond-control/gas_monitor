#include <fcntl.h>
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
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
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
        temperature_offset_raw_ = declare_parameter<int>("temperature_offset_raw", 0);
        max_retries_per_slave_ = declare_parameter<int>("max_retries_per_slave", 3);
        debug_hex_ = declare_parameter<bool>("debug_hex", true);
        use_config_alarm_thresholds_ = declare_parameter<bool>("use_config_alarm_thresholds", false);

        const auto slave_ids_raw = declare_parameter<std::vector<int64_t>>("slave_ids", {1});
        for (const auto sid : slave_ids_raw)
            slave_ids_.push_back(static_cast<int>(sid));

        const auto threshold_ids_raw = declare_parameter<std::vector<int64_t>>("alarm_threshold_slave_ids", std::vector<int64_t>{});
        const auto low_alarm_raw = declare_parameter<std::vector<double>>("low_alarm_overrides", std::vector<double>{});
        const auto high_alarm_raw = declare_parameter<std::vector<double>>("high_alarm_overrides", std::vector<double>{});
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

        status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/monitor/gas/status", 10);
        start_srv_ = create_service<std_srvs::srv::Trigger>("/monitor/gas/start", std::bind(&SerialGasNode::on_start, this, std::placeholders::_1, std::placeholders::_2));
        stop_srv_ = create_service<std_srvs::srv::Trigger>("/monitor/gas/stop", std::bind(&SerialGasNode::on_stop, this, std::placeholders::_1, std::placeholders::_2));
        test_alarm_srv_ = create_service<std_srvs::srv::Trigger>("/monitor/gas/test_alarm", std::bind(&SerialGasNode::on_test_alarm, this, std::placeholders::_1, std::placeholders::_2));

        publish_status(default_readings(), diagnostic_msgs::msg::DiagnosticStatus::STALE, "气体传感器未启动");
        RCLCPP_INFO(get_logger(), "气体传感器服务已就绪：start=/monitor/gas/start stop=/monitor/gas/stop status=/monitor/gas/status 串口=%s 站号=%s 阈值覆盖=%s",
                    serial_port_.c_str(), join_ints(slave_ids_).c_str(), use_config_alarm_thresholds_ ? "开启" : "关闭");
    }

    ~SerialGasNode() override
    {
        monitoring_active_ = false;
        if (polling_thread_.joinable())
            polling_thread_.join();
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

        std::string probe_message;
        std::vector<GasSensorReading> probe_readings;
        if (!probe_sensors(probe_readings, probe_message))
        {
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
                response->success = true;
                response->message = "气体传感器已停止";
                publish_status(default_readings(), diagnostic_msgs::msg::DiagnosticStatus::STALE, "气体传感器已停止");
                return;
            }
            monitoring_active_ = false;
        }

        if (polling_thread_.joinable())
            polling_thread_.join();

        response->success = true;
        response->message = "气体传感器已停止";
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
        publish_status(readings, diagnostic_msgs::msg::DiagnosticStatus::WARN, alarm.detail);

        response->success = true;
        response->message = "已触发气体传感器测试报警";
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
        msg.gas_type_code = (msg.registers[8] >> 8) & 0xFF;
        msg.status_code = msg.registers[5] & 0xFF;

        msg.id = slave_id;
        msg.valid = true;
        msg.gas = gas_name(msg.gas_type_code);
        msg.concentration = static_cast<double>(msg.registers[1]) / divisor;
        msg.low_alarm = static_cast<double>(msg.registers[2]) / divisor;
        msg.high_alarm = static_cast<double>(msg.registers[3]) / divisor;
        apply_config_threshold_override(msg);
        msg.full_scale = static_cast<double>(msg.registers[4]) / divisor;
        msg.status = sensor_status(msg.status_code);
        msg.ad_value = msg.registers[6];
        msg.temp = (static_cast<double>(msg.registers[7]) - temperature_offset_raw_) / 10.0;
        msg.humidity = static_cast<double>(msg.registers[9]) / 10.0;
        msg.error.clear();

        RCLCPP_INFO(get_logger(), "[气体] 地址=%d 气体=%s(%d) 浓度=%.3f%s 低报=%.3f 高报=%.3f 状态=%s(0x%02X) AD=%d 温度=%.1f°C 湿度=%.1f%%RH",
                    slave_id, msg.gas.c_str(), msg.gas_type_code, msg.concentration, msg.unit.c_str(),
                    msg.low_alarm, msg.high_alarm, msg.status.c_str(), msg.status_code, msg.ad_value, msg.temp, msg.humidity);
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
        if (reading.status_code == 1 || reading.status_code == 4 || reading.status_code == 5 || reading.status_code == 6)
        {
            if (reading.high_alarm >= 0.0 && reading.concentration >= reading.high_alarm)
                reading.status_code = 6;
            else if (reading.low_alarm >= 0.0 && reading.concentration >= reading.low_alarm)
                reading.status_code = 5;
            else
                reading.status_code = 1;
            reading.status = sensor_status(reading.status_code);
        }
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
            if (debug_hex_)
                RCLCPP_DEBUG(get_logger(), "[串口] 地址=%d 第%d次 数据=%s", slave_id, attempt, to_hex(request).c_str());

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
        summary.active = reading.status_code != 1;
        summary.type = "gas_" + level;
        if (summary.active)
            summary.detail = "气体传感器状态异常：" + reading.status;
        else
        {
            summary.type = "gas_normal";
            summary.detail = "";
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
                    if (level == diagnostic_msgs::msg::DiagnosticStatus::OK)
                        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
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
    int temperature_offset_raw_{};
    int max_retries_per_slave_{};
    bool debug_hex_{};
    bool use_config_alarm_thresholds_{};
    std::vector<int> slave_ids_;
    std::map<int, AlarmThreshold> threshold_overrides_;

    std::atomic<bool> monitoring_active_{false};
    std::thread polling_thread_;
    std::mutex thread_mutex_;
    mutable std::mutex status_mutex_;
    std::map<int, int> last_status_codes_;
    std::map<int, std::chrono::steady_clock::time_point> last_alarm_times_;
    std::chrono::steady_clock::time_point last_manual_test_alarm_until_{};
    std::vector<GasSensorReading> last_readings_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr test_alarm_srv_;

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
