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
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "monitor_interfaces/msg/gas_alarm_event.hpp"
#include "monitor_interfaces/msg/gas_sensor_array.hpp"
#include "monitor_interfaces/msg/gas_sensor_reading.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class SerialGasNode : public rclcpp::Node
{
public:
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
        temperature_offset_raw_ = declare_parameter<int>("temperature_offset_raw", 0);
        max_retries_per_slave_ = declare_parameter<int>("max_retries_per_slave", 3);
        debug_hex_ = declare_parameter<bool>("debug_hex", true);

        const auto slave_ids_raw = declare_parameter<std::vector<int64_t>>("slave_ids", {1});
        for (const auto sid : slave_ids_raw)
            slave_ids_.push_back(static_cast<int>(sid));

        readings_pub_ = create_publisher<monitor_interfaces::msg::GasSensorArray>("/monitor/gas/serial/readings", 10);
        alarm_pub_ = create_publisher<monitor_interfaces::msg::GasAlarmEvent>("/monitor/gas/serial/alarm_event", 10);
        polling_thread_ = std::thread(&SerialGasNode::polling_loop, this);

        RCLCPP_INFO(get_logger(), "串口气体节点已启动：串口=%s 波特率=%d 校验=%s 请求=起始地址0,寄存器数10 重试=%d 发送间隔=%dms 接收超时=%dms 报警重复=%d秒 温度偏移=%d",
                    serial_port_.c_str(), baud_rate_, parity_.c_str(), max_retries_per_slave_, inter_request_delay_ms_, response_timeout_ms_, alarm_repeat_seconds_, temperature_offset_raw_);
    }

    ~SerialGasNode() override
    {
        running_ = false;
        if (polling_thread_.joinable())
            polling_thread_.join();
    }

private:
    static constexpr const char *kLogRed = "\033[31m";
    static constexpr const char *kLogYellow = "\033[33m";
    static constexpr const char *kLogGreen = "\033[32m";
    static constexpr const char *kLogReset = "\033[0m";
    static constexpr int kRegisterCount = 10;
    static constexpr size_t kResponseSize = 25;

    static uint16_t calculate_crc16(const uint8_t *data, size_t length)
    {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < length; ++i)
        {
            crc ^= data[i];
            for (int bit = 0; bit < 8; ++bit)
            {
                if (crc & 1)
                {
                    crc >>= 1;
                    crc ^= 0xA001;
                }
                else
                {
                    crc >>= 1;
                }
            }
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
        return to_hex(data.data(), data.size());
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

    std::vector<uint8_t> read_response_frame(int fd)
    {
        std::vector<uint8_t> frame;
        frame.reserve(kResponseSize);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(response_timeout_ms_);

        while (rclcpp::ok() && running_ && std::chrono::steady_clock::now() < deadline)
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

    monitor_interfaces::msg::GasSensorReading default_sensor_msg(int slave_id) const
    {
        monitor_interfaces::msg::GasSensorReading msg;
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
        msg.status = "通信失败";
        msg.ad_value = -1;
        msg.temp = -1.0;
        msg.humidity = -1.0;
        return msg;
    }

    bool parse_register_payload(int slave_id, const std::vector<uint8_t> &frame, monitor_interfaces::msg::GasSensorReading &msg)
    {
        const std::string error = frame_error(slave_id, frame);
        if (!error.empty())
            return false;

        uint16_t registers[kRegisterCount]{};
        for (size_t i = 0; i < kRegisterCount; ++i)
            registers[i] = register_value(frame, i);

        const int unit_code = (registers[0] >> 12) & 0x0F;
        const int decimal_code = (registers[0] >> 8) & 0x0F;
        const int decimal_places = decimal_code == 4 ? 1 : decimal_code == 8 ? 2 : decimal_code == 12  ? 3 : 0;
        const double divisor = std::pow(10, decimal_places);
        const std::string unit = unit_code == 2 ? "%LEL" : unit_code == 4 ? "%VOL" : unit_code == 6   ? "mg/m3" : unit_code == 8   ? "ppb" : "ppm";
        const int gas_type_code = (registers[8] >> 8) & 0xFF;
        const int status_code = registers[5] & 0xFF;

        msg.id = slave_id;
        msg.valid = true;
        msg.gas = gas_name(gas_type_code);
        msg.gas_type_code = gas_type_code;
        msg.concentration = static_cast<double>(registers[1]) / divisor;
        msg.unit = unit;
        msg.low_alarm = static_cast<double>(registers[2]) / divisor;
        msg.high_alarm = static_cast<double>(registers[3]) / divisor;
        msg.full_scale = static_cast<double>(registers[4]) / divisor;
        msg.status_code = status_code;
        msg.status = sensor_status(status_code);
        msg.ad_value = registers[6];
        msg.temp = (static_cast<double>(registers[7]) - temperature_offset_raw_) / 10.0;
        msg.humidity = static_cast<double>(registers[9]) / 10.0;

        RCLCPP_INFO(get_logger(),
                    "[气体] 地址=%d 原始=%04X,%04X,%04X,%04X,%04X,%04X,%04X,%04X,%04X,%04X "
                    "气体=%s(%d) 浓度=%.3f%s 低报=%.3f 高报=%.3f 量程=%.3f 状态=%s(0x%02X) AD=%d 温度=%.1f°C 湿度=%.1f%%RH",
                    slave_id, registers[0], registers[1], registers[2], registers[3], registers[4],
                    registers[5], registers[6], registers[7], registers[8], registers[9],
                    msg.gas.c_str(), msg.gas_type_code, msg.concentration, msg.unit.c_str(), msg.low_alarm,
                    msg.high_alarm, msg.full_scale, msg.status.c_str(), msg.status_code, msg.ad_value, msg.temp, msg.humidity);
        return true;
    }

    bool read_and_parse_sensor(int fd, int slave_id, monitor_interfaces::msg::GasSensorReading &msg)
    {
        const auto request = build_modbus_request(slave_id);
        for (int attempt = 1; attempt <= std::max(1, max_retries_per_slave_); ++attempt)
        {
            tcflush(fd, TCIFLUSH);
            const ssize_t written = write(fd, request.data(), request.size());
            if (written != static_cast<ssize_t>(request.size()))
            {
                RCLCPP_WARN(get_logger(), "[串口] 地址=%d 第%d次发送失败，已写入=%zd，错误=%s",
                            slave_id, attempt, written, strerror(errno));
                continue;
            }
            if (tcdrain(fd) != 0)
            {
                RCLCPP_WARN(get_logger(), "[串口] 地址=%d 第%d次等待发送完成失败：%s",
                            slave_id, attempt, strerror(errno));
                continue;
            }
            if (debug_hex_)
                RCLCPP_DEBUG(get_logger(), "[串口] 地址=%d 第%d次 数据=%s", slave_id, attempt, to_hex(request).c_str());

            const auto frame = read_response_frame(fd);
            const std::string error = frame_error(slave_id, frame);
            if (error.empty() && parse_register_payload(slave_id, frame, msg))
                return true;

            RCLCPP_WARN(get_logger(), "[串口] 地址=%d 第%d次应答无效：%s，长度=%zu，数据=%s",
                        slave_id, attempt, error.c_str(), frame.size(), to_hex(frame).c_str());
            std::this_thread::sleep_for(std::chrono::milliseconds(inter_request_delay_ms_));
        }
        return false;
    }

    void publish_gas_alarm(const monitor_interfaces::msg::GasSensorReading &reading)
    {
        monitor_interfaces::msg::GasAlarmEvent event;
        event.header.stamp = now();
        event.header.frame_id = "gas_serial";
        event.id = reading.id;
        event.gas = reading.gas;
        event.status_code = reading.status_code;
        event.status = reading.status;
        event.level = alarm_level(reading.status_code);
        event.active = reading.status_code != 1;
        event.audible = audible_status(reading.status_code);
        event.detail = event.active ? "气体传感器状态异常：" + reading.status : "气体传感器状态已恢复正常";
        alarm_pub_->publish(event);
        RCLCPP_INFO(get_logger(), "%s[气体] 地址=%d 气体=%s 状态=%s(0x%02X) 级别=%s 音频=%s%s",
                    event.active ? (event.audible ? kLogRed : kLogYellow) : kLogGreen, event.id, event.gas.c_str(), event.status.c_str(), event.status_code, event.level.c_str(), event.audible ? "是" : "否", kLogReset);
    }

    void update_alarm_state(const monitor_interfaces::msg::GasSensorReading &reading)
    {
        const auto now_tp = std::chrono::steady_clock::now();
        const auto status_it = last_status_codes_.find(reading.id);
        const bool status_changed = status_it == last_status_codes_.end() || status_it->second != reading.status_code;
        const auto repeat_it = last_alarm_times_.find(reading.id);
        const bool repeat_due = repeat_it == last_alarm_times_.end() || std::chrono::duration_cast<std::chrono::seconds>(now_tp - repeat_it->second).count() >= alarm_repeat_seconds_;

        if ((status_changed && (reading.status_code != 1 || status_it != last_status_codes_.end())) || (audible_status(reading.status_code) && repeat_due))
        {
            publish_gas_alarm(reading);
            last_alarm_times_[reading.id] = now_tp;
        }
        last_status_codes_[reading.id] = reading.status_code;
    }

    void publish_array(const std::vector<monitor_interfaces::msg::GasSensorReading> &readings)
    {
        monitor_interfaces::msg::GasSensorArray array;
        array.header.stamp = now();
        array.header.frame_id = "gas_serial";
        array.readings = readings;
        readings_pub_->publish(array);
    }

    void polling_loop()
    {
        const auto start_time = std::chrono::steady_clock::now();
        bool startup_check_done = false;
        bool has_valid_data = false;

        while (rclcpp::ok() && running_)
        {
            const int fd = open_serial_port();
            if (fd == -1)
            {
                std::vector<monitor_interfaces::msg::GasSensorReading> readings;
                for (const int sid : slave_ids_)
                    readings.push_back(default_sensor_msg(sid));
                publish_array(readings);
                std::this_thread::sleep_for(2s);
                continue;
            }

            while (rclcpp::ok() && running_)
            {
                std::vector<monitor_interfaces::msg::GasSensorReading> readings;
                for (const int sid : slave_ids_)
                {
                    monitor_interfaces::msg::GasSensorReading reading;
                    if (read_and_parse_sensor(fd, sid, reading))
                    {
                        has_valid_data = true;
                        update_alarm_state(reading);
                        readings.push_back(reading);
                    }
                    else
                    {
                        readings.push_back(default_sensor_msg(sid));
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(inter_request_delay_ms_));
                }
                publish_array(readings);

                if (!startup_check_done)
                {
                    const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count();
                    if (has_valid_data)
                    {
                        startup_check_done = true;
                    }
                    else if (elapsed >= startup_timeout_seconds_)
                    {
                        if (fail_on_startup_timeout_)
                        {
                            RCLCPP_FATAL(get_logger(), "启动等待超时（%d 秒），未读取到有效气体数据，节点即将退出。", startup_timeout_seconds_);
                            close(fd);
                            rclcpp::shutdown();
                            return;
                        }
                        RCLCPP_ERROR(get_logger(), "启动等待超时，但 fail_on_startup_timeout=false，将继续发布通信失败状态。");
                        startup_check_done = true;
                    }
                }
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
    int temperature_offset_raw_{};
    int max_retries_per_slave_{};
    bool debug_hex_{};
    std::vector<int> slave_ids_;

    std::atomic<bool> running_{true};
    std::thread polling_thread_;
    std::map<int, int> last_status_codes_;
    std::map<int, std::chrono::steady_clock::time_point> last_alarm_times_;
    rclcpp::Publisher<monitor_interfaces::msg::GasSensorArray>::SharedPtr readings_pub_;
    rclcpp::Publisher<monitor_interfaces::msg::GasAlarmEvent>::SharedPtr alarm_pub_;

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
