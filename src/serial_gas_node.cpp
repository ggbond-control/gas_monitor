#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "monitor_interfaces/msg/gas_sensor_array.hpp"
#include "monitor_interfaces/msg/gas_sensor_reading.hpp"

using namespace std::chrono_literals;

class SerialGasNode : public rclcpp::Node
{
public:
    SerialGasNode() : Node("serial_gas_node")
    {
        serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        baud_rate_ = declare_parameter<int>("baud_rate", 9600);
        parity_ = declare_parameter<std::string>("parity", "none");
        poll_interval_ms_ = declare_parameter<int>("poll_interval_ms", 500);
        startup_timeout_seconds_ = declare_parameter<int>("startup_timeout_seconds", 15);
        fail_on_startup_timeout_ = declare_parameter<bool>("fail_on_startup_timeout", true);

        request_start_addr_ = declare_parameter<int>("request_start_addr", 0);
        request_count_ = declare_parameter<int>("request_count", 10);
        inter_request_delay_ms_ = declare_parameter<int>("inter_request_delay_ms", 500);
        max_retries_per_slave_ = declare_parameter<int>("max_retries_per_slave", 3);
        debug_hex_ = declare_parameter<bool>("debug_hex", true);

        auto slave_ids_raw = declare_parameter<std::vector<int64_t>>("slave_ids", {1, 2, 3, 4, 5, 6});
        for (auto sid : slave_ids_raw)
            slave_ids_.push_back(static_cast<int>(sid));

        pub_ = create_publisher<monitor_interfaces::msg::GasSensorArray>("/monitor/gas/serial/readings", 10);
        polling_thread_ = std::thread(&SerialGasNode::polling_loop, this);

        RCLCPP_INFO(
            get_logger(),
            "Serial gas node started. port=%s baud=%d parity=%s request=[start:%d count:%d] retries=%d inter_delay=%dms",
            serial_port_.c_str(), baud_rate_, parity_.c_str(), request_start_addr_, request_count_, max_retries_per_slave_, inter_request_delay_ms_);
    }

    ~SerialGasNode() override
    {
        running_ = false;
        if (polling_thread_.joinable())
            polling_thread_.join();
    }

private:
    static uint16_t calculate_crc16(const uint8_t *data, size_t length)
    {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < length; i++)
        {
            crc ^= data[i];
            for (int j = 0; j < 8; j++)
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

    static std::vector<uint8_t> build_modbus_request(int slave_id, int start_addr, int count)
    {
        std::vector<uint8_t> packet(6);
        packet[0] = static_cast<uint8_t>(slave_id);
        packet[1] = 0x03;
        packet[2] = static_cast<uint8_t>((start_addr >> 8) & 0xFF);
        packet[3] = static_cast<uint8_t>(start_addr & 0xFF);
        packet[4] = static_cast<uint8_t>((count >> 8) & 0xFF);
        packet[5] = static_cast<uint8_t>(count & 0xFF);

        const uint16_t crc = calculate_crc16(packet.data(), packet.size());
        packet.push_back(static_cast<uint8_t>(crc & 0xFF));
        packet.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
        return packet;
    }

    static std::string to_hex(const uint8_t *data, size_t len)
    {
        std::ostringstream oss;
        for (size_t i = 0; i < len; ++i)
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
            return B9600;
        }
    }

    int open_serial_port(const std::string &port_name, int baud_rate)
    {
        const int fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd == -1)
        {
            RCLCPP_ERROR(get_logger(), "Unable to open %s. Reason: %s", port_name.c_str(), strerror(errno));
            return -1;
        }
        // Switch back to blocking mode; non-blocking write may return EAGAIN on USB-serial adapters.
        if (fcntl(fd, F_SETFL, 0) == -1)
        {
            RCLCPP_WARN(get_logger(), "Failed to set blocking mode for %s: %s", port_name.c_str(), strerror(errno));
        }

        struct termios options{};
        tcgetattr(fd, &options);

        const auto speed = to_speed_t(baud_rate);
        cfsetispeed(&options, speed);
        cfsetospeed(&options, speed);

        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag &= ~CRTSCTS;

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

        options.c_cflag &= ~CSTOPB; // 1 stop bit
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag &= ~OPOST;
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 10;

        tcsetattr(fd, TCSANOW, &options);
        tcflush(fd, TCIOFLUSH);
        return fd;
    }

    monitor_interfaces::msg::GasSensorReading default_sensor_msg(int slave_id)
    {
        monitor_interfaces::msg::GasSensorReading msg;
        msg.id = slave_id;
        msg.gas = gas_name(slave_id);
        msg.concentration = -1.0;
        msg.unit = "N/A";
        msg.low_alarm = -1.0;
        msg.high_alarm = -1.0;
        msg.status = "通信失败";
        msg.temp = -1.0;
        msg.humidity = -1.0;
        return msg;
    }

    std::string gas_name(int sid) const
    {
        const auto it = gas_type_map_.find(sid);
        return it == gas_type_map_.end() ? "Unknown" : it->second;
    }

    std::string sensor_status(int code) const
    {
        const auto it = sensor_status_map_.find(code);
        return it == sensor_status_map_.end() ? "Unknown" : it->second;
    }

    bool parse_register_payload(int slave_id, const uint8_t *buffer, ssize_t bytes_read, monitor_interfaces::msg::GasSensorReading &out_msg)
    {
        if (bytes_read < 25)
            return false;
        if (buffer[0] != static_cast<uint8_t>(slave_id) || buffer[1] != 0x03)
            return false;

        const uint16_t reg0 = (static_cast<uint16_t>(buffer[3]) << 8) | buffer[4];
        const int unit_code = (reg0 >> 8) & 0x0F;
        const int decimal_code = (reg0 >> 12) & 0x0F;

        std::string unit_str = "ppm";
        if (unit_code == 2)
            unit_str = "%LEL";
        else if (unit_code == 4)
            unit_str = "%VOL";
        else if (unit_code == 6)
            unit_str = "mg/m3";
        else if (unit_code == 8)
            unit_str = "ppb";

        int decimal_places = 0;
        if (decimal_code == 4)
            decimal_places = 1;
        else if (decimal_code == 8)
            decimal_places = 2;
        else if (decimal_code == 12)
            decimal_places = 3;

        const double divisor = std::pow(10, decimal_places);

        const uint16_t raw_conc = (static_cast<uint16_t>(buffer[5]) << 8) | buffer[6];
        const uint16_t raw_low = (static_cast<uint16_t>(buffer[7]) << 8) | buffer[8];
        const uint16_t raw_high = (static_cast<uint16_t>(buffer[9]) << 8) | buffer[10];
        const uint16_t status_reg = (static_cast<uint16_t>(buffer[13]) << 8) | buffer[14];
        const int status_code = status_reg & 0xFF;
        const uint16_t raw_temp = (static_cast<uint16_t>(buffer[17]) << 8) | buffer[18];
        const uint16_t raw_hum = (static_cast<uint16_t>(buffer[21]) << 8) | buffer[22];

        out_msg.id = slave_id;
        out_msg.gas = gas_name(slave_id);
        out_msg.concentration = static_cast<double>(raw_conc) / divisor;
        out_msg.unit = unit_str;
        out_msg.low_alarm = static_cast<double>(raw_low) / divisor;
        out_msg.high_alarm = static_cast<double>(raw_high) / divisor;
        out_msg.status = sensor_status(status_code);
        out_msg.temp = static_cast<double>(raw_temp) / 10.0;
        out_msg.humidity = static_cast<double>(raw_hum) / 10.0;
        return true;
    }

    bool read_and_parse_sensor(int fd, int slave_id, monitor_interfaces::msg::GasSensorReading &out_msg)
    {
        const auto request = build_modbus_request(slave_id, request_start_addr_, request_count_);

        for (int attempt = 1; attempt <= std::max(1, max_retries_per_slave_); ++attempt)
        {
            tcflush(fd, TCIFLUSH);
            const ssize_t written = write(fd, request.data(), request.size());
            if (written != static_cast<ssize_t>(request.size()))
            {
                RCLCPP_WARN(get_logger(), "[SERIAL] sid=%d attempt=%d write failed written=%zd errno=%s", slave_id, attempt, written, strerror(errno));
                continue;
            }

            if (debug_hex_)
            {
                RCLCPP_INFO(get_logger(), "[SERIAL TX] sid=%d attempt=%d len=%zu data=%s", slave_id, attempt, request.size(), to_hex(request).c_str());
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(inter_request_delay_ms_));

            uint8_t buffer[256]{};
            const ssize_t bytes_read = read(fd, buffer, sizeof(buffer));
            if (bytes_read <= 0)
            {
                RCLCPP_WARN(get_logger(), "[SERIAL RX] sid=%d attempt=%d no data bytes_read=%zd errno=%s", slave_id, attempt, bytes_read, strerror(errno));
                continue;
            }

            if (debug_hex_)
            {
                RCLCPP_INFO(get_logger(), "[SERIAL RX] sid=%d attempt=%d len=%zd data=%s", slave_id, attempt, bytes_read, to_hex(buffer, static_cast<size_t>(bytes_read)).c_str());
            }

            if (parse_register_payload(slave_id, buffer, bytes_read, out_msg))
            {
                return true;
            }

            RCLCPP_WARN(get_logger(), "[SERIAL] sid=%d attempt=%d parse failed.", slave_id, attempt);
        }

        return false;
    }

    void publish_array(const std::vector<monitor_interfaces::msg::GasSensorReading> &readings)
    {
        monitor_interfaces::msg::GasSensorArray arr;
        arr.header.stamp = now();
        arr.header.frame_id = "gas_serial";
        arr.readings = readings;
        pub_->publish(arr);
    }

    void polling_loop()
    {
        const auto start_time = std::chrono::steady_clock::now();
        bool has_successfully_read = false;
        bool startup_check_done = false;

        std::map<int, monitor_interfaces::msg::GasSensorReading> last_valid_cache;

        while (rclcpp::ok() && running_)
        {
            if (!startup_check_done)
            {
                const auto now_time = std::chrono::steady_clock::now();
                const double elapsed = std::chrono::duration<double>(now_time - start_time).count();
                if (elapsed > startup_timeout_seconds_)
                {
                    if (!has_successfully_read)
                    {
                        if (fail_on_startup_timeout_)
                        {
                            RCLCPP_FATAL(get_logger(), "Startup timeout (%d s) reached with NO valid data. Shutting down.", startup_timeout_seconds_);
                            rclcpp::shutdown();
                            return;
                        }
                        else
                        {
                            RCLCPP_ERROR(get_logger(), "Startup timeout reached but fail_on_startup_timeout=false; continue publishing default values.");
                            startup_check_done = true;
                        }
                    }
                    else
                    {
                        startup_check_done = true;
                    }
                }
            }

            const int fd = open_serial_port(serial_port_, baud_rate_);
            if (fd == -1)
            {
                std::vector<monitor_interfaces::msg::GasSensorReading> defaults;
                for (const int sid : slave_ids_)
                    defaults.push_back(default_sensor_msg(sid));
                publish_array(defaults);
                std::this_thread::sleep_for(2s);
                continue;
            }

            while (rclcpp::ok() && running_)
            {
                std::vector<monitor_interfaces::msg::GasSensorReading> cycle;
                bool current_cycle_success = false;

                for (const int sid : slave_ids_)
                {
                    monitor_interfaces::msg::GasSensorReading reading;
                    if (read_and_parse_sensor(fd, sid, reading))
                    {
                        cycle.push_back(reading);
                        last_valid_cache[sid] = reading;
                        current_cycle_success = true;
                    }
                    else
                    {
                        auto it = last_valid_cache.find(sid);
                        cycle.push_back(it != last_valid_cache.end() ? it->second : default_sensor_msg(sid));
                    }
                }

                if (current_cycle_success)
                {
                    has_successfully_read = true;
                    startup_check_done = true;
                }

                publish_array(cycle);
                std::this_thread::sleep_for(std::chrono::milliseconds(poll_interval_ms_));

                if (!startup_check_done)
                {
                    const auto now_time = std::chrono::steady_clock::now();
                    const double elapsed = std::chrono::duration<double>(now_time - start_time).count();
                    if (elapsed > startup_timeout_seconds_ && !has_successfully_read)
                    {
                        if (fail_on_startup_timeout_)
                        {
                            RCLCPP_FATAL(get_logger(), "Startup timeout reached during polling loop. Shutting down.");
                            close(fd);
                            rclcpp::shutdown();
                            return;
                        }
                        startup_check_done = true;
                    }
                }
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

    int request_start_addr_{};
    int request_count_{};
    int inter_request_delay_ms_{};
    int max_retries_per_slave_{};
    bool debug_hex_{};

    std::vector<int> slave_ids_;

    std::atomic<bool> running_{true};
    std::thread polling_thread_;
    rclcpp::Publisher<monitor_interfaces::msg::GasSensorArray>::SharedPtr pub_;

    const std::map<int, std::string> gas_type_map_ = {{1, "SF6"}, {2, "CO"}, {3, "O2"}, {4, "CH4"}, {5, "CO2"}, {6, "O3"}};
    const std::map<int, std::string> sensor_status_map_ = {{0, "预热"}, {1, "正常"}, {2, "数据错误"}, {3, "传感器故障"}, {4, "预警"}, {5, "低报"}, {6, "高报"}, {7, "超量程"}};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialGasNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
