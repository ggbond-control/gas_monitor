#include "gas_monitor/pump_protocol.hpp"

#include <array>
#include <cstring>
#include <string>
#include <vector>

#include <unistd.h>

namespace gas_monitor
{
    namespace
    {

        class PayloadWriter
        {
        public:
            void write_u8(std::uint8_t value)
            {
                buffer_.push_back(value);
            }

            void write_u32(std::uint32_t value)
            {
                for (int shift = 0; shift < 32; shift += 8)
                    buffer_.push_back(static_cast<std::uint8_t>((value >> shift) & 0xFF));
            }

            void write_i32(std::int32_t value)
            {
                write_u32(static_cast<std::uint32_t>(value));
            }

            void write_string(const std::string &value)
            {
                write_u32(static_cast<std::uint32_t>(value.size()));
                buffer_.insert(buffer_.end(), value.begin(), value.end());
            }

            std::vector<std::uint8_t> take()
            {
                return std::move(buffer_);
            }

        private:
            std::vector<std::uint8_t> buffer_;
        };

        class PayloadReader
        {
        public:
            explicit PayloadReader(const std::vector<std::uint8_t> &buffer) : buffer_(buffer) {}

            bool read_u8(std::uint8_t *value, std::string *error)
            {
                if (remaining() < 1)
                {
                    if (error)
                        *error = "payload too short";
                    return false;
                }
                *value = buffer_[offset_++];
                return true;
            }

            bool read_u32(std::uint32_t *value, std::string *error)
            {
                if (remaining() < 4)
                {
                    if (error)
                        *error = "payload too short";
                    return false;
                }
                *value = static_cast<std::uint32_t>(buffer_[offset_]) |
                         (static_cast<std::uint32_t>(buffer_[offset_ + 1]) << 8) |
                         (static_cast<std::uint32_t>(buffer_[offset_ + 2]) << 16) |
                         (static_cast<std::uint32_t>(buffer_[offset_ + 3]) << 24);
                offset_ += 4;
                return true;
            }

            bool read_i32(std::int32_t *value, std::string *error)
            {
                std::uint32_t raw{};
                if (!read_u32(&raw, error))
                    return false;
                *value = static_cast<std::int32_t>(raw);
                return true;
            }

            bool read_string(std::string *value, std::string *error)
            {
                std::uint32_t size{};
                if (!read_u32(&size, error))
                    return false;
                if (remaining() < size)
                {
                    if (error)
                        *error = "payload too short";
                    return false;
                }
                value->assign(reinterpret_cast<const char *>(buffer_.data() + offset_), size);
                offset_ += size;
                return true;
            }

        private:
            std::size_t remaining() const
            {
                return buffer_.size() - offset_;
            }

            const std::vector<std::uint8_t> &buffer_;
            std::size_t offset_{0};
        };

        bool write_all(int fd, const void *data, std::size_t size, std::string *error)
        {
            const auto *ptr = static_cast<const std::uint8_t *>(data);
            std::size_t written = 0;
            while (written < size)
            {
                const ssize_t rc = ::write(fd, ptr + written, size - written);
                if (rc <= 0)
                {
                    if (error)
                        *error = "write failed";
                    return false;
                }
                written += static_cast<std::size_t>(rc);
            }
            return true;
        }

        bool read_all(int fd, void *data, std::size_t size, std::string *error)
        {
            auto *ptr = static_cast<std::uint8_t *>(data);
            std::size_t received = 0;
            while (received < size)
            {
                const ssize_t rc = ::read(fd, ptr + received, size - received);
                if (rc <= 0)
                {
                    if (error)
                        *error = "read failed";
                    return false;
                }
                received += static_cast<std::size_t>(rc);
            }
            return true;
        }

    } // namespace

    std::vector<std::uint8_t> serialize_pump_config(const PumpConfig &config)
    {
        PayloadWriter writer;
        writer.write_i32(config.relay_gpio);
        writer.write_u8(config.active_high ? 1 : 0);
        return writer.take();
    }

    bool deserialize_pump_config(const std::vector<std::uint8_t> &payload, PumpConfig *config, std::string *error)
    {
        PayloadReader reader(payload);
        std::int32_t relay_gpio{};
        std::uint8_t active_high{};
        return reader.read_i32(&relay_gpio, error) &&
               reader.read_u8(&active_high, error) &&
               (config->relay_gpio = relay_gpio, config->active_high = active_high != 0, true);
    }

    std::vector<std::uint8_t> serialize_pump_state_command(const PumpStateCommand &command)
    {
        PayloadWriter writer;
        writer.write_u8(command.enable ? 1 : 0);
        return writer.take();
    }

    bool deserialize_pump_state_command(const std::vector<std::uint8_t> &payload, PumpStateCommand *command, std::string *error)
    {
        PayloadReader reader(payload);
        std::uint8_t enable{};
        if (!reader.read_u8(&enable, error))
            return false;
        command->enable = enable != 0;
        return true;
    }

    std::vector<std::uint8_t> serialize_pump_status_response(const PumpStatusResponse &response)
    {
        PayloadWriter writer;
        writer.write_u8(response.success ? 1 : 0);
        writer.write_u8(response.agent_ready ? 1 : 0);
        writer.write_u8(response.configured ? 1 : 0);
        writer.write_u8(response.enable ? 1 : 0);
        writer.write_u8(response.active_high ? 1 : 0);
        writer.write_i32(response.relay_gpio);
        writer.write_string(response.message);
        return writer.take();
    }

    bool deserialize_pump_status_response(const std::vector<std::uint8_t> &payload, PumpStatusResponse *response, std::string *error)
    {
        PayloadReader reader(payload);
        std::uint8_t success{};
        std::uint8_t agent_ready{};
        std::uint8_t configured{};
        std::uint8_t enable{};
        std::uint8_t active_high{};
        std::int32_t relay_gpio{};
        std::string message;
        if (!reader.read_u8(&success, error) ||
            !reader.read_u8(&agent_ready, error) ||
            !reader.read_u8(&configured, error) ||
            !reader.read_u8(&enable, error) ||
            !reader.read_u8(&active_high, error) ||
            !reader.read_i32(&relay_gpio, error) ||
            !reader.read_string(&message, error))
        {
            return false;
        }
        response->success = success != 0;
        response->agent_ready = agent_ready != 0;
        response->configured = configured != 0;
        response->enable = enable != 0;
        response->active_high = active_high != 0;
        response->relay_gpio = relay_gpio;
        response->message = std::move(message);
        return true;
    }

    bool write_pump_message(int fd, PumpMessageType type, const std::vector<std::uint8_t> &payload, std::string *error)
    {
        PumpMessageHeader header{};
        std::memcpy(header.magic, kPumpProtocolMagic, sizeof(header.magic));
        header.version = kPumpProtocolVersion;
        header.type = static_cast<std::uint16_t>(type);
        header.payload_size = static_cast<std::uint32_t>(payload.size());
        return write_all(fd, &header, sizeof(header), error) &&
               (payload.empty() || write_all(fd, payload.data(), payload.size(), error));
    }

    bool read_pump_message(int fd, PumpMessageType *type, std::vector<std::uint8_t> *payload, std::string *error)
    {
        PumpMessageHeader header{};
        if (!read_all(fd, &header, sizeof(header), error))
            return false;
        if (std::memcmp(header.magic, kPumpProtocolMagic, sizeof(header.magic)) != 0)
        {
            if (error)
                *error = "invalid magic";
            return false;
        }
        if (header.version != kPumpProtocolVersion)
        {
            if (error)
                *error = "invalid version";
            return false;
        }
        *type = static_cast<PumpMessageType>(header.type);
        payload->assign(header.payload_size, 0);
        return header.payload_size == 0 || read_all(fd, payload->data(), payload->size(), error);
    }

} // namespace gas_monitor
