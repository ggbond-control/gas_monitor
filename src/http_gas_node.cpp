#include <chrono>
#include <cctype>
#include <string>

#include <curl/curl.h>

#include "rclcpp/rclcpp.hpp"
#include "monitor_interfaces/msg/gas_leak_event.hpp"

using namespace std::chrono_literals;

static size_t WriteCallback(void *contents, size_t size, size_t nmemb, std::string *userp)
{
    userp->append(static_cast<char *>(contents), size * nmemb);
    return size * nmemb;
}

class HttpGasNode : public rclcpp::Node
{
public:
    HttpGasNode() : Node("http_gas_node")
    {
        gas_api_url_ = declare_parameter<std::string>("gas_api_url", "http://192.168.2.180/spl");
        gas_threshold_ = declare_parameter<double>("gas_threshold", 35.0);
        consecutive_seconds_ = declare_parameter<int>("consecutive_seconds", 4);
        source_name_ = declare_parameter<std::string>("source_name", "http_spl");

        pub_ = create_publisher<monitor_interfaces::msg::GasLeakEvent>("/monitor/gas/http/leak_event", 10);

        curl_global_init(CURL_GLOBAL_DEFAULT);
        timer_ = create_wall_timer(1s, std::bind(&HttpGasNode::tick, this));
    }

    ~HttpGasNode() override { curl_global_cleanup(); }

private:
    void tick()
    {
        double spl_value = 0.0;
        bool valid_data = false;
        std::string read_buffer;

        CURL *curl = curl_easy_init();
        if (curl)
        {
            curl_easy_setopt(curl, CURLOPT_URL, gas_api_url_.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &read_buffer);
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5L);
            const CURLcode res = curl_easy_perform(curl);
            curl_easy_cleanup(curl);

            if (res == CURLE_OK)
            {
                const std::string key = "\"spl\":";
                const size_t pos = read_buffer.find(key);
                if (pos != std::string::npos)
                {
                    size_t start = pos + key.length();
                    while (start < read_buffer.length() && std::isspace(read_buffer[start]))
                        start++;
                    size_t end = start;
                    while (end < read_buffer.length() &&
                           (std::isdigit(read_buffer[end]) || read_buffer[end] == '.' || read_buffer[end] == '-' ||
                            read_buffer[end] == 'e' || read_buffer[end] == 'E' || read_buffer[end] == '+'))
                    {
                        end++;
                    }
                    try
                    {
                        spl_value = std::stod(read_buffer.substr(start, end - start));
                        valid_data = true;
                    }
                    catch (...)
                    {
                        valid_data = false;
                    }
                }
            }
        }

        if (!valid_data)
            return;

        if (spl_value > gas_threshold_)
        {
            consecutive_count_++;
        }
        else
        {
            const bool was_leaking = leaking_;
            leaking_ = false;
            consecutive_count_ = 0;
            if (was_leaking)
                publish_event(false, spl_value, "gas level normalized");
        }

        if (consecutive_count_ >= consecutive_seconds_)
        {
            if (!leaking_)
            {
                leaking_ = true;
                publish_event(true, spl_value, "continuous threshold exceed");
            }
        }
    }

    void publish_event(bool leaking, double spl_value, const std::string &detail)
    {
        monitor_interfaces::msg::GasLeakEvent msg;
        msg.header.stamp = now();
        msg.header.frame_id = "gas_http";
        msg.leaking = leaking;
        msg.spl_value = spl_value;
        msg.threshold = gas_threshold_;
        msg.consecutive_seconds = consecutive_seconds_;
        msg.source = source_name_;
        msg.detail = detail;
        pub_->publish(msg);
    }

    std::string gas_api_url_;
    double gas_threshold_{};
    int consecutive_seconds_{};
    std::string source_name_;

    int consecutive_count_{0};
    bool leaking_{false};

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<monitor_interfaces::msg::GasLeakEvent>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HttpGasNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
