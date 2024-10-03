#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <chrono>

class DynamicSubscriberNode : public rclcpp::Node
{
public:
    DynamicSubscriberNode()
        : Node("dynamic_subscriber_node"), last_time_(this->now()), message_count_(0)
    {
        // Declare parameters for topic name and topic type
        this->declare_parameter<std::string>("topic_name", "/camera/camera/depth/image_rect_raw");
        this->declare_parameter<std::string>("topic_type", "sensor_msgs/msg/Image");

        // Retrieve the parameters
        std::string topic_name = this->get_parameter("topic_name").as_string();
        std::string topic_type = this->get_parameter("topic_type").as_string();

        // Timer to print frequency at regular intervals
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&DynamicSubscriberNode::print_frequency, this));

        // Dynamically create the subscriber based on the topic type
        if (topic_type == "sensor_msgs/msg/CompressedImage") {
            RCLCPP_INFO(this->get_logger(), "Subscribing to sensor_msgs/msg/CompressedImage on topic: %s", topic_name.c_str());
            compressed_image_subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                topic_name, rclcpp::QoS(10).best_effort(),
                std::bind(&DynamicSubscriberNode::compressed_image_callback, this, std::placeholders::_1));
        } else if (topic_type == "sensor_msgs/msg/Image") {
            RCLCPP_INFO(this->get_logger(), "Subscribing to sensor_msgs/msg/Image on topic: %s", topic_name.c_str());
            image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                topic_name, rclcpp::QoS(10).best_effort(),
                std::bind(&DynamicSubscriberNode::image_callback, this, std::placeholders::_1));
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unknown message type: %s", topic_type.c_str());
        }
    }

private:
    // Callback for CompressedImage
    void compressed_image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        message_count_++;
        calculate_hz();
    }

    // Callback for Image
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        message_count_++;
        calculate_hz();
    }

    // Function to calculate the Hz (frequency)
    void calculate_hz()
    {
        auto current_time = this->now();
        auto time_diff = current_time - last_time_;
        last_time_ = current_time;

        auto time_diff_in_seconds = time_diff.seconds();
        if (time_diff_in_seconds > 0.0)
        {
            current_hz_ = 1.0 / time_diff_in_seconds;
        }
    }

    // Print the current frequency
    void print_frequency()
    {
        RCLCPP_INFO(this->get_logger(), "Current Frequency (Hz): %.2f", current_hz_);
        message_count_ = 0;
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time last_time_;
    double current_hz_ = 0.0;
    size_t message_count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
