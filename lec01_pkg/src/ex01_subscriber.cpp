#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class ex01_subscriber : public rclcpp::Node
{
    public:
    ex01_subscriber()
    : Node("ex01_subscriber")
    {
        ex01_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "topic_ex01_aaa", 10, std::bind(&ex01_subscriber::ex01_topic_callback, this, _1)
        );
    }
    private:
    void ex01_topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ex01_subscriber_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto a = std::make_shared<ex01_subscriber>();
    rclcpp::spin(a);
    rclcpp::shutdown();
    return 0;
}