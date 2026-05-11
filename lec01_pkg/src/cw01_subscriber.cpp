#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
using std::placeholders::_1;

class cw01_subscriber : public rclcpp::Node
{
    public:
    cw01_subscriber()
    : Node("cw01_subscriber")
    {
        cw01_subscriber_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "topic_cw01_aaa", 10, std::bind(&cw01_subscriber::cw01_topic_callback, this, _1)
        );
    }
    private:
    void cw01_topic_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) const
    {
        int a = msg->data[0];
        int b = msg->data[1];
        int count = msg->data[2];
        int c = a * count;
        int d = b * count;
        RCLCPP_INFO(this->get_logger(), "hello world count *%d, %d", c, d);
    }
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr cw01_subscriber_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto a = std::make_shared<cw01_subscriber>();
    rclcpp::spin(a);
    rclcpp::shutdown();
    return 0;
}