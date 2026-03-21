#include <chrono>
#include <functional>
#include <memory>
#include <string>

/*package dependencies -->CMakelist.txt, package.xml*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;


class ex01_publisher : public rclcpp::Node
{
    public:
    ex01_publisher()
    : Node("ex01_publisher"), count_(0)
    {
        ex01_publisher_ = this->create_publisher<std_msgs::msg::String>("topic_ex01_aaa", 10);
        timer_=this->create_wall_timer(500ms, std::bind(&ex01_publisher::ex01_timer_callback, this));
    }
    private:
    void ex01_timer_callback()
    {
        auto message_ex01 = std_msgs::msg::String();
        message_ex01.data = "Hello, world! " + std::to_string(count_++);

        RCLCPP_INFO(this->get_logger(), "Publishing: '%s' %f", message_ex01.data.c_str());

        //RCLCPP_INFO_STREAM(get_logger(), "now(): "<<now().seconds());

        ex01_publisher_->publish(message_ex01);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ex01_publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto aa = std::make_shared<ex01_publisher>();
    rclcpp::spin(aa);
    rclcpp::shutdown();
    return 0;
}
