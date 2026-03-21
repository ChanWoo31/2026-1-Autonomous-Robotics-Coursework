#include <chrono>
#include <functional>
#include <memory>
#include <string>

/*package dependencies -->CMakelist.txt, package.xml*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std::chrono_literals;


class cw01_publisher : public rclcpp::Node
{
    public:
    int input_a, input_b;
    cw01_publisher()
    : Node("cw01_publisher"), count_(0)
    {
        
        cw01_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("topic_cw01_aaa", 10);
        timer_=this->create_wall_timer(500ms, std::bind(&cw01_publisher::cw01_timer_callback, this));
    }
    private:
    void cw01_timer_callback()
    {
        auto message_cw01 = std_msgs::msg::Int32MultiArray();
        
        message_cw01.data.push_back(input_a);
        message_cw01.data.push_back(input_b);
        message_cw01.data.push_back(count_++);
        // message_ex01.data = std::"Hello, world! " + std::to_string(count_++);

        RCLCPP_INFO(this->get_logger(), "%d, %d ... hello world count %zu", input_a, input_b, count_);

        //RCLCPP_INFO_STREAM(get_logger(), "now(): "<<now().seconds());

        cw01_publisher_->publish(message_cw01);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr cw01_publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto aa = std::make_shared<cw01_publisher>();
    int a, b;
    scanf("%d %d", &a, &b);
    aa->input_a = a;
    aa->input_b = b;
    rclcpp::spin(aa);
    rclcpp::shutdown();
    return 0;
}
