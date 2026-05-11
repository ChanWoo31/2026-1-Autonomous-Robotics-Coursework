#include <chrono>
#include <functional>
#include <memory>
#include <string>

/*package dependencies -->CMakelist.txt, package.xml*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class cw01_publisher : public rclcpp::Node
{
    public:
    int input_a, input_b, input_c;
    cw01_publisher()
    : Node("cw01_publisher")
    {
        cw01_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("topic", 10);
        timer_=this->create_wall_timer(500ms, std::bind(&cw01_publisher::cw01_timer_callback, this));
    }
    private:
    void cw01_timer_callback()
    {
        auto message_cw01 = std_msgs::msg::Int32MultiArray();

        // scanf("%d %d %d", &input_a, &input_b, &input_c);
        
        message_cw01.data.push_back(input_a);
        message_cw01.data.push_back(input_b);
        message_cw01.data.push_back(input_c);

        RCLCPP_INFO(this->get_logger(), "각도: %d, %d, %d", input_a, input_b, input_c);


        cw01_publisher_->publish(message_cw01);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr cw01_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto aa = std::make_shared<cw01_publisher>();
    int a, b, c;
    scanf("%d %d %d", &a, &b, &c);
    aa->input_a = a;
    aa->input_b = b;
    aa->input_c = c;
    rclcpp::spin(aa);
    rclcpp::shutdown();
    return 0;
}
