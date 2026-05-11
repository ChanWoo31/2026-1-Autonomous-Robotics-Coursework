#include <chrono>
#include <functional>
#include <memory>
#include <string>

/*package dependencies -->CMakelist.txt, package.xml*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std::chrono_literals;

class cw_pub : public rclcpp::Node
{
    public:
    // int input_a, input_b, input_c, input_d;
    cw_pub()
    : Node("cw_pub")
    {
        cw_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("topic", 10);
        timer_=this->create_wall_timer(500ms, std::bind(&cw_pub::cw01_timer_callback, this));
    }
    private:
    void cw01_timer_callback()
    {
        auto message = std_msgs::msg::Int32MultiArray();

        int a, b, c, d;
        scanf("%d %d %d %d", &a, &b, &c, &d);
        
        message.data.push_back(a);
        message.data.push_back(b);
        message.data.push_back(c);
        message.data.push_back(d);

        RCLCPP_INFO(this->get_logger(), "send : %d, %d, %d, %d", a, b, c, d);


        cw_pub_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr cw_pub_;
};

int main(int argc, char * argv[])
{
    // int a, b, c, d;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cw_pub>();
    // scanf("%d %d %d %d", &a, &b, &c, &d);
    // node->input_a = a;
    // node->input_b = b;
    // node->input_c = c;
    // node->input_d = d;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
