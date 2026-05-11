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
    int input_a, input_b;
    cw_pub()
    : Node("cw_pub"), count_(0)
    {
        cw_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("topic_cw01_aaa", 10);
        timer_=this->create_wall_timer(500ms, std::bind(&cw_pub::cw01_timer_callback, this));
    }
    private:
    void cw01_timer_callback()
    {
        auto message_cw01 = std_msgs::msg::Int32MultiArray();

        int a, b;
        scanf("%d %d", &a, &b);
        
        message_cw01.data.push_back(a);
        message_cw01.data.push_back(b);
        message_cw01.data.push_back(count_++);

        RCLCPP_INFO(this->get_logger(), "send : %d, %d, %zu", a, b, count_);


        cw_pub_->publish(message_cw01);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr cw_pub_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cw_pub>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
