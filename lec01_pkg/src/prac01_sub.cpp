#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using std::placeholders::_1;

class prac01_sub : public rclcpp::Node
{
    public:
    prac01_sub(): Node("prac01_sub")
    {
        prac01_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "topic_prac01_aa", 10, std::bind(&prac01_sub::prac01_topic_callback, this, _1)
        );
    }

    private:
    void prac01_topic_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) const
    {
        int q1 = msg->data[0];
        int q2 = msg->data[1];
        int q3 = msg->data[2];

        bool is_all_safe = true;

        if (q1 >= 150) {
            RCLCPP_WARN(this->get_logger(), "Warn! over joint_1 value: %d", q1);
            is_all_safe = false;
        } 
        if (q2 >= 150) {
            RCLCPP_WARN(this->get_logger(), "Warn! over joint_2 value: %d", q2);
            is_all_safe = false;
        } 
        if (q3 >= 150) {
            RCLCPP_WARN(this->get_logger(), "Warn! over joint_3 value: %d", q3);
            is_all_safe = false;
        } 
        if (is_all_safe) {
            RCLCPP_INFO(this->get_logger(), "joint_1: %d, joint_2: %d, joint_3: %d", q1, q2, q3);
        }

    }

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr prac01_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto a = std::make_shared<prac01_sub>();
    rclcpp::spin(a);
    rclcpp::shutdown();
    return 0;
}