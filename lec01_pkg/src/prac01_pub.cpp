#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std::chrono_literals;

class prac01_pub : public rclcpp::Node
{
    public:

    int input_q1, input_q2, input_q3;

    prac01_pub(): Node("prac01_pub"), count_(0)
    {
        prac01_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("topic_prac01_aa", 10);
        timer_ = this->create_wall_timer(1000ms, std::bind(&prac01_pub::prac01_timer_callback, this));
    }

    private:
    void prac01_timer_callback()
    {
        auto message_prac01 = std_msgs::msg::Int32MultiArray();

        this->input_q1 += 10;
        this->input_q3 -= 10;
        message_prac01.data.push_back(input_q1);
        message_prac01.data.push_back(input_q2);
        message_prac01.data.push_back(input_q3);

        RCLCPP_INFO(this->get_logger(), "data publishing...");

        prac01_pub_->publish(message_prac01);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr prac01_pub_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    int q1, q2, q3;
    rclcpp::init(argc, argv);
    auto aa = std::make_shared<prac01_pub>();
    printf("Enter q1, q2, q3: ");
    scanf("%d %d %d", &q1, &q2, &q3);
    aa->input_q1 = q1;
    aa->input_q2 = q2;
    aa->input_q3 = q3;
    rclcpp::spin(aa);
    rclcpp::shutdown();
    return 0;
}