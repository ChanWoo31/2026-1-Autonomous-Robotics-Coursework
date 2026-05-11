
/*package dependencies -->CMakelist.txt, package.xml*/

#include <stdio.h>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"


class velocity_publisher : public rclcpp::Node
{
    public:
    float set_velocity_;
    velocity_publisher(): Node("velocity_publisher")
    {
        velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/set_velocity", 10
        );
    }

    void publish_vel(float vel)
    {
        auto msg = std_msgs::msg::Float64();
        float normalized_vel = vel * 0.22;
        msg.data = normalized_vel;
        velocity_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "속도 설정: %f", msg.data);
    }

    private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<velocity_publisher>();

    // 스레드 설정
    std::thread spin_thread([&node]() {
        rclcpp::spin(node);
    });

    float vel;

    while (rclcpp::ok()){
        printf("속도 입력(0~1 사이 입력) : ");
        
        if (scanf("%f", &vel) == 1){
            node->publish_vel(vel);
        } else {
            int c;
            while ((c = getchar()) != '\n' && c != EOF) {}
            printf("다시 입력:\n");
        }
    }

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}





