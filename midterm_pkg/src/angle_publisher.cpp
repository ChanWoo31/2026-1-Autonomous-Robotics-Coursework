
/*package dependencies -->CMakelist.txt, package.xml*/

#include <stdio.h>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

class angle_publisher : public rclcpp::Node
{
    public:
    int angle_1, angle_2;

    angle_publisher(): Node("angle_publisher")
    {
        angle_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "/lidar_angle", 10
        );
    }

    void publish_ang(int ang1, int ang2)
    {
        auto msg =std_msgs::msg::Int32MultiArray();
        msg.data.push_back(ang1);
        msg.data.push_back(ang2);
        angle_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "라이다 각도 설정: %d %d", ang1, ang2);
    }

    private:
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr angle_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<angle_publisher>();

    // 스레드 설정
    std::thread spin_thread([&node]() {
        rclcpp::spin(node);
    });

    int ang1, ang2;

    while (rclcpp::ok()){
        printf("각도 2개 입력(0~720) : ");

        if (scanf("%d %d", &ang1, &ang2) == 2){
            node->publish_ang(ang1, ang2);
        }
    }
}

