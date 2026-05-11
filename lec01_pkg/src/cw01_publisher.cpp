#include <chrono>
#include <functional>
#include <memory>
#include <string>

/*package dependencies -->CMakelist.txt, package.xml*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std::chrono_literals;

// ros2 Node를 상속받는다.
class cw01_publisher : public rclcpp::Node
{
    public:
    int input_a, input_b;
    cw01_publisher()    // 생성자 선언.
    // 멤버 초기화 리스트 (준비 단계). Node()는 괄호 안에 이름으로 활동할 노드라고 등록하는 것.
    // 즉, 부모 클래스 초기화를 의미.
    // count_(0)은 변수 초기화.
    : Node("cw01_publisher"), count_(0)
    {
        // 생성자 본문 Node 상속 받은 후에 이 아래 함수를 쓸 수 있음.
        // this-> 로 이 노드가 가진 create_publisher 기능을 사용하겠다를 알려줌.
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

        RCLCPP_INFO(this->get_logger(), "%d, %d ... hello world count %zu", input_a, input_b, count_);


        cw01_publisher_->publish(message_cw01);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr cw01_publisher_;
    size_t count_;
};

// argc는 몇 개의 단어를 입력했는지(개수). argv는 그 단어들의 실제 내용이 무엇인지(값).
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv); // 터미널에서 ros2 run my_pkg my_node --ros-args -p param:=10 이라고 쳤으면 이걸 알아서 rclpy::init이 초기 설정을 해줌.
    auto aa = std::make_shared<cw01_publisher>(); // 스마트한 포인터
    int a, b;
    scanf("%d %d", &a, &b);
    aa->input_a = a;
    aa->input_b = b;
    rclcpp::spin(aa);
    rclcpp::shutdown();
    return 0;
}
