#include "rclcpp/rclcpp.hpp"
#include "more_interfaces/srv/add_two_ints.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;
int main(int argc, char **argv) // 노드 메인 함수
{
rclcpp::init(argc, argv); // 노드명 초기화
if (argc != 3) {
RCLCPP_INFO(rclcpp::get_logger(" rclcpp "), " usage: add_two_ints_client X Y");
return 1;
}
// add_two_ints_client라는 이름의 노드 선언
std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
// 해당 노드에 add_two_ints를 추가
rclcpp::Client<more_interfaces::srv::AddTwoInts>::SharedPtr client =
node->create_client<more_interfaces::srv::AddTwoInts>("add_two_ints");
// request에 넣을 값 두 개를 세팅
auto request = std::make_shared<more_interfaces::srv::AddTwoInts::Request>();
request->a = atoll(argv[1]);
request->b = atoll(argv[2]);

while (!client->wait_for_service(1s)) { // 1초를 기다린 후 서비스를 찾는다
if (!rclcpp::ok()) { // 취소 시, 아래 문구 출력 및 코드 종료
RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), " Interrupted while waiting for the service. Exiting. ");
return 0;
}
RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
}
auto result = client->async_send_request(request);
// 서버로부터 result를 수신할 때까지 기다린다
if (rclcpp::spin_until_future_complete(node, result) ==
rclcpp::FutureReturnCode::SUCCESS)
{
// 서비스를 받는 것에 성공했다면 결과를 출력한다.
RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum); } else {
// 서비스를 받는 것에 실패했다면 아래 문구를 출력한다.
RCLCPP_ERROR(rclcpp::get_logger(" rclcpp "), " Failed to call service add_two_ints ");
}
rclcpp::shutdown();
return 0;
}
