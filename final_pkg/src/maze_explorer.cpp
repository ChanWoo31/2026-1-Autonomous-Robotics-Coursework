#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vector>
#include <cmath>
#include <limits>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<NavigateToPose>;

using std::placeholders::_1;

class MazeExplorer : public rclcpp::Node
{
    public:
    MazeExplorer() : Node("maze_explorer") {
        // slam 맵
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&MazeExplorer::mapCallback, this, _1)
        );
        // 오돔
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MazeExplorer::odomCallback, this, _1)
        );
        // nav2 액션 클라이언트
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    }

    private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    
    bool goal_in_progress_ = false;
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;

    struct Point2D {
        double x;
        double y;
    };
    std::vector<Point2D> blacklist_;

    const double BLACK_LIST_RADIUS = 0.3;

    double current_goal_x_ = 0.0;
    double current_goal_y_ = 0.0;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (goal_in_progress_) return;

        // 여기 로직 추가
        geometry_msgs::msg::PoseStamped next_goal;

        if (findBestFrontier(msg, next_goal)){
            sendGoal(next_goal);
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
    }

    bool findBestFrontier(const nav_msgs::msg::OccupancyGrid::SharedPtr& map, geometry_msgs::msg::PoseStamped& goal) {
        // 프론티어
        int width = map->info.width;
        int height = map->info.height;
        double resolution = map->info.resolution;
        double origin_x = map->info.origin.position.x;
        double origin_y = map->info.origin.position.y;

        double min_distance = std::numeric_limits<double>::max();
        bool frontier_found = false;
        int best_mx = 0, best_my = 0;

        //상하좌우 탐색 오프셋
        int dx[4] = {0, 0, -1, 1};
        int dy[4] = {-1, 1, 0, 0};

        // 배열 분석
        for (int my=1; my<height-1; ++my){
            for (int mx=1; mx<width-1; ++mx){
                int index = my * width + mx;

                // 현재 칸이 지나갈 수 있는 빈 공간인지 => 0
                if (map->data[index] == 0){
                    bool is_frontier = false;

                    // 아직 안 가본 곳이 있는지 => -1
                    for (int i=0; i<4; ++i){
                        int nx = mx + dx[i];
                        int ny = my + dy[i];
                        int neighbor_index = ny * width + nx;

                        if (map->data[neighbor_index] == -1){
                            is_frontier = true;
                            break;
                        }
                    }

                    // 프론티어 찾았다면, 내 위치와 가장 가까운 녀석인지 계산
                    if (is_frontier){
                        // 픽셀 좌표(mx, my)를 실제 미로의 미터(m) 단위 세계 좌표(wx, wy)로 변환
                        double wx = origin_x + (mx + 0.5) * resolution;
                        double wy = origin_y + (my + 0.5) * resolution;

                        // 블랙리스트 검사
                        bool is_blacklisted = false;
                        for (const auto& bp : blacklist_) {
                            if (std::hypot(wx - bp.x, wy - bp.y) < BLACK_LIST_RADIUS){
                                is_blacklisted = true;
                                break;
                            }
                        }

                        // 블랙리스트 걸렸으면 다음 픽셀로.
                        if (is_blacklisted) continue;

                        // 로봇과 프론티어 거리 계산
                        double dist = std::hypot(wx - robot_x_, wy - robot_y_);

                        if (dist > 0.3 && dist < min_distance){
                            min_distance = dist;
                            best_mx = mx;
                            best_my = my;
                            frontier_found = true;
                        }
                    }
                }
            }
        }

        if (frontier_found){
            goal.header.frame_id = "map";
            goal.header.stamp = this->now();
            goal.pose.position.x = origin_x + (best_mx + 0.5) * resolution;
            goal.pose.position.y = origin_y + (best_my + 0.5) * resolution;
            goal.pose.orientation.w = 1.0;

            current_goal_x_ = goal.pose.position.x;
            current_goal_y_ = goal.pose.position.y;
            return true;
        }

        return false;
        
    }

    void sendGoal(const geometry_msgs::msg::PoseStamped& goal_pose) {
        if (!nav_client_->wait_for_action_server(std::chrono::seconds(3))){
            RCLCPP_ERROR(this->get_logger(), "에러");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goal_pose;
        
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&MazeExplorer::resultCallback, this, _1);

        nav_client_->async_send_goal(goal_msg, send_goal_options);
        goal_in_progress_ = true;
    }

    // Nav2 액션 결과 처리
    void resultCallback(const GoalHandleNav2::WrappedResult& result) {
        goal_in_progress_ = false;
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "성공");
            // std::this_thread::sleep_for(std::chrono::milliseconds(500));
        } else{
            RCLCPP_INFO(this->get_logger(), "실패. 실패 위치 저장. X: %.2f, Y: %.2f", current_goal_x_, current_goal_y_);
            blacklist_.push_back({current_goal_x_, current_goal_y_});
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        goal_in_progress_ = false; // false로 플래그 줘서, mapCallback이 알아서 프론티어 탐ㅎ색하도록.
    }

};

int main(int argc, char**argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MazeExplorer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}









