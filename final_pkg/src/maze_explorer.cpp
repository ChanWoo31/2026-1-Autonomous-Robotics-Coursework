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
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&MazeExplorer::mapCallback, this, _1)
        );
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MazeExplorer::odomCallback, this, _1)
        );
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    }

    private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    
    bool goal_in_progress_ = false;
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;

    // 시작 위치 저장용
    double start_x_ = 0.0;
    double start_y_ = 0.0;
    bool is_start_position_saved_ = false;

    struct Point2D {
        double x;
        double y;
    };
    std::vector<Point2D> blacklist_;

    const double BLACK_LIST_RADIUS = 0.30;

    double current_goal_x_ = 0.0;
    double current_goal_y_ = 0.0;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (goal_in_progress_) return;

        geometry_msgs::msg::PoseStamped next_goal;

        if (findBestFrontier(msg, next_goal)){
            sendGoal(next_goal);
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;

        // 시작 위치 최초 1회 저장
        if (!is_start_position_saved_) {
            start_x_ = robot_x_;
            start_y_ = robot_y_;
            is_start_position_saved_ = true;
        }

        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        robot_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    }

    bool findBestFrontier(const nav_msgs::msg::OccupancyGrid::SharedPtr& map, geometry_msgs::msg::PoseStamped& goal) {
        int width = map->info.width;
        int height = map->info.height;
        double resolution = map->info.resolution;
        double origin_x = map->info.origin.position.x;
        double origin_y = map->info.origin.position.y;

        double min_distance = std::numeric_limits<double>::max();
        bool frontier_found = false;
        int best_mx = 0, best_my = 0;
        double best_yaw = 0.0;

        int dx[4] = {0, 0, -1, 1};
        int dy[4] = {-1, 1, 0, 0};

        // 로봇이 출발점으로부터 얼마나 이동했는지 계산
        double dist_from_start = std::hypot(robot_x_ - start_x_, robot_y_ - start_y_);

        for (int my=1; my<height-1; ++my){
            for (int mx=1; mx<width-1; ++mx){
                int index = my * width + mx;

                if (map->data[index] == 0){

                    bool is_near_wall = false;
                    for (int dy_wall = -4; dy_wall <= 4; ++dy_wall) {
                        for (int dx_wall = -4; dx_wall <= 4; ++dx_wall) {
                            int check_x = mx + dx_wall;
                            int check_y = my + dy_wall;
                            if (check_x >= 0 && check_x < width && check_y >= 0 && check_y < height) {
                                if (map->data[check_y * width + check_x] == 100) {
                                    is_near_wall = true;
                                    break;
                                }
                            }
                        }
                        if (is_near_wall) break;
                    }
                    // 벽 근처 10cm 이내면 이 프론티어는 버리고 다음 픽셀 검사!
                    if (is_near_wall) continue;

                    bool is_frontier = false;

                    for (int i=0; i<4; ++i){
                        int nx = mx + dx[i];
                        int ny = my + dy[i];
                        int neighbor_index = ny * width + nx;

                        if (map->data[neighbor_index] == -1){
                            is_frontier = true;
                            break;
                        }
                    }

                    if (is_frontier){
                        double wx = origin_x + (mx + 0.5) * resolution;
                        double wy = origin_y + (my + 0.5) * resolution;

                        bool is_blacklisted = false;
                        for (const auto& bp : blacklist_) {
                            if (std::hypot(wx - bp.x, wy - bp.y) < BLACK_LIST_RADIUS){
                                is_blacklisted = true;
                                break;
                            }
                        }

                        if (is_blacklisted) continue;

                        double target_yaw = std::atan2(wy - robot_y_, wx - robot_x_);
                        double angle_diff = std::abs(target_yaw - robot_yaw_);
                        if (angle_diff > M_PI) {
                            angle_diff = 2.0 * M_PI - angle_diff;
                        }
                        
                        double real_dist = std::hypot(wx - robot_x_, wy - robot_y_);

                        // [수정] 좁은 미로를 위해 도착 인정 범위 축소 (25cm -> 15cm)
                        if (real_dist < 0.15) {
                            continue;
                        }

                        // [핵심] 동적 페널티 점수 계산
                        double score = real_dist;
                        if (angle_diff > 1.57) { // 로봇 등 뒤에 있는 프론티어라면
                            if (dist_from_start < 0.8) {
                                score += 10.0; // 출발점 근처(0.8m 이내)면 뒤로 못 가게 페널티 폭탄
                            } else {
                                score += 0.2;  // 미로 깊숙한 곳이면 방향 전환 비용 정도만 가볍게 부과
                            }
                        }

                        if (score < min_distance){
                            min_distance = score;
                            best_mx = mx;
                            best_my = my;
                            best_yaw = target_yaw;
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
            goal.pose.orientation.x = 0.0;
            goal.pose.orientation.y = 0.0;
            goal.pose.orientation.z = std::sin(best_yaw * 0.5);
            goal.pose.orientation.w = std::cos(best_yaw * 0.5);

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

        goal_pub_->publish(goal_pose);

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goal_pose;
        
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&MazeExplorer::resultCallback, this, _1);

        nav_client_->async_send_goal(goal_msg, send_goal_options);
        goal_in_progress_ = true;
    }

    void resultCallback(const GoalHandleNav2::WrappedResult& result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "성공");
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        } else {
            RCLCPP_INFO(this->get_logger(), "실패. 실패 위치 저장. X: %.2f, Y: %.2f", current_goal_x_, current_goal_y_);
            blacklist_.push_back({current_goal_x_, current_goal_y_});
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        goal_in_progress_ = false; 
    }
};

int main(int argc, char**argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MazeExplorer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}