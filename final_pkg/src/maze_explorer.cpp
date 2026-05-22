#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <algorithm>
#include <chrono>
#include <thread>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using std::placeholders::_1;

class MazeExplorer : public rclcpp::Node
{
public:
    MazeExplorer() : Node("maze_explorer") {
        // TF2 초기화 (Odom 대신 절대 좌표를 구하기 위함)
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&MazeExplorer::mapCallback, this, _1)
        );
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    bool goal_in_progress_ = false;
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;

    double start_x_ = 0.0;
    double start_y_ = 0.0;
    double start_yaw_ = 0.0;
    bool is_start_position_saved_ = false;

    struct Point2D { double x; double y; };
    std::vector<Point2D> blacklist_;

    const double BLACK_LIST_RADIUS = 0.30;
    const double GOAL_CLEARANCE = 0.12;
    const double GOAL_STANDOFF = 0.18;
    const double MIN_GOAL_DISTANCE = 0.35;
    const int MIN_FRONTIER_CLUSTER_SIZE = 6;
    double current_goal_x_ = 0.0;
    double current_goal_y_ = 0.0;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (goal_in_progress_) return;

        // Odom 토픽 대신 TF를 통해 Map 상의 정확한 로봇 절대 좌표 획득
        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            return; // 아직 좌표 변환 트리가 준비 안 됐으면 무시
        }

        robot_x_ = t.transform.translation.x;
        robot_y_ = t.transform.translation.y;
        double qx = t.transform.rotation.x;
        double qy = t.transform.rotation.y;
        double qz = t.transform.rotation.z;
        double qw = t.transform.rotation.w;
        robot_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

        // 시작 위치 최초 1회 저장
        if (!is_start_position_saved_) {
            start_x_ = robot_x_;
            start_y_ = robot_y_;
            start_yaw_ = robot_yaw_;
            is_start_position_saved_ = true;
            RCLCPP_INFO(this->get_logger(), "진짜 시작 위치 저장 완료! (X:%.2f, Y:%.2f)", start_x_, start_y_);
        }

        geometry_msgs::msg::PoseStamped next_goal;
        if (findBestFrontier(msg, next_goal)){
            sendGoal(next_goal);
        }
    }

    bool worldToMap(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        double wx,
        double wy,
        int& mx,
        int& my)
    {
        mx = static_cast<int>((wx - map->info.origin.position.x) / map->info.resolution);
        my = static_cast<int>((wy - map->info.origin.position.y) / map->info.resolution);
        return mx >= 0 && mx < static_cast<int>(map->info.width) &&
               my >= 0 && my < static_cast<int>(map->info.height);
    }

    bool hasObstacleWithin(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        int mx,
        int my,
        double clearance)
    {
        const int width = static_cast<int>(map->info.width);
        const int height = static_cast<int>(map->info.height);
        const int radius_cells = std::max(1, static_cast<int>(std::ceil(clearance / map->info.resolution)));

        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                if (dx * dx + dy * dy > radius_cells * radius_cells) {
                    continue;
                }

                const int cx = mx + dx;
                const int cy = my + dy;
                if (cx < 0 || cx >= width || cy < 0 || cy >= height) {
                    return true;
                }

                const int value = map->data[cy * width + cx];
                if (value >= 50) {
                    return true;
                }
            }
        }
        return false;
    }

    bool isFrontierCell(const nav_msgs::msg::OccupancyGrid::SharedPtr& map, int mx, int my) {
        const int width = static_cast<int>(map->info.width);
        const int height = static_cast<int>(map->info.height);
        if (mx <= 0 || mx >= width - 1 || my <= 0 || my >= height - 1) {
            return false;
        }

        const int index = my * width + mx;
        if (map->data[index] != 0) {
            return false;
        }

        const int dx[4] = {0, 0, -1, 1};
        const int dy[4] = {-1, 1, 0, 0};
        for (int i = 0; i < 4; ++i) {
            const int neighbor_index = (my + dy[i]) * width + (mx + dx[i]);
            if (map->data[neighbor_index] == -1) {
                return true;
            }
        }
        return false;
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
        std::vector<bool> visited(width * height, false);

        for (int my=1; my<height-1; ++my){
            for (int mx=1; mx<width-1; ++mx){
                int index = my * width + mx;

                if (visited[index] || !isFrontierCell(map, mx, my)) {
                    continue;
                }

                std::vector<int> cluster;
                std::queue<int> frontier_queue;
                frontier_queue.push(index);
                visited[index] = true;

                while (!frontier_queue.empty()) {
                    const int current = frontier_queue.front();
                    frontier_queue.pop();
                    cluster.push_back(current);

                    const int current_mx = current % width;
                    const int current_my = current / width;
                    for (int i = 0; i < 4; ++i) {
                        const int nx = current_mx + dx[i];
                        const int ny = current_my + dy[i];
                        const int neighbor_index = ny * width + nx;
                        if (nx <= 0 || nx >= width - 1 || ny <= 0 || ny >= height - 1 ||
                            visited[neighbor_index] || !isFrontierCell(map, nx, ny)) {
                            continue;
                        }

                        visited[neighbor_index] = true;
                        frontier_queue.push(neighbor_index);
                    }
                }

                if (static_cast<int>(cluster.size()) < MIN_FRONTIER_CLUSTER_SIZE) {
                    continue;
                }

                for (const int frontier_index : cluster) {
                    const int frontier_mx = frontier_index % width;
                    const int frontier_my = frontier_index / width;
                    const double frontier_wx = origin_x + (frontier_mx + 0.5) * resolution;
                    const double frontier_wy = origin_y + (frontier_my + 0.5) * resolution;
                    const double frontier_dist = std::hypot(frontier_wx - robot_x_, frontier_wy - robot_y_);

                    if (frontier_dist < MIN_GOAL_DISTANCE) {
                        continue;
                    }

                    const double unit_x = (frontier_wx - robot_x_) / frontier_dist;
                    const double unit_y = (frontier_wy - robot_y_) / frontier_dist;
                    const double candidate_wx = frontier_wx - (unit_x * GOAL_STANDOFF);
                    const double candidate_wy = frontier_wy - (unit_y * GOAL_STANDOFF);

                    int candidate_mx = 0;
                    int candidate_my = 0;
                    if (!worldToMap(map, candidate_wx, candidate_wy, candidate_mx, candidate_my)) {
                        continue;
                    }

                    const int candidate_index = candidate_my * width + candidate_mx;
                    if (map->data[candidate_index] != 0 ||
                        hasObstacleWithin(map, candidate_mx, candidate_my, GOAL_CLEARANCE)) {
                        continue;
                    }

                    bool is_blacklisted = false;
                    for (const auto& bp : blacklist_) {
                        if (std::hypot(candidate_wx - bp.x, candidate_wy - bp.y) < BLACK_LIST_RADIUS){
                            is_blacklisted = true;
                            break;
                        }
                    }
                    if (is_blacklisted) {
                        continue;
                    }

                    const double target_yaw = std::atan2(frontier_wy - candidate_wy, frontier_wx - candidate_wx);
                    double angle_diff = std::abs(std::atan2(
                        std::sin(target_yaw - robot_yaw_),
                        std::cos(target_yaw - robot_yaw_)));

                    double score = std::hypot(candidate_wx - robot_x_, candidate_wy - robot_y_);

                    const double vec_x = candidate_wx - start_x_;
                    const double vec_y = candidate_wy - start_y_;
                    const double forward_x = std::cos(start_yaw_);
                    const double forward_y = std::sin(start_yaw_);
                    const double projection = vec_x * forward_x + vec_y * forward_y;

                    if (projection < -0.1) {
                        score += 10000.0;
                    }

                    if (angle_diff > 1.57) {
                        score += 0.4;
                    }

                    if (score < min_distance){
                        min_distance = score;
                        best_mx = candidate_mx;
                        best_my = candidate_my;
                        best_yaw = target_yaw;
                        frontier_found = true;
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
            RCLCPP_INFO(
                this->get_logger(),
                "다음 목표: x=%.2f, y=%.2f, score=%.2f",
                current_goal_x_,
                current_goal_y_,
                min_distance);
            return true;
        }
        return false;
    }

    void sendGoal(const geometry_msgs::msg::PoseStamped& goal_pose) {
        if (!nav_client_->wait_for_action_server(std::chrono::seconds(3))){
            return;
        }
        goal_pub_->publish(goal_pose);
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goal_pose;
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](const GoalHandleNav2::SharedPtr& goal_handle) {
                if (!goal_handle) {
                    RCLCPP_WARN(this->get_logger(), "Nav2가 목표를 거부했습니다. 블랙리스트에 추가합니다.");
                    blacklist_.push_back({current_goal_x_, current_goal_y_});
                    goal_in_progress_ = false;
                }
            };
        send_goal_options.result_callback = std::bind(&MazeExplorer::resultCallback, this, _1);
        goal_in_progress_ = true;
        nav_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void resultCallback(const GoalHandleNav2::WrappedResult& result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "목표 도착: x=%.2f, y=%.2f", current_goal_x_, current_goal_y_);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        } else {
            RCLCPP_WARN(
                this->get_logger(),
                "목표 실패. 블랙리스트 추가: x=%.2f, y=%.2f",
                current_goal_x_,
                current_goal_y_);
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
