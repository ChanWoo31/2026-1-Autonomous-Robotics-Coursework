#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vector>
#include <queue>
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
            "/global_costmap/costmap", 10, std::bind(&MazeExplorer::mapCallback, this, _1)
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
    const int MIN_CLUSTER_SIZE = 15; // 프론티어가 최소 픽셀 15개
    const int COST_THRESHOLD = 100; // 코스트맵 100 이상이면 안 가도록.

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

        double dist_from_start = std::hypot(robot_x_ - start_x_, robot_y_ - start_y_);

        // BFS 군집화를 위한 중복 방문 방지 배열
        std::vector<bool> visited(width * height, false);

        for (int my=1; my<height-1; ++my){
            for (int mx=1; mx<width-1; ++mx){
                int index = my * width + mx;

                // [Costmap 동기화] 주행 가능 구역(0)이면서, 동시에 위험 구역(COST_THRESHOLD 미만)인지 검사
                // Costmap에서 안전지대는 0, 벽은 254, 그 사이 Inflation 영역은 1~253
                if (map->data[index] == 0 && !visited[index]) {
                    
                    bool is_frontier = false;
                    for (int i=0; i<4; ++i){
                        int neighbor_index = (my + dy[i]) * width + (mx + dx[i]);
                        if (map->data[neighbor_index] == -1){ // 미지 영역과 닿아있다면 프론티어!
                            is_frontier = true;
                            break;
                        }
                    }

                    // 프론티어의 시작점을 찾았다면, BFS로 군집 크기를 측정합니다.
                    if (is_frontier) {
                        std::vector<int> cluster_indices;
                        std::queue<int> queue;

                        queue.push(index);
                        visited[index] = true;

                        while(!queue.empty()) {
                            int curr = queue.front();
                            queue.pop();
                            cluster_indices.push_back(curr);

                            int curr_x = curr % width;
                            int curr_y = curr / width;

                            for(int i=0; i<4; ++i) {
                                int nx = curr_x + dx[i];
                                int ny = curr_y + dy[i];
                                int n_idx = ny * width + nx;

                                if(nx > 0 && nx < width-1 && ny > 0 && ny < height-1 && !visited[n_idx]) {
                                    // 이웃 픽셀도 주행가능구역(0)이면서 미지 영역(-1)과 닿아있는 프론티어라면 같은 군집으로 묶음
                                    bool n_is_frontier = false;
                                    for(int j=0; j<4; ++j) {
                                        if(map->data[(ny + dy[j]) * width + (nx + dx[j])] == -1) {
                                            n_is_frontier = true;
                                            break;
                                        }
                                    }
                                    if(map->data[n_idx] == 0 && n_is_frontier) {
                                        visited[n_idx] = true;
                                        queue.push(n_idx);
                                    }
                                }
                            }
                        }

                        // [BFS 필터링] 판자 틈새 노이즈 방지: 군집 크기가 기준치보다 작으면 싹 다 버림!
                        if (static_cast<int>(cluster_indices.size()) < MIN_CLUSTER_SIZE) {
                            continue;
                        }

                        // 군집의 중심점(대표 픽셀)을 계산.
                        int representative_idx = cluster_indices[cluster_indices.size() / 2];
                        int r_mx = representative_idx % width;
                        int r_my = representative_idx / width;

                        double wx = origin_x + (r_mx + 0.5) * resolution;
                        double wy = origin_y + (r_my + 0.5) * resolution;

                        // [블랙리스트 검사]
                        bool is_blacklisted = false;
                        for (const auto& bp : blacklist_) {
                            if (std::hypot(wx - bp.x, wy - bp.y) < BLACK_LIST_RADIUS){
                                is_blacklisted = true;
                                break;
                            }
                        }
                        if (is_blacklisted) continue;

                        // 목적지 반경 4칸 안에 코스트가 위험수위(COST_THRESHOLD) 이상인 곳이 있으면 Nav2가 튕겨내기 전에 패스!
                        bool unsafe_cost = false;
                        for (int cy = -4; cy <= 4; ++cy) {
                            for (int cx = -4; cx <= 4; ++cx) {
                                int check_idx = (r_my + cy) * width + (r_mx + cx);
                                if (map->data[check_idx] >= COST_THRESHOLD || map->data[check_idx] == 100) {
                                    unsafe_cost = true;
                                    break;
                                }
                            }
                            if (unsafe_cost) break;
                        }
                        if (unsafe_cost) continue;

                        // 점수 계산 및 최적의 목적지 선정 (거리 및 방향 페널티)
                        double target_yaw = std::atan2(wy - robot_y_, wx - robot_x_);
                        double angle_diff = std::abs(target_yaw - robot_yaw_);
                        if (angle_diff > M_PI) angle_diff = 2.0 * M_PI - angle_diff;
                        
                        double real_dist = std::hypot(wx - robot_x_, wy - robot_y_);
                        if (real_dist < 0.20) continue; // 너무 코앞(20cm 이내)은 패스

                        double score = real_dist;
                        if (angle_diff > 1.57) { 
                            if (dist_from_start < 0.8) score += 10.0; // 출발점 근처 역주행 방지
                            else score += 0.2;
                        }

                        if (score < min_distance){
                            min_distance = score;
                            best_mx = r_mx;
                            best_my = r_my;
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