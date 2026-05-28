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
    MazeExplorer() : Node("maze_explorer")
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&MazeExplorer::mapCallback, this, _1)
        );

        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        watchdog_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MazeExplorer::watchdogCallback, this)
        );
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    GoalHandleNav2::SharedPtr active_goal_handle_;

    bool goal_in_progress_ = false;
    bool canceling_goal_ = false;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;

    double best_goal_distance_ = std::numeric_limits<double>::max();
    rclcpp::Time last_progress_time_;

    double start_x_ = 0.0;
    double start_y_ = 0.0;
    double start_yaw_ = 0.0;
    double max_forward_projection_ = 0.0;
    bool is_start_position_saved_ = false;

    struct Point2D
    {
        double x;
        double y;
    };

    std::vector<Point2D> blacklist_;

    const double BLACKLIST_RADIUS = 0.15;

    const double GOAL_CLEARANCE = 0.05;
    const double PATH_CLEARANCE = 0.03;
    const double GOAL_STANDOFF = 0.16;
    const double GOAL_SEARCH_RADIUS = 0.22;
    const double PREFERRED_GOAL_CLEARANCE = 0.12;

    const double MIN_GOAL_DISTANCE = 0.45;
    const double MIN_FINAL_GOAL_DISTANCE = 0.40;
    const double MIN_FINAL_GOAL_CLEARANCE = 0.10;

    const double START_LINE_MARGIN = 0.05;
    const double BACKTRACK_GOAL_ALLOWANCE = 0.25;
    const double BACKTRACK_CANCEL_DISTANCE = 0.35;

    const double STUCK_GOAL_IMPROVEMENT = 0.03;
    const double STUCK_TIMEOUT = 8.0;

    const int MIN_FRONTIER_CLUSTER_SIZE = 10;

    double current_goal_x_ = 0.0;
    double current_goal_y_ = 0.0;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        if (!updateRobotPose()) {
            return;
        }

        saveStartPoseOnce();
        updateForwardProgress();

        if (goal_in_progress_) {
            return;
        }

        geometry_msgs::msg::PoseStamped next_goal;
        if (findBestFrontier(msg, next_goal)) {
            sendGoal(next_goal);
        } else {
            RCLCPP_WARN(this->get_logger(), "선택 가능한 frontier를 찾지 못했습니다.");
        }
    }

    bool updateRobotPose()
    {
        geometry_msgs::msg::TransformStamped t;

        try {
            t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            return false;
        }

        robot_x_ = t.transform.translation.x;
        robot_y_ = t.transform.translation.y;

        const double qx = t.transform.rotation.x;
        const double qy = t.transform.rotation.y;
        const double qz = t.transform.rotation.z;
        const double qw = t.transform.rotation.w;

        robot_yaw_ = std::atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        );

        return true;
    }

    void saveStartPoseOnce()
    {
        if (!is_start_position_saved_) {
            start_x_ = robot_x_;
            start_y_ = robot_y_;
            start_yaw_ = robot_yaw_;
            max_forward_projection_ = 0.0;
            is_start_position_saved_ = true;

            RCLCPP_INFO(
                this->get_logger(),
                "시작 위치 저장 완료: x=%.2f, y=%.2f, yaw=%.2f",
                start_x_,
                start_y_,
                start_yaw_
            );
        }
    }

    double forwardProjection(double wx, double wy) const
    {
        const double forward_x = std::cos(start_yaw_);
        const double forward_y = std::sin(start_yaw_);

        return ((wx - start_x_) * forward_x) + ((wy - start_y_) * forward_y);
    }

    void updateForwardProgress()
    {
        if (!is_start_position_saved_) {
            return;
        }

        max_forward_projection_ = std::max(
            max_forward_projection_,
            forwardProjection(robot_x_, robot_y_)
        );
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

        return mx >= 0 &&
               mx < static_cast<int>(map->info.width) &&
               my >= 0 &&
               my < static_cast<int>(map->info.height);
    }

    bool hasObstacleWithin(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        int mx,
        int my,
        double clearance)
    {
        const int width = static_cast<int>(map->info.width);
        const int height = static_cast<int>(map->info.height);

        const int radius_cells = std::max(
            1,
            static_cast<int>(std::ceil(clearance / map->info.resolution))
        );

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

    double nearestObstacleDistance(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        int mx,
        int my,
        double max_distance)
    {
        const int width = static_cast<int>(map->info.width);
        const int height = static_cast<int>(map->info.height);

        const int radius_cells = std::max(
            1,
            static_cast<int>(std::ceil(max_distance / map->info.resolution))
        );

        double best_distance = max_distance;

        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                if (dx == 0 && dy == 0) {
                    continue;
                }

                const int cx = mx + dx;
                const int cy = my + dy;

                const double distance = std::hypot(
                    dx * map->info.resolution,
                    dy * map->info.resolution
                );

                if (distance > best_distance) {
                    continue;
                }

                if (cx < 0 || cx >= width || cy < 0 || cy >= height) {
                    best_distance = distance;
                    continue;
                }

                const int value = map->data[cy * width + cx];

                if (value >= 50) {
                    best_distance = distance;
                }
            }
        }

        return best_distance;
    }

    bool findCenteredReachableGoalCell(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        const std::vector<int>& reachable_distances,
        int seed_mx,
        int seed_my,
        int& goal_mx,
        int& goal_my)
    {
        const int width = static_cast<int>(map->info.width);
        const int height = static_cast<int>(map->info.height);

        const int search_cells = std::max(
            1,
            static_cast<int>(std::ceil(GOAL_SEARCH_RADIUS / map->info.resolution))
        );

        bool found = false;
        double best_score = -std::numeric_limits<double>::max();

        for (int dy = -search_cells; dy <= search_cells; ++dy) {
            for (int dx = -search_cells; dx <= search_cells; ++dx) {
                const double offset_distance = std::hypot(
                    dx * map->info.resolution,
                    dy * map->info.resolution
                );

                if (offset_distance > GOAL_SEARCH_RADIUS) {
                    continue;
                }

                const int mx = seed_mx + dx;
                const int my = seed_my + dy;

                if (mx <= 0 || mx >= width - 1 || my <= 0 || my >= height - 1) {
                    continue;
                }

                const int index = my * width + mx;

                if (reachable_distances[index] < 0 || map->data[index] != 0) {
                    continue;
                }

                const double clearance = nearestObstacleDistance(
                    map,
                    mx,
                    my,
                    PREFERRED_GOAL_CLEARANCE
                );

                if (clearance < GOAL_CLEARANCE) {
                    continue;
                }

                const double score = clearance - (0.25 * offset_distance);

                if (!found || score > best_score) {
                    found = true;
                    best_score = score;
                    goal_mx = mx;
                    goal_my = my;
                }
            }
        }

        return found;
    }

    bool isFrontierCell(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        int mx,
        int my)
    {
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

    bool isTraversableCell(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        int mx,
        int my)
    {
        const int width = static_cast<int>(map->info.width);
        const int height = static_cast<int>(map->info.height);

        if (mx <= 0 || mx >= width - 1 || my <= 0 || my >= height - 1) {
            return false;
        }

        const int index = my * width + mx;

        return map->data[index] == 0 &&
               !hasObstacleWithin(map, mx, my, PATH_CLEARANCE);
    }

    bool computeReachableDistances(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        std::vector<int>& distances)
    {
        const int width = static_cast<int>(map->info.width);
        const int height = static_cast<int>(map->info.height);

        int start_mx = 0;
        int start_my = 0;

        if (!worldToMap(map, robot_x_, robot_y_, start_mx, start_my)) {
            return false;
        }

        distances.assign(width * height, -1);

        std::queue<int> queue;
        const int start_index = start_my * width + start_mx;

        distances[start_index] = 0;
        queue.push(start_index);

        const int dx[4] = {0, 0, -1, 1};
        const int dy[4] = {-1, 1, 0, 0};

        while (!queue.empty()) {
            const int current = queue.front();
            queue.pop();

            const int current_mx = current % width;
            const int current_my = current / width;

            for (int i = 0; i < 4; ++i) {
                const int nx = current_mx + dx[i];
                const int ny = current_my + dy[i];

                if (nx <= 0 || nx >= width - 1 || ny <= 0 || ny >= height - 1) {
                    continue;
                }

                const int neighbor_index = ny * width + nx;

                if (distances[neighbor_index] >= 0) {
                    continue;
                }

                if (!isTraversableCell(map, nx, ny)) {
                    continue;
                }

                distances[neighbor_index] = distances[current] + 1;
                queue.push(neighbor_index);
            }
        }

        return true;
    }

    bool findBestFrontier(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        geometry_msgs::msg::PoseStamped& goal)
    {
        const int width = static_cast<int>(map->info.width);
        const int height = static_cast<int>(map->info.height);
        const double resolution = map->info.resolution;
        const double origin_x = map->info.origin.position.x;
        const double origin_y = map->info.origin.position.y;

        double min_distance = std::numeric_limits<double>::max();
        bool frontier_found = false;

        int best_mx = 0;
        int best_my = 0;
        double best_yaw = 0.0;

        const int dx[4] = {0, 0, -1, 1};
        const int dy[4] = {-1, 1, 0, 0};

        std::vector<bool> visited(width * height, false);
        std::vector<int> reachable_distances;

        if (!computeReachableDistances(map, reachable_distances)) {
            return false;
        }

        for (int my = 1; my < height - 1; ++my) {
            for (int mx = 1; mx < width - 1; ++mx) {
                const int index = my * width + mx;

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

                        if (nx <= 0 || nx >= width - 1 || ny <= 0 || ny >= height - 1) {
                            continue;
                        }

                        const int neighbor_index = ny * width + nx;

                        if (visited[neighbor_index] || !isFrontierCell(map, nx, ny)) {
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

                    const double frontier_wx =
                        origin_x + (frontier_mx + 0.5) * resolution;

                    const double frontier_wy =
                        origin_y + (frontier_my + 0.5) * resolution;

                    const double frontier_dist =
                        std::hypot(frontier_wx - robot_x_, frontier_wy - robot_y_);

                    if (frontier_dist < MIN_GOAL_DISTANCE) {
                        continue;
                    }

                    const double unit_x = (frontier_wx - robot_x_) / frontier_dist;
                    const double unit_y = (frontier_wy - robot_y_) / frontier_dist;

                    const double candidate_wx =
                        frontier_wx - (unit_x * GOAL_STANDOFF);

                    const double candidate_wy =
                        frontier_wy - (unit_y * GOAL_STANDOFF);

                    int candidate_mx = 0;
                    int candidate_my = 0;

                    if (!worldToMap(map, candidate_wx, candidate_wy, candidate_mx, candidate_my)) {
                        continue;
                    }

                    int goal_mx = 0;
                    int goal_my = 0;

                    if (!findCenteredReachableGoalCell(
                            map,
                            reachable_distances,
                            candidate_mx,
                            candidate_my,
                            goal_mx,
                            goal_my)) {
                        continue;
                    }

                    const int goal_index = goal_my * width + goal_mx;

                    const double goal_wx =
                        origin_x + (goal_mx + 0.5) * resolution;

                    const double goal_wy =
                        origin_y + (goal_my + 0.5) * resolution;

                    const double final_goal_distance =
                        std::hypot(goal_wx - robot_x_, goal_wy - robot_y_);

                    if (final_goal_distance < MIN_FINAL_GOAL_DISTANCE) {
                        continue;
                    }

                    const double final_goal_clearance =
                        nearestObstacleDistance(map, goal_mx, goal_my, 0.30);

                    if (final_goal_clearance < MIN_FINAL_GOAL_CLEARANCE) {
                        continue;
                    }

                    bool is_blacklisted = false;

                    for (const auto& bp : blacklist_) {
                        if (std::hypot(goal_wx - bp.x, goal_wy - bp.y) < BLACKLIST_RADIUS) {
                            is_blacklisted = true;
                            break;
                        }
                    }

                    if (is_blacklisted) {
                        continue;
                    }

                    const double target_yaw =
                        std::atan2(frontier_wy - goal_wy, frontier_wx - goal_wx);

                    const double angle_diff = std::abs(std::atan2(
                        std::sin(target_yaw - robot_yaw_),
                        std::cos(target_yaw - robot_yaw_)
                    ));

                    const double projection = forwardProjection(goal_wx, goal_wy);

                    if (projection < START_LINE_MARGIN) {
                        continue;
                    }

                    if (projection < max_forward_projection_ - BACKTRACK_GOAL_ALLOWANCE) {
                        continue;
                    }

                    const double path_distance =
                        reachable_distances[goal_index] * resolution;

                    const double clearance =
                        nearestObstacleDistance(map, goal_mx, goal_my, 0.30);

                    double score = 0.7 * path_distance;

                    score -= 1.2 * clearance;
                    score -= 0.25 * projection;

                    if (angle_diff > 1.57) {
                        score += 0.3;
                    }

                    if (score < min_distance) {
                        min_distance = score;
                        best_mx = goal_mx;
                        best_my = goal_my;
                        best_yaw = target_yaw;
                        frontier_found = true;
                    }
                }
            }
        }

        if (!frontier_found) {
            return false;
        }

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
            min_distance
        );

        return true;
    }

    void sendGoal(const geometry_msgs::msg::PoseStamped& goal_pose)
    {
        if (!nav_client_->wait_for_action_server(std::chrono::seconds(3))) {
            return;
        }

        goal_pub_->publish(goal_pose);

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goal_pose;

        auto send_goal_options =
            rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            [this](const GoalHandleNav2::SharedPtr& goal_handle) {
                if (!goal_handle) {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Nav2가 목표를 거부했습니다. blacklist 추가"
                    );

                    blacklist_.push_back({current_goal_x_, current_goal_y_});
                    goal_in_progress_ = false;
                    active_goal_handle_.reset();
                    return;
                }

                active_goal_handle_ = goal_handle;
            };

        send_goal_options.result_callback =
            std::bind(&MazeExplorer::resultCallback, this, _1);

        goal_in_progress_ = true;
        canceling_goal_ = false;

        best_goal_distance_ =
            std::hypot(current_goal_x_ - robot_x_, current_goal_y_ - robot_y_);

        last_progress_time_ = this->now();

        nav_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void watchdogCallback()
    {
        if (!goal_in_progress_ || canceling_goal_) {
            return;
        }

        if (!updateRobotPose()) {
            return;
        }

        updateForwardProgress();

        const double current_projection = forwardProjection(robot_x_, robot_y_);

        if (current_projection < max_forward_projection_ - BACKTRACK_CANCEL_DISTANCE) {
            RCLCPP_WARN(
                this->get_logger(),
                "입구 방향으로 되돌아가는 경로 감지. 현재 목표 취소: 현재진행=%.2f, 최대진행=%.2f",
                current_projection,
                max_forward_projection_
            );

            blacklist_.push_back({current_goal_x_, current_goal_y_});
            canceling_goal_ = true;

            if (active_goal_handle_) {
                nav_client_->async_cancel_goal(active_goal_handle_);
            } else {
                goal_in_progress_ = false;
                canceling_goal_ = false;
            }

            return;
        }

        const double goal_distance =
            std::hypot(current_goal_x_ - robot_x_, current_goal_y_ - robot_y_);

        if (goal_distance + STUCK_GOAL_IMPROVEMENT < best_goal_distance_) {
            best_goal_distance_ = goal_distance;
            last_progress_time_ = this->now();
            return;
        }

        if ((this->now() - last_progress_time_).seconds() < STUCK_TIMEOUT) {
            return;
        }

        RCLCPP_WARN(
            this->get_logger(),
            "목표 접근 실패. 현재 목표 취소 및 blacklist 추가: x=%.2f, y=%.2f, 남은거리=%.2f",
            current_goal_x_,
            current_goal_y_,
            goal_distance
        );

        blacklist_.push_back({current_goal_x_, current_goal_y_});
        canceling_goal_ = true;

        if (active_goal_handle_) {
            nav_client_->async_cancel_goal(active_goal_handle_);
        } else {
            goal_in_progress_ = false;
            canceling_goal_ = false;
        }
    }

    void resultCallback(const GoalHandleNav2::WrappedResult& result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(
                this->get_logger(),
                "목표 도착: x=%.2f, y=%.2f",
                current_goal_x_,
                current_goal_y_
            );

            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        } else if (canceling_goal_) {
            RCLCPP_WARN(this->get_logger(), "정체된 목표 취소 완료");
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        } else {
            RCLCPP_WARN(
                this->get_logger(),
                "목표 실패. blacklist 추가: x=%.2f, y=%.2f",
                current_goal_x_,
                current_goal_y_
            );

            blacklist_.push_back({current_goal_x_, current_goal_y_});
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        active_goal_handle_.reset();
        canceling_goal_ = false;
        goal_in_progress_ = false;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MazeExplorer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}