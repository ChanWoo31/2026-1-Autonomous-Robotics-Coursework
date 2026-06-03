#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/time.h>
#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <algorithm>
#include <chrono>
#include <string>

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

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SensorDataQoS(),
            std::bind(&MazeExplorer::scanCallback, this, _1)
        );

        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        // Nav2의 cmd_vel을 safety filter로 보내기 위한 입력 토픽.
        // 실제 로봇으로 나가는 /cmd_vel은 safety filter가 publish해야 한다.
        escape_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_nav", 10);
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        watchdog_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MazeExplorer::watchdogCallback, this)
        );

        idle_retry_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&MazeExplorer::idleRetryCallback, this)
        );

        escape_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&MazeExplorer::escapeTimerCallback, this)
        );

        RCLCPP_INFO(this->get_logger(), "maze_explorer v12 no-return progress loaded");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr escape_cmd_pub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::TimerBase::SharedPtr idle_retry_timer_;
    rclcpp::TimerBase::SharedPtr escape_timer_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    rclcpp::Time last_scan_time_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    GoalHandleNav2::SharedPtr active_goal_handle_;

    bool goal_in_progress_ = false;
    bool canceling_goal_ = false;
    bool escape_active_ = false;
    rclcpp::Time escape_until_;
    double recovery_linear_speed_ = 0.0;
    double recovery_angular_speed_ = 0.0;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;

    double best_goal_distance_ = std::numeric_limits<double>::max();
    rclcpp::Time last_progress_time_;

    double start_x_ = 0.0;
    double start_y_ = 0.0;
    double start_yaw_ = 0.0;
    double max_forward_projection_ = 0.0;
    double max_start_distance_ = 0.0;
    double max_start_path_distance_ = 0.0;
    double current_start_path_distance_ = 0.0;
    bool is_start_position_saved_ = false;

    struct BlacklistEntry
    {
        double x;
        double y;
        rclcpp::Time expires_at;
    };

    std::vector<BlacklistEntry> blacklist_;

    const double BLACKLIST_RADIUS = 0.08;
    const double BLACKLIST_TTL_SUCCESS = 3.0;
    const double BLACKLIST_TTL_FAILURE = 5.0;
    const double BLACKLIST_TTL_CANCEL = 4.0;

    const double GOAL_CLEARANCE = 0.110;
    const double PATH_CLEARANCE = 0.085;
    const double PATH_BOTTLENECK_CLEARANCE = 0.105;
    const double GOAL_STANDOFF = 0.28;
    const double GOAL_SEARCH_RADIUS = 0.32;
    const double PREFERRED_GOAL_CLEARANCE = 0.18;
    const double PATH_BOTTLENECK_WEIGHT = 2.2;
    const double FRONTIER_CLUSTER_LENGTH_WEIGHT = 0.2;
    const double WALL_CLEARANCE_COMFORT = 0.16;
    const double WALL_CLEARANCE_PENALTY_WEIGHT = 9.0;

    const double MIN_GOAL_DISTANCE = 0.35;
    const double MIN_FINAL_GOAL_DISTANCE = 0.35;
    const double MIN_FINAL_GOAL_CLEARANCE = 0.105;

    const double START_LINE_MARGIN = 0.20;
    const double START_DIRECTION_WEIGHT = 1.1;
    const double EARLY_FRONT_GATE_DISTANCE = 1.20;
    const double MIN_EARLY_START_ALIGNMENT = 0.10;
    const double START_RETURN_ARM_DISTANCE = 0.85;
    const double START_RETURN_GOAL_RADIUS = 0.75;
    const double START_RETURN_CANCEL_RADIUS = 0.50;
    const double ENTRANCE_NO_RETURN_PATH_DISTANCE = 0.75;
    const double ENTRANCE_NO_RETURN_WORLD_RADIUS = 0.70;
    const double NO_RETURN_DISTANCE_BACKTRACK_ALLOWANCE = 0.20;
    const double NO_RETURN_GOAL_BACKTRACK_ALLOWANCE = 0.20;
    const double NO_RETURN_PROJECTION_ARM_DISTANCE = 0.60;
    const double NO_RETURN_PROJECTION_BACKTRACK_ALLOWANCE = 0.08;
    const double RELAXED_BACKTRACK_ALLOWANCE = 0.60;
    const double START_PATH_BACKTRACK_ALLOWANCE = 0.35;
    const double START_PATH_CANCEL_BACKTRACK = 0.60;
    const double START_PATH_CANCEL_ARM_DISTANCE = 0.90;
    const double START_PATH_PROGRESS_WEIGHT = 1.8;
    const double MIN_START_PATH_GOAL_DISTANCE = 0.60;
    const double KEEP_MOVING_MIN_DISTANCE = 0.45;
    const double KEEP_MOVING_MAX_DISTANCE = 1.20;
    const double KEEP_MOVING_MIN_EUCLIDEAN_DISTANCE = 0.32;
    const double KEEP_MOVING_SHORT_MIN_EUCLIDEAN_DISTANCE = 0.22;
    const double KEEP_MOVING_MIN_CLEARANCE = 0.115;
    const double KEEP_MOVING_RELAXED_CLEARANCE = 0.105;
    const double KEEP_MOVING_MIN_BOTTLENECK = 0.115;
    const double KEEP_MOVING_RELAXED_BOTTLENECK = 0.105;
    const double KEEP_MOVING_BLACKLIST_RADIUS = 0.08;
    const double KEEP_MOVING_BACKTRACK_ALLOWANCE = 0.25;
    const double OPEN_SPACE_RADIUS = 0.40;
    const double OPEN_SPACE_MIN_RATIO = 0.40;
    const double FORWARD_ALIGNMENT_WEIGHT = 1.20;
    const double OPEN_SPACE_WEIGHT = 1.80;
    const double OPEN_RAY_MAX_DISTANCE = 1.10;
    const double OPEN_RAY_LINE_CLEARANCE = 0.075;
    const double OPEN_RAY_GOAL_CLEARANCE = 0.105;
    const double LOOKAHEAD_PIVOT_CLEARANCE = 0.075;
    const double LOOKAHEAD_GOAL_CLEARANCE = 0.105;

    const double STUCK_GOAL_IMPROVEMENT = 0.03;
    const double STUCK_TIMEOUT = 5.0;
    const double ESCAPE_BACKUP_SPEED = 0.040;
    const double ESCAPE_BACKUP_DURATION = 0.40;
    const double ESCAPE_BACKUP_MIN_START_DISTANCE = 0.80;
    const int ESCAPE_BACKUP_MIN_FAILURES = 4;
    const double FORWARD_CREEP_SPEED = 0.060;
    const double FORWARD_CREEP_DURATION = 0.70;
    const double FORWARD_CREEP_PROJECTION_CHECK_DISTANCE = 0.30;
    const double FORWARD_CREEP_TURN_SPEED = 0.45;
    const double LOCAL_ESCAPE_RAY_MAX_DISTANCE = 0.85;
    const double LOCAL_ESCAPE_RAY_CLEARANCE = 0.060;
    const double LOCAL_ESCAPE_MIN_RAY_DISTANCE = 0.20;
    const double LOCAL_ESCAPE_MIN_SCAN_DISTANCE = 0.135;
    const double LOCAL_ESCAPE_SCAN_HALF_WIDTH = 0.22;
    const double LOCAL_ESCAPE_REVERSE_SCAN_DISTANCE = 0.16;
    const double LOCAL_ESCAPE_REVERSE_SPEED = 0.035;
    const double LOCAL_ESCAPE_REVERSE_PROJECTION_ALLOWANCE = 0.05;
    const double LOCAL_ESCAPE_FORWARD_SCAN_DISTANCE = 0.16;
    const double LOCAL_ESCAPE_FORWARD_RAY_DISTANCE = 0.30;
    const double LOCAL_ESCAPE_STEP_DISTANCE = 0.45;
    const double LOCAL_ESCAPE_ALIGN_YAW = 0.35;
    const double LOCAL_ESCAPE_MAX_TURN_SPEED = 0.65;
    const double BRANCH_ESCAPE_MIN_PATH_DISTANCE = 0.70;
    const double BRANCH_ESCAPE_MAX_PATH_DISTANCE = 2.40;
    const double BRANCH_ESCAPE_MAX_START_PATH_LOSS = 1.40;
    const double BRANCH_ESCAPE_MIN_CLEARANCE = 0.090;
    const double BRANCH_ESCAPE_MIN_BOTTLENECK = 0.080;

    const double ESCAPE_MIN_DISTANCE = 0.22;
    const double ESCAPE_MAX_DISTANCE = 0.55;
    const double ESCAPE_MIN_CLEARANCE = 0.080;
    const double ESCAPE_LINE_CLEARANCE = 0.070;

    const int MIN_FRONTIER_CLUSTER_SIZE = 6;

    double current_goal_x_ = 0.0;
    double current_goal_y_ = 0.0;
    double goal_start_x_ = 0.0;
    double goal_start_y_ = 0.0;
    double goal_start_projection_ = 0.0;
    double goal_start_path_distance_ = 0.0;
    int consecutive_goal_failures_ = 0;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        latest_map_ = msg;

        if (!updateRobotPose()) {
            return;
        }

        saveStartPoseOnce();
        updateForwardProgress();
        updateStartPathProgress(msg);

        if (goal_in_progress_) {
            if (isBacktrackingTowardEntrance()) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "시작점 방향 되감기 감지. 현재 목표 취소: 현재거리=%.2f, 최대거리=%.2f, 현재투영=%.2f, 최대투영=%.2f, 현재경로거리=%.2f, 최대경로거리=%.2f",
                    distanceFromStart(robot_x_, robot_y_),
                    max_start_distance_,
                    forwardProjection(robot_x_, robot_y_),
                    max_forward_projection_,
                    current_start_path_distance_,
                    max_start_path_distance_
                );

                cancelCurrentGoalAndBlacklist();
            }

            return;
        }

        trySendNextGoal();
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        latest_scan_ = msg;
        last_scan_time_ = this->now();
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
            max_start_distance_ = 0.0;
            max_start_path_distance_ = 0.0;
            current_start_path_distance_ = 0.0;
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

    static double normalizeAngle(double angle)
    {
        while (angle > 3.14159265358979323846) {
            angle -= 2.0 * 3.14159265358979323846;
        }

        while (angle < -3.14159265358979323846) {
            angle += 2.0 * 3.14159265358979323846;
        }

        return angle;
    }

    bool validScanRange(const sensor_msgs::msg::LaserScan& scan, double range) const
    {
        return std::isfinite(range) &&
               range >= std::max(0.04f, scan.range_min) &&
               range <= scan.range_max;
    }

    double scanSectorMin(double base_link_yaw, double half_width) const
    {
        if (!latest_scan_ ||
            latest_scan_->ranges.empty() ||
            latest_scan_->angle_increment <= 0.0 ||
            (this->now() - last_scan_time_).seconds() > 0.60) {
            return std::numeric_limits<double>::infinity();
        }

        // base_link -> laser static yaw is pi in the current setup.
        const double scan_center = normalizeAngle(base_link_yaw + 3.14159265358979323846);
        double best = std::numeric_limits<double>::infinity();

        for (size_t i = 0; i < latest_scan_->ranges.size(); ++i) {
            const double angle = normalizeAngle(
                latest_scan_->angle_min +
                static_cast<double>(i) * latest_scan_->angle_increment
            );

            if (std::abs(normalizeAngle(angle - scan_center)) > half_width) {
                continue;
            }

            const double range = latest_scan_->ranges[i];

            if (validScanRange(*latest_scan_, range)) {
                best = std::min(best, range);
            }
        }

        return best;
    }

    double startHeadingAlignment(double wx, double wy) const
    {
        const double dx = wx - robot_x_;
        const double dy = wy - robot_y_;
        const double distance = std::hypot(dx, dy);

        if (distance < 1e-6) {
            return 1.0;
        }

        const double forward_x = std::cos(start_yaw_);
        const double forward_y = std::sin(start_yaw_);

        return ((dx / distance) * forward_x) + ((dy / distance) * forward_y);
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

        max_start_distance_ = std::max(
            max_start_distance_,
            distanceFromStart(robot_x_, robot_y_)
        );
    }

    double distanceFromStart(double wx, double wy) const
    {
        return std::hypot(wx - start_x_, wy - start_y_);
    }

    void pruneExpiredBlacklist()
    {
        const rclcpp::Time now = this->now();

        blacklist_.erase(
            std::remove_if(
                blacklist_.begin(),
                blacklist_.end(),
                [&](const BlacklistEntry& entry) {
                    return entry.expires_at <= now;
                }),
            blacklist_.end());
    }

    void addBlacklistPoint(double wx, double wy, double ttl_seconds)
    {
        pruneExpiredBlacklist();

        blacklist_.push_back({
            wx,
            wy,
            this->now() + rclcpp::Duration::from_seconds(ttl_seconds)
        });

        if (blacklist_.size() > 16) {
            blacklist_.erase(blacklist_.begin(), blacklist_.begin() + 4);
        }
    }

    bool hasLeftEntrance() const
    {
        return max_start_distance_ >= START_RETURN_ARM_DISTANCE ||
               max_forward_projection_ >= NO_RETURN_PROJECTION_ARM_DISTANCE;
    }

    bool isInEntranceReturnZone(double wx, double wy) const
    {
        return hasLeftEntrance() &&
               distanceFromStart(wx, wy) < START_RETURN_GOAL_RADIUS;
    }

    bool isEntranceReturnForbiddenCandidate(
        double wx,
        double wy,
        double start_path_distance) const
    {
        if (!hasLeftEntrance()) {
            return false;
        }

        const double start_distance = distanceFromStart(wx, wy);
        const double projection = forwardProjection(wx, wy);

        if (start_distance < ENTRANCE_NO_RETURN_WORLD_RADIUS) {
            return true;
        }

        if (projection < START_LINE_MARGIN) {
            return true;
        }

        if (projection + NO_RETURN_PROJECTION_BACKTRACK_ALLOWANCE <
            max_forward_projection_) {
            return true;
        }

        if (start_distance + NO_RETURN_GOAL_BACKTRACK_ALLOWANCE <
            max_start_distance_) {
            return true;
        }

        if (max_start_path_distance_ >= 1.20 &&
            start_path_distance < ENTRANCE_NO_RETURN_PATH_DISTANCE) {
            return true;
        }

        return false;
    }

    bool isBacktrackingTowardEntrance() const
    {
        // dead-end 탈출을 위한 아주 짧은 후퇴만 허용한다.
        // 최대 진입 깊이에서 크게 되돌아가면 입구 복귀로 보고 즉시 막는다.
        if (!hasLeftEntrance()) {
            return false;
        }

        if (distanceFromStart(robot_x_, robot_y_) < START_RETURN_CANCEL_RADIUS) {
            return true;
        }

        if (forwardProjection(robot_x_, robot_y_) +
            NO_RETURN_PROJECTION_BACKTRACK_ALLOWANCE <
            max_forward_projection_) {
            return true;
        }

        if (distanceFromStart(robot_x_, robot_y_) +
            NO_RETURN_DISTANCE_BACKTRACK_ALLOWANCE <
            max_start_distance_) {
            return true;
        }

        if (max_start_path_distance_ >= START_PATH_CANCEL_ARM_DISTANCE &&
            current_start_path_distance_ + START_PATH_CANCEL_BACKTRACK <
            max_start_path_distance_) {
            return true;
        }

        return false;
    }

    bool violatesNoReturnPolicy(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        double wx,
        double wy)
    {
        if (!hasLeftEntrance()) {
            return false;
        }

        const double goal_start_distance = distanceFromStart(wx, wy);
        const double current_start_distance = distanceFromStart(robot_x_, robot_y_);
        const double goal_projection = forwardProjection(wx, wy);
        const double current_projection = forwardProjection(robot_x_, robot_y_);

        if (goal_projection < START_LINE_MARGIN) {
            return true;
        }

        if (goal_projection + NO_RETURN_PROJECTION_BACKTRACK_ALLOWANCE <
            current_projection) {
            return true;
        }

        if (goal_projection + NO_RETURN_PROJECTION_BACKTRACK_ALLOWANCE <
            max_forward_projection_) {
            return true;
        }

        if (goal_start_distance + NO_RETURN_GOAL_BACKTRACK_ALLOWANCE <
            current_start_distance) {
            return true;
        }

        if (goal_start_distance + NO_RETURN_GOAL_BACKTRACK_ALLOWANCE <
            max_start_distance_) {
            return true;
        }

        std::vector<int> start_distances;
        std::vector<double> start_bottleneck_clearances;

        if (!computeReachableMetricsFrom(
                map,
                start_x_,
                start_y_,
                false,
                start_distances,
                start_bottleneck_clearances)) {
            return true;
        }

        int mx = 0;
        int my = 0;

        if (!worldToMap(map, wx, wy, mx, my)) {
            return true;
        }

        const int width = static_cast<int>(map->info.width);
        const int index = my * width + mx;

        if (start_distances[index] < 0) {
            return true;
        }

        const double start_path_distance =
            start_distances[index] * map->info.resolution;

        if (isEntranceReturnForbiddenCandidate(wx, wy, start_path_distance)) {
            return true;
        }

        if (start_path_distance + START_PATH_BACKTRACK_ALLOWANCE <
            current_start_path_distance_) {
            return true;
        }

        if (start_path_distance + START_PATH_BACKTRACK_ALLOWANCE <
            max_start_path_distance_) {
            return true;
        }

        return false;
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

    double mapToWorldX(const nav_msgs::msg::OccupancyGrid::SharedPtr& map, int mx) const
    {
        return map->info.origin.position.x + (mx + 0.5) * map->info.resolution;
    }

    double mapToWorldY(const nav_msgs::msg::OccupancyGrid::SharedPtr& map, int my) const
    {
        return map->info.origin.position.y + (my + 0.5) * map->info.resolution;
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

    double openSpaceRatio(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        int mx,
        int my,
        double radius)
    {
        const int width = static_cast<int>(map->info.width);
        const int height = static_cast<int>(map->info.height);
        const int radius_cells = std::max(
            1,
            static_cast<int>(std::ceil(radius / map->info.resolution))
        );

        int open_count = 0;
        int total_count = 0;

        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                const double distance = std::hypot(
                    dx * map->info.resolution,
                    dy * map->info.resolution
                );

                if (distance > radius) {
                    continue;
                }

                const int cx = mx + dx;
                const int cy = my + dy;

                if (cx < 0 || cx >= width || cy < 0 || cy >= height) {
                    continue;
                }

                total_count++;

                if (map->data[cy * width + cx] == 0) {
                    open_count++;
                }
            }
        }

        if (total_count == 0) {
            return 0.0;
        }

        return static_cast<double>(open_count) / static_cast<double>(total_count);
    }

    bool findCenteredReachableGoalCell(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        const std::vector<int>& reachable_distances,
        const std::vector<double>& reachable_bottleneck_clearances,
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

                const double bottleneck = reachable_bottleneck_clearances[index];
                const double low_clearance_penalty =
                    std::max(0.0, WALL_CLEARANCE_COMFORT - clearance);
                const double low_bottleneck_penalty =
                    std::max(0.0, WALL_CLEARANCE_COMFORT - bottleneck);
                const double score =
                    clearance +
                    (0.70 * bottleneck) -
                    (0.25 * offset_distance) -
                    (WALL_CLEARANCE_PENALTY_WEIGHT * low_clearance_penalty) -
                    (WALL_CLEARANCE_PENALTY_WEIGHT * low_bottleneck_penalty);

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

    bool computeReachableMetricsFrom(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        double start_wx,
        double start_wy,
        bool block_entrance_return_zone,
        std::vector<int>& distances,
        std::vector<double>& bottleneck_clearances)
    {
        const int width = static_cast<int>(map->info.width);
        const int height = static_cast<int>(map->info.height);

        int start_mx = 0;
        int start_my = 0;

        if (!worldToMap(map, start_wx, start_wy, start_mx, start_my)) {
            return false;
        }

        distances.assign(width * height, -1);
        bottleneck_clearances.assign(width * height, 0.0);

        struct ReachabilityNode
        {
            int index;
            int distance;
            double bottleneck_clearance;
        };

        struct PreferWiderPath
        {
            bool operator()(
                const ReachabilityNode& lhs,
                const ReachabilityNode& rhs) const
            {
                if (std::abs(lhs.bottleneck_clearance - rhs.bottleneck_clearance) > 1e-6) {
                    return lhs.bottleneck_clearance < rhs.bottleneck_clearance;
                }

                return lhs.distance > rhs.distance;
            }
        };

        std::vector<double> clearance_cache(width * height, -1.0);
        auto cachedClearance = [&](int mx, int my) {
            const int index = my * width + mx;

            if (clearance_cache[index] < 0.0) {
                clearance_cache[index] = nearestObstacleDistance(map, mx, my, 0.30);
            }

            return clearance_cache[index];
        };

        std::priority_queue<
            ReachabilityNode,
            std::vector<ReachabilityNode>,
            PreferWiderPath
        > queue;

        const int start_index = start_my * width + start_mx;

        distances[start_index] = 0;
        bottleneck_clearances[start_index] = cachedClearance(start_mx, start_my);
        queue.push({start_index, 0, bottleneck_clearances[start_index]});

        const int dx[4] = {0, 0, -1, 1};
        const int dy[4] = {-1, 1, 0, 0};

        while (!queue.empty()) {
            const ReachabilityNode node = queue.top();
            queue.pop();

            const int current = node.index;
            const int current_mx = current % width;
            const int current_my = current / width;

            if (node.bottleneck_clearance < bottleneck_clearances[current] - 1e-6) {
                continue;
            }

            if (std::abs(node.bottleneck_clearance - bottleneck_clearances[current]) <= 1e-6 &&
                node.distance > distances[current]) {
                continue;
            }

            for (int i = 0; i < 4; ++i) {
                const int nx = current_mx + dx[i];
                const int ny = current_my + dy[i];

                if (nx <= 0 || nx >= width - 1 || ny <= 0 || ny >= height - 1) {
                    continue;
                }

                const double neighbor_wx =
                    map->info.origin.position.x + (nx + 0.5) * map->info.resolution;

                const double neighbor_wy =
                    map->info.origin.position.y + (ny + 0.5) * map->info.resolution;

                if (block_entrance_return_zone &&
                    (isInEntranceReturnZone(neighbor_wx, neighbor_wy) ||
                     forwardProjection(neighbor_wx, neighbor_wy) < START_LINE_MARGIN ||
                     forwardProjection(neighbor_wx, neighbor_wy) +
                     NO_RETURN_PROJECTION_BACKTRACK_ALLOWANCE <
                     max_forward_projection_)) {
                    continue;
                }

                const int neighbor_index = ny * width + nx;

                if (!isTraversableCell(map, nx, ny)) {
                    continue;
                }

                const int candidate_distance = distances[current] + 1;
                const double candidate_bottleneck = std::min(
                    bottleneck_clearances[current],
                    cachedClearance(nx, ny)
                );

                const bool wider_path =
                    candidate_bottleneck > bottleneck_clearances[neighbor_index] + 1e-6;

                const bool equally_wide_shorter_path =
                    std::abs(candidate_bottleneck - bottleneck_clearances[neighbor_index]) <= 1e-6 &&
                    (distances[neighbor_index] < 0 ||
                     candidate_distance < distances[neighbor_index]);

                if (!wider_path && !equally_wide_shorter_path) {
                    continue;
                }

                distances[neighbor_index] = candidate_distance;
                bottleneck_clearances[neighbor_index] = candidate_bottleneck;
                queue.push({neighbor_index, candidate_distance, candidate_bottleneck});
            }
        }

        return true;
    }

    bool computeReachableMetrics(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        std::vector<int>& distances,
        std::vector<double>& bottleneck_clearances)
    {
        return computeReachableMetricsFrom(
            map,
            robot_x_,
            robot_y_,
            true,
            distances,
            bottleneck_clearances
        );
    }

    bool updateStartPathProgress(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map)
    {
        if (!is_start_position_saved_) {
            return false;
        }

        const int width = static_cast<int>(map->info.width);
        const double resolution = map->info.resolution;

        std::vector<int> start_distances;
        std::vector<double> start_bottleneck_clearances;

        if (!computeReachableMetricsFrom(
                map,
                start_x_,
                start_y_,
                false,
                start_distances,
                start_bottleneck_clearances)) {
            return false;
        }

        int robot_mx = 0;
        int robot_my = 0;

        if (!worldToMap(map, robot_x_, robot_y_, robot_mx, robot_my)) {
            return false;
        }

        const int robot_index = robot_my * width + robot_mx;

        if (start_distances[robot_index] < 0) {
            return false;
        }

        current_start_path_distance_ = start_distances[robot_index] * resolution;
        max_start_path_distance_ = std::max(
            max_start_path_distance_,
            current_start_path_distance_
        );

        return true;
    }

    bool trySendNextGoal()
    {
        if (!latest_map_ || goal_in_progress_ || canceling_goal_ || escape_active_) {
            return false;
        }

        if (!updateRobotPose()) {
            return false;
        }

        updateForwardProgress();
        updateStartPathProgress(latest_map_);
        pruneExpiredBlacklist();

        geometry_msgs::msg::PoseStamped next_goal;
        bool allow_branch_escape_goal = false;

        // v12 핵심:
        // - 출발 직후에는 frontier가 안정적으로 생기기 전이라 local lookahead로 먼저 전진시킨다.
        // - 어느 정도 입구에서 벗어나면 progress-frontier를 우선해서 막다른 포켓을 피한다.
        // - 실패가 반복되면 relaxed frontier와 escape goal을 허용한다.
        bool found_goal = false;

        const bool early_start =
            max_start_distance_ < 0.60 &&
            current_start_path_distance_ < 0.60 &&
            consecutive_goal_failures_ == 0;

        if (early_start) {
            found_goal = makeLookaheadTurnGoal(latest_map_, next_goal) ||
                         makeOpenRayGoal(latest_map_, next_goal) ||
                         makeKeepMovingGoal(latest_map_, next_goal);
        }

        if (!found_goal) {
            found_goal = findBestFrontier(
                latest_map_,
                next_goal,
                consecutive_goal_failures_ > 0);
        }

        if (!found_goal && consecutive_goal_failures_ == 0 && !hasLeftEntrance()) {
            found_goal = makeLookaheadTurnGoal(latest_map_, next_goal) ||
                         makeOpenRayGoal(latest_map_, next_goal) ||
                         makeKeepMovingGoal(latest_map_, next_goal);
        }

        if (!found_goal) {
            if (hasLeftEntrance()) {
                found_goal = findBestFrontier(latest_map_, next_goal, false);

                if (!found_goal) {
                    found_goal = makeBranchEscapeGoal(latest_map_, next_goal);
                    allow_branch_escape_goal = found_goal;
                }

                if (!found_goal) {
                    found_goal = makeKeepMovingGoal(latest_map_, next_goal);
                }
            } else {
                found_goal = findBestFrontier(latest_map_, next_goal, true) ||
                             makeEscapeGoal(latest_map_, next_goal) ||
                             makeKeepMovingGoal(latest_map_, next_goal);
            }
        }

        if (!found_goal) {
            RCLCPP_WARN(
                this->get_logger(),
                "v12: 보낼 수 있는 frontier/local goal이 없습니다. 멈추지 않고 전진 크리프를 시도합니다."
            );
            if (canUseBackupRecovery()) {
                startEscapeRecovery("no reachable v12 goal");
            } else {
                startForwardCreep("no reachable v12 goal");
            }
            return true;
        }

        if (!allow_branch_escape_goal &&
            violatesNoReturnPolicy(
                latest_map_,
                next_goal.pose.position.x,
                next_goal.pose.position.y)) {
            RCLCPP_WARN(
                this->get_logger(),
                "입구 방향 goal 차단: goal=(%.2f, %.2f), goal시작거리=%.2f, 현재거리=%.2f, 최대거리=%.2f, goal투영=%.2f, 현재투영=%.2f, 최대투영=%.2f, 현재경로거리=%.2f, 최대경로거리=%.2f",
                next_goal.pose.position.x,
                next_goal.pose.position.y,
                distanceFromStart(
                    next_goal.pose.position.x,
                    next_goal.pose.position.y
                ),
                distanceFromStart(robot_x_, robot_y_),
                max_start_distance_,
                forwardProjection(
                    next_goal.pose.position.x,
                    next_goal.pose.position.y
                ),
                forwardProjection(robot_x_, robot_y_),
                max_forward_projection_,
                current_start_path_distance_,
                max_start_path_distance_
            );
            addBlacklistPoint(
                next_goal.pose.position.x,
                next_goal.pose.position.y,
                BLACKLIST_TTL_CANCEL
            );
            startForwardCreep("blocked entrance-direction goal");
            return true;
        }

        sendGoal(next_goal);
        return true;
    }

    void startEscapeRecovery(const std::string& reason)
    {
        if (escape_active_ || goal_in_progress_ || canceling_goal_) {
            return;
        }

        geometry_msgs::msg::Twist stop;
        escape_cmd_pub_->publish(stop);

        if (hasLeftEntrance()) {
            RCLCPP_WARN(
                this->get_logger(),
                "입구 복귀 방지를 위해 후진 복구를 생략합니다. reason=%s",
                reason.c_str()
            );
            return;
        }

        escape_active_ = true;
        escape_until_ = this->now() + rclcpp::Duration::from_seconds(ESCAPE_BACKUP_DURATION);
        recovery_linear_speed_ = -ESCAPE_BACKUP_SPEED;
        recovery_angular_speed_ = 0.0;

        RCLCPP_WARN(
            this->get_logger(),
            "Nav2 goal 생성 실패/벽 접촉 복구: %.2fs 동안 %.3fm/s 후진합니다. reason=%s",
            ESCAPE_BACKUP_DURATION,
            ESCAPE_BACKUP_SPEED,
            reason.c_str()
        );
    }

    void startForwardCreep(const std::string& reason)
    {
        if (escape_active_ || goal_in_progress_ || canceling_goal_) {
            return;
        }

        if (!updateRobotPose()) {
            return;
        }

        const double current_projection = forwardProjection(robot_x_, robot_y_);
        const double pi = 3.14159265358979323846;
        const std::vector<double> angle_offsets = {
            0.0,
            pi / 12.0,
            -pi / 12.0,
            pi / 6.0,
            -pi / 6.0,
            pi / 4.0,
            -pi / 4.0,
            pi / 3.0,
            -pi / 3.0,
            pi / 2.0,
            -pi / 2.0,
            pi * 2.0 / 3.0,
            -pi * 2.0 / 3.0
        };

        bool found_direction = false;
        const bool scan_ready =
            latest_scan_ &&
            !latest_scan_->ranges.empty() &&
            latest_scan_->angle_increment > 0.0 &&
            (this->now() - last_scan_time_).seconds() <= 0.60;
        double best_score = -std::numeric_limits<double>::max();
        double best_yaw = robot_yaw_;
        double best_ray = 0.0;
        double best_scan_clearance = 0.0;
        double best_projected_next = current_projection;
        double best_scan_only_score = -std::numeric_limits<double>::max();
        double best_scan_only_yaw = robot_yaw_;
        double best_scan_only_clearance = 0.0;

        if (scan_ready) {
            const double front_scan_clearance = scanSectorMin(
                0.0,
                LOCAL_ESCAPE_SCAN_HALF_WIDTH
            );
            const double front_ray_distance = rayClearDistance(
                latest_map_,
                robot_yaw_,
                LOCAL_ESCAPE_RAY_MAX_DISTANCE,
                LOCAL_ESCAPE_RAY_CLEARANCE
            );
            const double forward_projection = forwardProjection(
                robot_x_ + FORWARD_CREEP_PROJECTION_CHECK_DISTANCE * std::cos(robot_yaw_),
                robot_y_ + FORWARD_CREEP_PROJECTION_CHECK_DISTANCE * std::sin(robot_yaw_)
            );
            const bool forward_keeps_progress =
                !hasLeftEntrance() ||
                forward_projection + NO_RETURN_PROJECTION_BACKTRACK_ALLOWANCE >=
                current_projection;

            if (front_scan_clearance >= LOCAL_ESCAPE_FORWARD_SCAN_DISTANCE &&
                front_ray_distance >= LOCAL_ESCAPE_FORWARD_RAY_DISTANCE &&
                forward_keeps_progress) {
                recovery_linear_speed_ = FORWARD_CREEP_SPEED;
                recovery_angular_speed_ = 0.0;
                escape_active_ = true;
                escape_until_ = this->now() + rclcpp::Duration::from_seconds(FORWARD_CREEP_DURATION);

                RCLCPP_WARN(
                    this->get_logger(),
                    "Nav2 goal 없이 전방 유지 이동: linear=%.3f, scan=%.2f, map_ray=%.2f, proj=%.2f -> %.2f, reason=%s",
                    recovery_linear_speed_,
                    front_scan_clearance,
                    front_ray_distance,
                    current_projection,
                    forward_projection,
                    reason.c_str()
                );
                return;
            }
        }

        for (const double offset : angle_offsets) {
            const double yaw = robot_yaw_ + offset;

            if (!scan_ready) {
                continue;
            }

            const double scan_clearance = scanSectorMin(
                offset,
                LOCAL_ESCAPE_SCAN_HALF_WIDTH
            );

            const double scan_step_projection = forwardProjection(
                robot_x_ + FORWARD_CREEP_PROJECTION_CHECK_DISTANCE * std::cos(yaw),
                robot_y_ + FORWARD_CREEP_PROJECTION_CHECK_DISTANCE * std::sin(yaw)
            );
            double scan_only_score = 0.0;
            scan_only_score += scan_clearance;
            scan_only_score += 0.35 * (scan_step_projection - current_projection);
            scan_only_score -= 0.08 * std::abs(offset);

            if (scan_only_score > best_scan_only_score) {
                best_scan_only_score = scan_only_score;
                best_scan_only_yaw = yaw;
                best_scan_only_clearance = scan_clearance;
            }

            if (scan_clearance < LOCAL_ESCAPE_MIN_SCAN_DISTANCE) {
                continue;
            }

            const double ray_distance = rayClearDistance(
                latest_map_,
                yaw,
                LOCAL_ESCAPE_RAY_MAX_DISTANCE,
                LOCAL_ESCAPE_RAY_CLEARANCE
            );

            if (ray_distance < LOCAL_ESCAPE_MIN_RAY_DISTANCE) {
                continue;
            }

            const double step_distance = std::min(
                LOCAL_ESCAPE_STEP_DISTANCE,
                std::max(0.0, ray_distance - 0.05)
            );

            const double wx = robot_x_ + step_distance * std::cos(yaw);
            const double wy = robot_y_ + step_distance * std::sin(yaw);
            const double projected_next = forwardProjection(wx, wy);

            if (hasLeftEntrance() &&
                projected_next + NO_RETURN_PROJECTION_BACKTRACK_ALLOWANCE <
                current_projection) {
                continue;
            }

            if (hasLeftEntrance() &&
                projected_next + NO_RETURN_PROJECTION_BACKTRACK_ALLOWANCE <
                max_forward_projection_ - 0.25) {
                continue;
            }

            double score = 0.0;
            score += 2.00 * ray_distance;
            score += 1.20 * std::min(scan_clearance, LOCAL_ESCAPE_RAY_MAX_DISTANCE);
            score += 1.60 * (projected_next - current_projection);
            score -= 0.35 * std::abs(offset);

            if (!found_direction || score > best_score) {
                found_direction = true;
                best_score = score;
                best_yaw = yaw;
                best_ray = ray_distance;
                best_scan_clearance = scan_clearance;
                best_projected_next = projected_next;
            }
        }

        if (!found_direction) {
            best_yaw = scan_ready ? best_scan_only_yaw : start_yaw_;
            best_ray = 0.0;
            best_scan_clearance = best_scan_only_clearance;
            best_projected_next = current_projection;
        }

        const double yaw_error = std::atan2(
            std::sin(best_yaw - robot_yaw_),
            std::cos(best_yaw - robot_yaw_)
        );

        recovery_linear_speed_ = FORWARD_CREEP_SPEED;
        recovery_angular_speed_ = std::clamp(
            1.20 * yaw_error,
            -LOCAL_ESCAPE_MAX_TURN_SPEED,
            LOCAL_ESCAPE_MAX_TURN_SPEED
        );

        if (!found_direction) {
            recovery_linear_speed_ = 0.0;
        } else if (std::abs(yaw_error) > LOCAL_ESCAPE_ALIGN_YAW) {
            recovery_linear_speed_ = FORWARD_CREEP_SPEED * 0.45;
        }

        if (!found_direction && std::abs(recovery_angular_speed_) < 0.05) {
            recovery_angular_speed_ = FORWARD_CREEP_TURN_SPEED;
        }

        if (!found_direction && scan_ready) {
            const double rear_clearance = scanSectorMin(
                3.14159265358979323846,
                LOCAL_ESCAPE_SCAN_HALF_WIDTH
            );
            const double reverse_projection = forwardProjection(
                robot_x_ - FORWARD_CREEP_PROJECTION_CHECK_DISTANCE * std::cos(robot_yaw_),
                robot_y_ - FORWARD_CREEP_PROJECTION_CHECK_DISTANCE * std::sin(robot_yaw_)
            );
            const bool reverse_keeps_progress =
                !hasLeftEntrance() ||
                reverse_projection + LOCAL_ESCAPE_REVERSE_PROJECTION_ALLOWANCE >=
                current_projection;

            if (rear_clearance > LOCAL_ESCAPE_REVERSE_SCAN_DISTANCE &&
                reverse_keeps_progress) {
                recovery_linear_speed_ = -LOCAL_ESCAPE_REVERSE_SPEED;
                recovery_angular_speed_ = 0.0;
                best_scan_clearance = rear_clearance;
                best_projected_next = reverse_projection;
            }
        }

        escape_active_ = true;
        escape_until_ = this->now() + rclcpp::Duration::from_seconds(FORWARD_CREEP_DURATION);

        RCLCPP_WARN(
            this->get_logger(),
            "Nav2 goal 없이 scan local escape: linear=%.3f, angular=%.3f, map_ray=%.2f, scan=%.2f, yaw_error=%.2f, proj=%.2f -> %.2f, found=%d, reason=%s",
            recovery_linear_speed_,
            recovery_angular_speed_,
            best_ray,
            best_scan_clearance,
            yaw_error,
            current_projection,
            best_projected_next,
            found_direction ? 1 : 0,
            reason.c_str()
        );
    }

    bool canUseBackupRecovery() const
    {
        if (hasLeftEntrance()) {
            return false;
        }

        return consecutive_goal_failures_ >= ESCAPE_BACKUP_MIN_FAILURES &&
               max_start_distance_ >= ESCAPE_BACKUP_MIN_START_DISTANCE &&
               distanceFromStart(robot_x_, robot_y_) >= 0.45;
    }

    void escapeTimerCallback()
    {
        if (!escape_active_) {
            return;
        }

        geometry_msgs::msg::Twist cmd;

        if (this->now() < escape_until_) {
            cmd.linear.x = recovery_linear_speed_;
            cmd.angular.z = recovery_angular_speed_;
            escape_cmd_pub_->publish(cmd);
            return;
        }

        escape_cmd_pub_->publish(cmd);
        escape_active_ = false;
        recovery_linear_speed_ = 0.0;
        recovery_angular_speed_ = 0.0;
        trySendNextGoal();
    }

    void cancelCurrentGoalAndBlacklist()
    {
        addBlacklistPoint(current_goal_x_, current_goal_y_, BLACKLIST_TTL_CANCEL);
        canceling_goal_ = true;

        geometry_msgs::msg::Twist stop;
        for (int i = 0; i < 3; ++i) {
            escape_cmd_pub_->publish(stop);
        }

        if (active_goal_handle_) {
            nav_client_->async_cancel_goal(active_goal_handle_);
        } else {
            goal_in_progress_ = false;
            canceling_goal_ = false;
        }
    }

    double rayClearDistanceFrom(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        double start_x,
        double start_y,
        double yaw,
        double max_distance,
        double clearance)
    {
        const double step = std::max(0.03, static_cast<double>(map->info.resolution));
        double last_safe_distance = 0.0;

        for (double distance = step; distance <= max_distance; distance += step) {
            const double wx = start_x + distance * std::cos(yaw);
            const double wy = start_y + distance * std::sin(yaw);

            int mx = 0;
            int my = 0;

            if (!worldToMap(map, wx, wy, mx, my)) {
                break;
            }

            const int index = my * static_cast<int>(map->info.width) + mx;

            if (map->data[index] != 0 ||
                hasObstacleWithin(map, mx, my, clearance)) {
                break;
            }

            last_safe_distance = distance;
        }

        return last_safe_distance;
    }

    double rayClearDistance(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        double yaw,
        double max_distance,
        double clearance)
    {
        return rayClearDistanceFrom(
            map,
            robot_x_,
            robot_y_,
            yaw,
            max_distance,
            clearance
        );
    }

    bool isLineSafeFrom(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        double start_x,
        double start_y,
        double target_x,
        double target_y,
        double clearance)
    {
        const double distance = std::hypot(target_x - start_x, target_y - start_y);
        const int steps = std::max(2, static_cast<int>(std::ceil(distance / map->info.resolution)));

        for (int i = 1; i <= steps; ++i) {
            const double ratio = static_cast<double>(i) / static_cast<double>(steps);
            const double wx = start_x + (target_x - start_x) * ratio;
            const double wy = start_y + (target_y - start_y) * ratio;

            int mx = 0;
            int my = 0;

            if (!worldToMap(map, wx, wy, mx, my)) {
                return false;
            }

            const int index = my * static_cast<int>(map->info.width) + mx;

            if (map->data[index] != 0 ||
                hasObstacleWithin(map, mx, my, clearance)) {
                return false;
            }
        }

        return true;
    }

    bool makeLookaheadTurnGoal(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        geometry_msgs::msg::PoseStamped& goal)
    {
        const double pi = 3.14159265358979323846;
        const double current_projection = forwardProjection(robot_x_, robot_y_);

        bool found = false;
        double best_score = std::numeric_limits<double>::max();
        double best_x = robot_x_;
        double best_y = robot_y_;
        double best_branch = 0.0;
        double best_clearance = 0.0;
        double best_turn = 0.0;

        const std::vector<double> first_offsets = {
            0.0,
            pi / 18.0,
            -pi / 18.0,
            pi / 12.0,
            -pi / 12.0
        };

        const std::vector<double> pivot_distances = {
            0.35,
            0.45,
            0.55
        };

        const std::vector<double> turn_offsets = {
            pi / 3.0,
            -pi / 3.0,
            pi / 4.0,
            -pi / 4.0,
            pi / 6.0,
            -pi / 6.0,
            pi / 2.0,
            -pi / 2.0,
            0.0
        };

        for (const double first_offset : first_offsets) {
            const double first_yaw = robot_yaw_ + first_offset;

            for (const double pivot_distance : pivot_distances) {
                const double pivot_x = robot_x_ + pivot_distance * std::cos(first_yaw);
                const double pivot_y = robot_y_ + pivot_distance * std::sin(first_yaw);

                if (!isLineSafeFrom(
                        map,
                        robot_x_,
                        robot_y_,
                        pivot_x,
                        pivot_y,
                        LOOKAHEAD_PIVOT_CLEARANCE)) {
                    continue;
                }

                for (const double turn_offset : turn_offsets) {
                    const double second_yaw = first_yaw + turn_offset;
                    const double branch_distance = rayClearDistanceFrom(
                        map,
                        pivot_x,
                        pivot_y,
                        second_yaw,
                        0.95,
                        OPEN_RAY_LINE_CLEARANCE
                    );

                    if (branch_distance < 0.42) {
                        continue;
                    }

                    const double goal_distance = std::min(0.78, branch_distance - 0.05);
                    const double wx = pivot_x + goal_distance * std::cos(second_yaw);
                    const double wy = pivot_y + goal_distance * std::sin(second_yaw);

                    if (!isLineSafeFrom(
                            map,
                            pivot_x,
                            pivot_y,
                            wx,
                            wy,
                            LOOKAHEAD_PIVOT_CLEARANCE)) {
                        continue;
                    }

                    if (isInEntranceReturnZone(wx, wy)) {
                        continue;
                    }

                    const double projection = forwardProjection(wx, wy);

                    if (projection + KEEP_MOVING_BACKTRACK_ALLOWANCE < current_projection ||
                        projection < START_LINE_MARGIN) {
                        continue;
                    }

                    int mx = 0;
                    int my = 0;

                    if (!worldToMap(map, wx, wy, mx, my)) {
                        continue;
                    }

                    const int index = my * static_cast<int>(map->info.width) + mx;

                    if (map->data[index] != 0) {
                        continue;
                    }

                    const double clearance = nearestObstacleDistance(map, mx, my, 0.30);

                    if (clearance < LOOKAHEAD_GOAL_CLEARANCE) {
                        continue;
                    }

                    const double open_ratio = openSpaceRatio(map, mx, my, OPEN_SPACE_RADIUS);
                    const double turn_amount = std::abs(turn_offset);

                    double score = 0.0;
                    score -= 2.20 * branch_distance;
                    score -= 1.30 * clearance;
                    score -= 0.90 * open_ratio;
                    score -= 0.75 * projection;
                    score += 0.45 * std::abs(first_offset);
                    score += 0.20 * turn_amount;

                    // A useful bend should beat a straight ray into a pocket.
                    if (turn_amount > pi / 5.0 && branch_distance > 0.55) {
                        score -= 0.75;
                    }

                    if (!found || score < best_score) {
                        found = true;
                        best_score = score;
                        best_x = wx;
                        best_y = wy;
                        best_branch = branch_distance;
                        best_clearance = clearance;
                        best_turn = turn_offset;
                    }
                }
            }
        }

        if (!found) {
            return false;
        }

        goal.header.frame_id = "map";
        goal.header.stamp = this->now();
        goal.pose.position.x = best_x;
        goal.pose.position.y = best_y;
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = std::sin(robot_yaw_ * 0.5);
        goal.pose.orientation.w = std::cos(robot_yaw_ * 0.5);

        current_goal_x_ = best_x;
        current_goal_y_ = best_y;

        RCLCPP_INFO(
            this->get_logger(),
            "2단계 열린 방향 goal: x=%.2f, y=%.2f, score=%.2f, branch=%.2f, turn=%.2f, clearance=%.2f",
            current_goal_x_,
            current_goal_y_,
            best_score,
            best_branch,
            best_turn,
            best_clearance
        );

        return true;
    }

    bool makeOpenRayGoal(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        geometry_msgs::msg::PoseStamped& goal)
    {
        const double pi = 3.14159265358979323846;
        const double current_projection = forwardProjection(robot_x_, robot_y_);

        bool found = false;
        double best_score = std::numeric_limits<double>::max();
        double best_x = robot_x_;
        double best_y = robot_y_;
        double best_ray = 0.0;
        double best_clearance = 0.0;

        const std::vector<double> angle_offsets = {
            0.0,
            pi / 18.0,
            -pi / 18.0,
            pi / 9.0,
            -pi / 9.0,
            pi / 6.0,
            -pi / 6.0,
            pi / 4.0,
            -pi / 4.0,
            pi / 3.0,
            -pi / 3.0
        };

        const std::vector<double> preferred_distances = {
            1.05,
            0.90,
            0.75,
            0.60,
            0.48
        };

        for (const double angle_offset : angle_offsets) {
            const double yaw = robot_yaw_ + angle_offset;
            const double ray_distance = rayClearDistance(
                map,
                yaw,
                OPEN_RAY_MAX_DISTANCE,
                OPEN_RAY_LINE_CLEARANCE
            );

            if (ray_distance < 0.42) {
                continue;
            }

            for (const double desired_distance : preferred_distances) {
                const double distance = std::min(desired_distance, ray_distance - 0.05);

                if (distance < 0.42) {
                    continue;
                }

                const double wx = robot_x_ + distance * std::cos(yaw);
                const double wy = robot_y_ + distance * std::sin(yaw);

                if (isInEntranceReturnZone(wx, wy)) {
                    continue;
                }

                const double projection = forwardProjection(wx, wy);

                if (projection + KEEP_MOVING_BACKTRACK_ALLOWANCE < current_projection ||
                    projection < START_LINE_MARGIN) {
                    continue;
                }

                int mx = 0;
                int my = 0;

                if (!worldToMap(map, wx, wy, mx, my)) {
                    continue;
                }

                const int index = my * static_cast<int>(map->info.width) + mx;

                if (map->data[index] != 0) {
                    continue;
                }

                const double clearance = nearestObstacleDistance(map, mx, my, 0.30);

                if (clearance < OPEN_RAY_GOAL_CLEARANCE) {
                    continue;
                }

                const double open_ratio = openSpaceRatio(map, mx, my, OPEN_SPACE_RADIUS);
                const double angle_cost = std::abs(angle_offset);
                const double step_error = std::abs(distance - 0.85);

                double score = 0.0;
                score += 1.20 * angle_cost;
                score += 0.50 * step_error;
                score -= 2.40 * ray_distance;
                score -= 1.20 * clearance;
                score -= 0.80 * open_ratio;
                score -= 0.70 * projection;

                if (!found || score < best_score) {
                    found = true;
                    best_score = score;
                    best_x = wx;
                    best_y = wy;
                    best_ray = ray_distance;
                    best_clearance = clearance;
                }
            }
        }

        if (!found) {
            return false;
        }

        goal.header.frame_id = "map";
        goal.header.stamp = this->now();
        goal.pose.position.x = best_x;
        goal.pose.position.y = best_y;
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = std::sin(robot_yaw_ * 0.5);
        goal.pose.orientation.w = std::cos(robot_yaw_ * 0.5);

        current_goal_x_ = best_x;
        current_goal_y_ = best_y;

        RCLCPP_INFO(
            this->get_logger(),
            "열린 방향 우선 goal: x=%.2f, y=%.2f, score=%.2f, ray=%.2f, clearance=%.2f",
            current_goal_x_,
            current_goal_y_,
            best_score,
            best_ray,
            best_clearance
        );

        return true;
    }

    bool makeKeepMovingGoal(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        geometry_msgs::msg::PoseStamped& goal)
    {
        std::vector<int> reachable_distances;
        std::vector<double> reachable_bottleneck_clearances;
        std::vector<int> start_distances;
        std::vector<double> start_bottleneck_clearances;

        if (!computeReachableMetrics(
                map,
                reachable_distances,
                reachable_bottleneck_clearances)) {
            return false;
        }

        if (!computeReachableMetricsFrom(
                map,
                start_x_,
                start_y_,
                false,
                start_distances,
                start_bottleneck_clearances)) {
            return false;
        }

        const int width = static_cast<int>(map->info.width);
        const int height = static_cast<int>(map->info.height);
        const double resolution = map->info.resolution;
        const double current_projection = forwardProjection(robot_x_, robot_y_);
        const bool short_fallback = consecutive_goal_failures_ >= 2;
        const double target_step = short_fallback ? 0.45 : 0.85;
        const double min_step = short_fallback ? 0.20 : KEEP_MOVING_MIN_DISTANCE;
        const double max_step = short_fallback ? 0.75 : KEEP_MOVING_MAX_DISTANCE;
        const double min_euclidean_step = short_fallback ?
            KEEP_MOVING_SHORT_MIN_EUCLIDEAN_DISTANCE :
            KEEP_MOVING_MIN_EUCLIDEAN_DISTANCE;

        auto selectGoal = [&](bool relaxed, bool ignore_blacklist, int& best_mx, int& best_my,
                              double& best_score, double& best_path_distance,
                              double& best_start_path_distance, double& best_projection,
                              double& best_bottleneck) -> bool {
            bool found = false;
            best_score = std::numeric_limits<double>::max();

            const double min_clearance =
                relaxed ? KEEP_MOVING_RELAXED_CLEARANCE : KEEP_MOVING_MIN_CLEARANCE;
            const double min_bottleneck =
                relaxed ? KEEP_MOVING_RELAXED_BOTTLENECK : KEEP_MOVING_MIN_BOTTLENECK;

            for (int my = 1; my < height - 1; ++my) {
                for (int mx = 1; mx < width - 1; ++mx) {
                    const int index = my * width + mx;

                    if (map->data[index] != 0 ||
                        reachable_distances[index] < 0 ||
                        start_distances[index] < 0) {
                        continue;
                    }

                    const double path_distance = reachable_distances[index] * resolution;

                    if (path_distance < min_step || path_distance > max_step) {
                        continue;
                    }

                    const double wx = mapToWorldX(map, mx);
                    const double wy = mapToWorldY(map, my);
                    const double euclidean_distance =
                        std::hypot(wx - robot_x_, wy - robot_y_);

                    if (euclidean_distance < min_euclidean_step) {
                        continue;
                    }

                    if (isInEntranceReturnZone(wx, wy)) {
                        continue;
                    }

                    const double projection = forwardProjection(wx, wy);

                    if (projection < START_LINE_MARGIN) {
                        continue;
                    }

                    const double backtrack_allowance =
                        relaxed ? 0.35 : KEEP_MOVING_BACKTRACK_ALLOWANCE;

                    if (projection + backtrack_allowance < current_projection) {
                        continue;
                    }

                    const double start_path_distance = start_distances[index] * resolution;

                    if (isEntranceReturnForbiddenCandidate(
                            wx,
                            wy,
                            start_path_distance)) {
                        continue;
                    }

                    const double path_backtrack_allowance =
                        relaxed ? 0.35 : START_PATH_BACKTRACK_ALLOWANCE;

                    if (start_path_distance + path_backtrack_allowance <
                        max_start_path_distance_) {
                        continue;
                    }

                    const double bottleneck = reachable_bottleneck_clearances[index];

                    if (bottleneck < min_bottleneck) {
                        continue;
                    }

                    const double clearance = nearestObstacleDistance(map, mx, my, 0.30);

                    if (clearance < min_clearance) {
                        continue;
                    }

                    if (!ignore_blacklist) {
                        bool is_blacklisted = false;

                        for (const auto& bp : blacklist_) {
                            if (std::hypot(wx - bp.x, wy - bp.y) <
                                KEEP_MOVING_BLACKLIST_RADIUS) {
                                is_blacklisted = true;
                                break;
                            }
                        }

                        if (is_blacklisted) {
                            continue;
                        }
                    }

                    const double yaw = std::atan2(wy - robot_y_, wx - robot_x_);
                    const double angle_diff = std::abs(std::atan2(
                        std::sin(yaw - robot_yaw_),
                        std::cos(yaw - robot_yaw_)
                    ));
                    const double forward_alignment = std::cos(angle_diff);
                    const double open_ratio = openSpaceRatio(map, mx, my, OPEN_SPACE_RADIUS);

                    const double step_error = std::abs(path_distance - target_step);

                    double score = 0.0;
                    score += 0.55 * step_error;
                    score += 0.75 * angle_diff;
                    score -= 2.10 * start_path_distance;
                    score -= 1.50 * projection;
                    score -= 1.25 * clearance;
                    score -= 1.00 * bottleneck;
                    score -= FORWARD_ALIGNMENT_WEIGHT * forward_alignment;
                    score -= OPEN_SPACE_WEIGHT * open_ratio;
                    score += WALL_CLEARANCE_PENALTY_WEIGHT *
                             std::max(0.0, WALL_CLEARANCE_COMFORT - clearance);
                    score += WALL_CLEARANCE_PENALTY_WEIGHT *
                             std::max(0.0, WALL_CLEARANCE_COMFORT - bottleneck);
                    score += 4.0 * std::max(0.0, OPEN_SPACE_MIN_RATIO - open_ratio);

                    // Nav2 global planner가 통로를 막힌 곳으로 판단하지 않게
                    // 좁은 병목 후보는 keep-moving goal에서 강하게 밀어낸다.
                    score += 12.0 * std::max(0.0, KEEP_MOVING_MIN_BOTTLENECK - bottleneck);

                    if (projection < current_projection) {
                        score += 2.00;
                    }

                    if (!found || score < best_score) {
                        found = true;
                        best_score = score;
                        best_mx = mx;
                        best_my = my;
                        best_path_distance = path_distance;
                        best_start_path_distance = start_path_distance;
                        best_projection = projection;
                        best_bottleneck = bottleneck;
                    }
                }
            }

            return found;
        };

        int best_mx = 0;
        int best_my = 0;
        double best_score = 0.0;
        double best_path_distance = 0.0;
        double best_start_path_distance = 0.0;
        double best_projection = 0.0;
        double best_bottleneck = 0.0;

        bool found = selectGoal(
            false,
            false,
            best_mx,
            best_my,
            best_score,
            best_path_distance,
            best_start_path_distance,
            best_projection,
            best_bottleneck
        );

        if (!found) {
            found = selectGoal(
                true,
                false,
                best_mx,
                best_my,
                best_score,
                best_path_distance,
                best_start_path_distance,
                best_projection,
                best_bottleneck
            );
        }

        if (!found) {
            found = selectGoal(
                true,
                true,
                best_mx,
                best_my,
                best_score,
                best_path_distance,
                best_start_path_distance,
                best_projection,
                best_bottleneck
            );
        }

        if (!found) {
            return false;
        }

        const double goal_x = mapToWorldX(map, best_mx);
        const double goal_y = mapToWorldY(map, best_my);
        goal.header.frame_id = "map";
        goal.header.stamp = this->now();
        goal.pose.position.x = goal_x;
        goal.pose.position.y = goal_y;
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = std::sin(robot_yaw_ * 0.5);
        goal.pose.orientation.w = std::cos(robot_yaw_ * 0.5);

        current_goal_x_ = goal_x;
        current_goal_y_ = goal_y;

        RCLCPP_WARN(
            this->get_logger(),
            "비상 지속 이동 목표: x=%.2f, y=%.2f, score=%.2f, path=%.2f, start_path=%.2f, projection=%.2f, bottleneck=%.2f",
            current_goal_x_,
            current_goal_y_,
            best_score,
            best_path_distance,
            best_start_path_distance,
            best_projection,
            best_bottleneck
        );

        return true;
    }

    bool isStraightLineSafe(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        double target_x,
        double target_y,
        double clearance)
    {
        const double distance = std::hypot(target_x - robot_x_, target_y - robot_y_);
        const int steps = std::max(2, static_cast<int>(std::ceil(distance / map->info.resolution)));

        for (int i = 1; i <= steps; ++i) {
            const double ratio = static_cast<double>(i) / static_cast<double>(steps);
            const double wx = robot_x_ + (target_x - robot_x_) * ratio;
            const double wy = robot_y_ + (target_y - robot_y_) * ratio;

            int mx = 0;
            int my = 0;

            if (!worldToMap(map, wx, wy, mx, my)) {
                return false;
            }

            const int index = my * static_cast<int>(map->info.width) + mx;

            if (map->data[index] != 0) {
                return false;
            }

            if (hasObstacleWithin(map, mx, my, clearance)) {
                return false;
            }
        }

        return true;
    }

    bool makeBranchEscapeGoal(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        geometry_msgs::msg::PoseStamped& goal)
    {
        std::vector<int> reachable_distances;
        std::vector<double> reachable_bottleneck_clearances;
        std::vector<int> start_distances;
        std::vector<double> start_bottleneck_clearances;

        if (!computeReachableMetricsFrom(
                map,
                robot_x_,
                robot_y_,
                false,
                reachable_distances,
                reachable_bottleneck_clearances)) {
            return false;
        }

        if (!computeReachableMetricsFrom(
                map,
                start_x_,
                start_y_,
                false,
                start_distances,
                start_bottleneck_clearances)) {
            return false;
        }

        const int width = static_cast<int>(map->info.width);
        const int height = static_cast<int>(map->info.height);
        const double resolution = map->info.resolution;
        const double current_projection = forwardProjection(robot_x_, robot_y_);

        bool found = false;
        double best_score = -std::numeric_limits<double>::max();
        int best_mx = 0;
        int best_my = 0;
        double best_path_distance = 0.0;
        double best_start_path_distance = 0.0;
        double best_clearance = 0.0;
        double best_bottleneck = 0.0;
        double best_open_ratio = 0.0;
        double best_projection = 0.0;

        for (int my = 1; my < height - 1; ++my) {
            for (int mx = 1; mx < width - 1; ++mx) {
                const int index = my * width + mx;

                if (map->data[index] != 0 ||
                    reachable_distances[index] < 0 ||
                    start_distances[index] < 0) {
                    continue;
                }

                const double path_distance = reachable_distances[index] * resolution;

                if (path_distance < BRANCH_ESCAPE_MIN_PATH_DISTANCE ||
                    path_distance > BRANCH_ESCAPE_MAX_PATH_DISTANCE) {
                    continue;
                }

                const double wx = mapToWorldX(map, mx);
                const double wy = mapToWorldY(map, my);

                bool is_blacklisted = false;
                for (const auto& bp : blacklist_) {
                    if (std::hypot(wx - bp.x, wy - bp.y) < BLACKLIST_RADIUS) {
                        is_blacklisted = true;
                        break;
                    }
                }

                if (is_blacklisted) {
                    continue;
                }

                if (isInEntranceReturnZone(wx, wy) ||
                    distanceFromStart(wx, wy) < ENTRANCE_NO_RETURN_WORLD_RADIUS) {
                    continue;
                }

                const double start_path_distance = start_distances[index] * resolution;

                if (start_path_distance < ENTRANCE_NO_RETURN_PATH_DISTANCE) {
                    continue;
                }

                if (start_path_distance + BRANCH_ESCAPE_MAX_START_PATH_LOSS <
                    max_start_path_distance_) {
                    continue;
                }

                const double projection = forwardProjection(wx, wy);

                if (projection < START_LINE_MARGIN) {
                    continue;
                }

                const double bottleneck = reachable_bottleneck_clearances[index];

                if (bottleneck < BRANCH_ESCAPE_MIN_BOTTLENECK) {
                    continue;
                }

                const double clearance = nearestObstacleDistance(map, mx, my, 0.30);

                if (clearance < BRANCH_ESCAPE_MIN_CLEARANCE) {
                    continue;
                }

                const double open_ratio = openSpaceRatio(map, mx, my, OPEN_SPACE_RADIUS);
                const double angle = std::atan2(wy - robot_y_, wx - robot_x_);
                const double angle_diff = std::abs(std::atan2(
                    std::sin(angle - robot_yaw_),
                    std::cos(angle - robot_yaw_)
                ));
                const double projection_loss =
                    std::max(0.0, current_projection - projection);
                const double start_path_loss =
                    std::max(0.0, max_start_path_distance_ - start_path_distance);

                double score = 0.0;
                score += 2.40 * open_ratio;
                score += 2.20 * clearance;
                score += 2.00 * bottleneck;
                score += 1.10 * path_distance;
                score += 0.70 * start_path_distance;
                score += 0.70 * projection;
                score -= 1.30 * projection_loss;
                score -= 0.80 * start_path_loss;
                score -= 0.25 * angle_diff;

                if (!found || score > best_score) {
                    found = true;
                    best_score = score;
                    best_mx = mx;
                    best_my = my;
                    best_path_distance = path_distance;
                    best_start_path_distance = start_path_distance;
                    best_clearance = clearance;
                    best_bottleneck = bottleneck;
                    best_open_ratio = open_ratio;
                    best_projection = projection;
                }
            }
        }

        if (!found) {
            return false;
        }

        goal.header.frame_id = "map";
        goal.header.stamp = this->now();
        goal.pose.position.x = mapToWorldX(map, best_mx);
        goal.pose.position.y = mapToWorldY(map, best_my);
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = std::sin(robot_yaw_ * 0.5);
        goal.pose.orientation.w = std::cos(robot_yaw_ * 0.5);

        current_goal_x_ = goal.pose.position.x;
        current_goal_y_ = goal.pose.position.y;

        RCLCPP_WARN(
            this->get_logger(),
            "포켓 branch 탈출 goal: x=%.2f, y=%.2f, score=%.2f, path=%.2f, start_path=%.2f, proj=%.2f, clearance=%.2f, bottleneck=%.2f, open=%.2f",
            current_goal_x_,
            current_goal_y_,
            best_score,
            best_path_distance,
            best_start_path_distance,
            best_projection,
            best_clearance,
            best_bottleneck,
            best_open_ratio
        );

        return true;
    }

    bool makeEscapeGoal(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        geometry_msgs::msg::PoseStamped& goal)
    {
        if (!updateRobotPose()) {
            return false;
        }

        const double pi = 3.14159265358979323846;
        const double current_projection = forwardProjection(robot_x_, robot_y_);

        bool found = false;
        double best_score = -std::numeric_limits<double>::max();
        double best_x = robot_x_;
        double best_y = robot_y_;
        double best_clearance = 0.0;

        const std::vector<double> distances = {
            ESCAPE_MAX_DISTANCE,
            0.45,
            0.35,
            ESCAPE_MIN_DISTANCE
        };

        const std::vector<double> angle_offsets = {
            0.0,
            pi / 12.0,
            -pi / 12.0,
            pi / 6.0,
            -pi / 6.0,
            pi / 4.0,
            -pi / 4.0,
            pi / 3.0,
            -pi / 3.0,
            pi / 2.0,
            -pi / 2.0
        };

        for (const double distance : distances) {
            for (const double angle_offset : angle_offsets) {
                const double yaw = robot_yaw_ + angle_offset;
                const double wx = robot_x_ + distance * std::cos(yaw);
                const double wy = robot_y_ + distance * std::sin(yaw);

                int mx = 0;
                int my = 0;

                if (!worldToMap(map, wx, wy, mx, my)) {
                    continue;
                }

                const int index = my * static_cast<int>(map->info.width) + mx;

                if (map->data[index] != 0) {
                    continue;
                }

                if (isInEntranceReturnZone(wx, wy)) {
                    continue;
                }

                const double projection = forwardProjection(wx, wy);

                // 완전한 입구 복귀 방향은 제외하되, 막힌 상황에서는 약간의 우회는 허용한다.
                if (projection + 0.18 < current_projection) {
                    continue;
                }

                const double clearance = nearestObstacleDistance(map, mx, my, 0.30);

                if (clearance < ESCAPE_MIN_CLEARANCE) {
                    continue;
                }

                if (!isStraightLineSafe(map, wx, wy, ESCAPE_LINE_CLEARANCE)) {
                    continue;
                }

                bool is_blacklisted = false;

                for (const auto& bp : blacklist_) {
                    if (std::hypot(wx - bp.x, wy - bp.y) < KEEP_MOVING_BLACKLIST_RADIUS) {
                        is_blacklisted = true;
                        break;
                    }
                }

                const double angle_cost = std::abs(std::atan2(
                    std::sin(yaw - robot_yaw_),
                    std::cos(yaw - robot_yaw_)
                ));

                double score = 0.0;
                score += 2.2 * distance;
                score += 3.0 * clearance;
                score += 0.9 * projection;
                score -= 0.7 * angle_cost;

                if (is_blacklisted) {
                    score -= 1.5;
                }

                if (!found || score > best_score) {
                    found = true;
                    best_score = score;
                    best_x = wx;
                    best_y = wy;
                    best_clearance = clearance;
                }
            }
        }

        if (!found) {
            return false;
        }

        goal.header.frame_id = "map";
        goal.header.stamp = this->now();
        goal.pose.position.x = best_x;
        goal.pose.position.y = best_y;
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        // 프론티어 탐색에서는 최종 방향이 중요하지 않다.
        // goal 방향을 강제하면 좁은 통로 근처에서 제자리 회전하다 충돌할 수 있다.
        goal.pose.orientation.z = std::sin(robot_yaw_ * 0.5);
        goal.pose.orientation.w = std::cos(robot_yaw_ * 0.5);

        current_goal_x_ = best_x;
        current_goal_y_ = best_y;

        RCLCPP_WARN(
            this->get_logger(),
            "강제 탈출 local goal: x=%.2f, y=%.2f, score=%.2f, clearance=%.2f",
            current_goal_x_,
            current_goal_y_,
            best_score,
            best_clearance
        );

        return true;
    }

    bool findBestFrontier(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        geometry_msgs::msg::PoseStamped& goal,
        bool allow_backtrack = false)
    {
        const int width = static_cast<int>(map->info.width);
        const int height = static_cast<int>(map->info.height);
        const double resolution = map->info.resolution;
        const double origin_x = map->info.origin.position.x;
        const double origin_y = map->info.origin.position.y;

        bool frontier_found = false;
        double best_score = -std::numeric_limits<double>::max();
        int best_mx = 0;
        int best_my = 0;
        double best_path_bottleneck_clearance = 0.0;
        double best_start_path_distance = 0.0;
        double best_current_path_distance = 0.0;
        int best_cluster_size = 0;

        const int dx[4] = {0, 0, -1, 1};
        const int dy[4] = {-1, 1, 0, 0};

        std::vector<bool> visited(width * height, false);
        std::vector<int> reachable_distances;
        std::vector<double> reachable_bottleneck_clearances;
        std::vector<int> start_distances;
        std::vector<double> start_bottleneck_clearances;

        if (!computeReachableMetrics(
                map,
                reachable_distances,
                reachable_bottleneck_clearances)) {
            return false;
        }

        if (!computeReachableMetricsFrom(
                map,
                start_x_,
                start_y_,
                false,
                start_distances,
                start_bottleneck_clearances)) {
            return false;
        }

        int robot_mx = 0;
        int robot_my = 0;
        double current_start_path = current_start_path_distance_;

        if (worldToMap(map, robot_x_, robot_y_, robot_mx, robot_my)) {
            const int robot_index = robot_my * width + robot_mx;

            if (start_distances[robot_index] >= 0) {
                current_start_path = start_distances[robot_index] * resolution;
                current_start_path_distance_ = current_start_path;
                max_start_path_distance_ = std::max(
                    max_start_path_distance_,
                    current_start_path
                );
            }
        }

        // strict 모드에서는 현재까지 가장 깊게 들어간 거리보다 너무 뒤쪽 frontier는 제외한다.
        // relaxed 모드에서는 dead-end 탈출을 위해 이 제한을 크게 완화한다.
        const double backtrack_allowance = allow_backtrack ?
            RELAXED_BACKTRACK_ALLOWANCE :
            START_PATH_BACKTRACK_ALLOWANCE;

        const double min_bottleneck = allow_backtrack ?
            std::max(0.075, PATH_BOTTLENECK_CLEARANCE - 0.030) :
            PATH_BOTTLENECK_CLEARANCE;

        const double min_goal_clearance = allow_backtrack ?
            std::max(0.085, MIN_FINAL_GOAL_CLEARANCE - 0.020) :
            MIN_FINAL_GOAL_CLEARANCE;

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

                // cluster 대표점들을 모두 보되, goal은 frontier 바로 앞의 안전한 reachable cell로 옮긴다.
                for (const int frontier_index : cluster) {
                    const int frontier_mx = frontier_index % width;
                    const int frontier_my = frontier_index / width;

                    const double frontier_wx = origin_x + (frontier_mx + 0.5) * resolution;
                    const double frontier_wy = origin_y + (frontier_my + 0.5) * resolution;

                    const double frontier_dist =
                        std::hypot(frontier_wx - robot_x_, frontier_wy - robot_y_);

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

                    int goal_mx = 0;
                    int goal_my = 0;

                    if (!findCenteredReachableGoalCell(
                            map,
                            reachable_distances,
                            reachable_bottleneck_clearances,
                            candidate_mx,
                            candidate_my,
                            goal_mx,
                            goal_my)) {
                        continue;
                    }

                    const int goal_index = goal_my * width + goal_mx;

                    if (reachable_distances[goal_index] < 0 || start_distances[goal_index] < 0) {
                        continue;
                    }

                    const double goal_wx = origin_x + (goal_mx + 0.5) * resolution;
                    const double goal_wy = origin_y + (goal_my + 0.5) * resolution;

                    if (isInEntranceReturnZone(goal_wx, goal_wy)) {
                        continue;
                    }

                    const double current_path_distance = reachable_distances[goal_index] * resolution;
                    const double start_path_distance = start_distances[goal_index] * resolution;
                    const double path_bottleneck_clearance = reachable_bottleneck_clearances[goal_index];
                    const double final_goal_distance =
                        std::hypot(goal_wx - robot_x_, goal_wy - robot_y_);

                    if (isEntranceReturnForbiddenCandidate(
                            goal_wx,
                            goal_wy,
                            start_path_distance)) {
                        continue;
                    }

                    if (final_goal_distance < MIN_FINAL_GOAL_DISTANCE) {
                        continue;
                    }

                    if (start_path_distance < MIN_START_PATH_GOAL_DISTANCE) {
                        continue;
                    }

                    // strict: 계속 더 깊은 쪽을 우선. relaxed: dead-end 탈출을 위해 뒤쪽 frontier도 허용.
                    if (start_path_distance + backtrack_allowance < max_start_path_distance_) {
                        continue;
                    }

                    if (path_bottleneck_clearance < min_bottleneck) {
                        continue;
                    }

                    const double clearance = nearestObstacleDistance(map, goal_mx, goal_my, 0.30);

                    if (clearance < min_goal_clearance) {
                        continue;
                    }

                    bool is_blacklisted = false;
                    for (const auto& bp : blacklist_) {
                        if (std::hypot(goal_wx - bp.x, goal_wy - bp.y) < BLACKLIST_RADIUS) {
                            is_blacklisted = true;
                            break;
                        }
                    }

                    if (is_blacklisted && !allow_backtrack) {
                        continue;
                    }

                    const double target_yaw = std::atan2(goal_wy - robot_y_, goal_wx - robot_x_);
                    const double angle_diff = std::abs(std::atan2(
                        std::sin(target_yaw - robot_yaw_),
                        std::cos(target_yaw - robot_yaw_)
                    ));

                    const double open_ratio = openSpaceRatio(map, goal_mx, goal_my, OPEN_SPACE_RADIUS);
                    const double cluster_length = static_cast<double>(cluster.size()) * resolution;
                    const double depth_gain = start_path_distance - current_start_path;
                    const double max_depth_gain = start_path_distance - max_start_path_distance_;
                    const double backtrack_amount = std::max(0.0, current_start_path - start_path_distance);

                    // 점수는 "가까운 열린 공간"이 아니라 "시작점에서 graph상 더 깊은 frontier"를 고르게 만든다.
                    // 그래서 특정 맵의 출구 방향을 하드코딩하지 않아도, 미로를 계속 진행하는 방향을 선택한다.
                    double score = 0.0;
                    score += 5.00 * start_path_distance;
                    score += 2.00 * std::max(0.0, max_depth_gain);
                    score += 1.30 * std::max(0.0, depth_gain);
                    score += 2.40 * path_bottleneck_clearance;
                    score += 1.20 * clearance;
                    score += 0.55 * cluster_length;
                    score += 0.80 * open_ratio;
                    score -= 0.45 * current_path_distance;
                    score -= 0.35 * angle_diff;
                    score -= (allow_backtrack ? 2.00 : 6.00) * backtrack_amount;
                    score -= 7.00 * std::max(0.0, WALL_CLEARANCE_COMFORT - clearance);
                    score -= 7.00 * std::max(0.0, WALL_CLEARANCE_COMFORT - path_bottleneck_clearance);

                    if (is_blacklisted) {
                        score -= 1.0;
                    }

                    // 완전히 입구 쪽으로 되돌아가는 후보는 relaxed에서도 낮은 점수를 준다.
                    if (hasLeftEntrance() && distanceFromStart(goal_wx, goal_wy) < START_RETURN_GOAL_RADIUS) {
                        score -= 6.0;
                    }

                    if (!frontier_found || score > best_score) {
                        frontier_found = true;
                        best_score = score;
                        best_mx = goal_mx;
                        best_my = goal_my;
                        best_path_bottleneck_clearance = path_bottleneck_clearance;
                        best_start_path_distance = start_path_distance;
                        best_current_path_distance = current_path_distance;
                        best_cluster_size = static_cast<int>(cluster.size());
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
        goal.pose.orientation.z = std::sin(robot_yaw_ * 0.5);
        goal.pose.orientation.w = std::cos(robot_yaw_ * 0.5);

        current_goal_x_ = goal.pose.position.x;
        current_goal_y_ = goal.pose.position.y;

        RCLCPP_INFO(
            this->get_logger(),
            "v12 frontier goal: x=%.2f, y=%.2f, score=%.2f, start_path=%.2f, current_path=%.2f, bottleneck=%.2f, cluster=%d, relaxed=%d",
            current_goal_x_,
            current_goal_y_,
            best_score,
            best_start_path_distance,
            best_current_path_distance,
            best_path_bottleneck_clearance,
            best_cluster_size,
            allow_backtrack ? 1 : 0
        );

        return true;
    }

    void sendGoal(const geometry_msgs::msg::PoseStamped& goal_pose)
    {
        if (!nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Nav2 action server 대기 중입니다. 다음 idle tick에서 재시도합니다.");
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
                        "Nav2가 목표를 거부했습니다. blacklist 초기화 후 재탐색"
                    );

                    blacklist_.clear();
                    consecutive_goal_failures_++;
                    goal_in_progress_ = false;
                    active_goal_handle_.reset();
                    trySendNextGoal();
                    return;
                }

                active_goal_handle_ = goal_handle;
            };

        send_goal_options.result_callback =
            std::bind(&MazeExplorer::resultCallback, this, _1);

        goal_in_progress_ = true;
        canceling_goal_ = false;
        goal_start_x_ = robot_x_;
        goal_start_y_ = robot_y_;
        goal_start_projection_ = forwardProjection(robot_x_, robot_y_);
        goal_start_path_distance_ = current_start_path_distance_;

        best_goal_distance_ =
            std::hypot(current_goal_x_ - robot_x_, current_goal_y_ - robot_y_);

        last_progress_time_ = this->now();

        nav_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void idleRetryCallback()
    {
        if (goal_in_progress_ || canceling_goal_ || escape_active_ || !latest_map_) {
            return;
        }

        if (!updateRobotPose()) {
            return;
        }

        trySendNextGoal();
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
        if (latest_map_) {
            updateStartPathProgress(latest_map_);
        }

        const double start_distance = distanceFromStart(robot_x_, robot_y_);
        const double projection = forwardProjection(robot_x_, robot_y_);

        if (hasLeftEntrance() &&
            (start_distance < START_RETURN_CANCEL_RADIUS ||
             projection + NO_RETURN_PROJECTION_BACKTRACK_ALLOWANCE <
             max_forward_projection_ ||
             start_distance + NO_RETURN_DISTANCE_BACKTRACK_ALLOWANCE <
             max_start_distance_)) {
            RCLCPP_WARN(
                this->get_logger(),
                "입구 복귀 방향 이동 감지. 현재 목표 취소: 시작거리=%.2f, 최대시작거리=%.2f, 현재투영=%.2f, 최대투영=%.2f",
                start_distance,
                max_start_distance_,
                projection,
                max_forward_projection_
            );

            cancelCurrentGoalAndBlacklist();
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

        cancelCurrentGoalAndBlacklist();
    }

    void resultCallback(const GoalHandleNav2::WrappedResult& result)
    {
        updateRobotPose();
        updateForwardProgress();
        if (latest_map_) {
            updateStartPathProgress(latest_map_);
        }

        const double moved_distance =
            std::hypot(robot_x_ - goal_start_x_, robot_y_ - goal_start_y_);
        const double projection_gain =
            forwardProjection(robot_x_, robot_y_) - goal_start_projection_;
        const double path_gain =
            current_start_path_distance_ - goal_start_path_distance_;

        bool should_escape_after_result = false;
        bool prefer_local_motion_after_result = false;
        std::string continue_reason = "after Nav2 result";

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            if (moved_distance < 0.08 &&
                projection_gain < 0.06 &&
                path_gain < 0.06) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "목표 성공 처리됐지만 실제 이동이 거의 없습니다. 더 먼 goal로 재생성: moved=%.2f, projection_gain=%.2f, path_gain=%.2f",
                    moved_distance,
                    projection_gain,
                    path_gain
                );

                consecutive_goal_failures_++;
                addBlacklistPoint(current_goal_x_, current_goal_y_, BLACKLIST_TTL_FAILURE);
                should_escape_after_result = canUseBackupRecovery();
                prefer_local_motion_after_result = true;
                continue_reason = "succeeded with almost no movement";
            } else {
                RCLCPP_INFO(
                    this->get_logger(),
                    "목표 도착: x=%.2f, y=%.2f, moved=%.2f, projection_gain=%.2f",
                    current_goal_x_,
                    current_goal_y_,
                    moved_distance,
                    projection_gain
                );
                consecutive_goal_failures_ = 0;
                addBlacklistPoint(current_goal_x_, current_goal_y_, BLACKLIST_TTL_SUCCESS);
                continue_reason = "goal succeeded";
            }
        } else if (canceling_goal_) {
            RCLCPP_WARN(this->get_logger(), "정체된 목표 취소 완료");
            consecutive_goal_failures_++;
            should_escape_after_result = canUseBackupRecovery();
            prefer_local_motion_after_result = true;
            continue_reason = "stuck goal canceled";
        } else {
            RCLCPP_WARN(
                this->get_logger(),
                "Nav2 목표 실패. 같은 지점을 즉시 금지하지 않고 재탐색합니다: x=%.2f, y=%.2f",
                current_goal_x_,
                current_goal_y_
            );

            consecutive_goal_failures_++;
            // 같은 실패 goal 반복만 짧게 막고, local fallback은 필요하면 blacklist를 무시한다.
            addBlacklistPoint(current_goal_x_, current_goal_y_, BLACKLIST_TTL_FAILURE);
            should_escape_after_result = canUseBackupRecovery();
            prefer_local_motion_after_result = true;
            continue_reason = "Nav2 goal failed";
        }

        active_goal_handle_.reset();
        canceling_goal_ = false;
        goal_in_progress_ = false;

        if (should_escape_after_result) {
            startEscapeRecovery("repeated Nav2 goal failure");
            if (escape_active_) {
                return;
            }

            prefer_local_motion_after_result = true;
            continue_reason = "backup recovery skipped";
        }

        if (prefer_local_motion_after_result) {
            startForwardCreep(continue_reason);
            if (escape_active_) {
                return;
            }
        }

        if (!trySendNextGoal() &&
            !escape_active_ &&
            !goal_in_progress_ &&
            !canceling_goal_) {
            startForwardCreep(continue_reason);
            return;
        }
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
