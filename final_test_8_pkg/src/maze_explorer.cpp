#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
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
#include <cstdint>
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

        // /goal_pose is also consumed by Nav2, so use a private topic only for visualization.
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/maze_explorer/goal_pose", 10);
        branch_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/maze_explorer/branch_markers", 10);
        candidate_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/maze_explorer/candidate_markers", 10);
        blacklist_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/maze_explorer/blacklist_markers",
            rclcpp::QoS(1).transient_local()
        );
        // Nav2의 cmd_vel을 safety filter로 보내기 위한 입력 토픽.
        // 실제 로봇으로 나가는 /cmd_vel은 safety filter가 publish해야 한다.
        escape_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_nav", 10);
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        watchdog_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MazeExplorer::watchdogCallback, this)
        );

        idle_retry_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(300),
            std::bind(&MazeExplorer::idleRetryCallback, this)
        );

        escape_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&MazeExplorer::escapeTimerCallback, this)
        );

        blacklist_marker_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MazeExplorer::publishBlacklistMarkers, this)
        );

        RCLCPP_INFO(this->get_logger(), "maze_explorer v12 no-return progress loaded");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr branch_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr candidate_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr blacklist_marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr escape_cmd_pub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::TimerBase::SharedPtr idle_retry_timer_;
    rclcpp::TimerBase::SharedPtr escape_timer_;
    rclcpp::TimerBase::SharedPtr blacklist_marker_timer_;
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
    double active_goal_projection_ = 0.0;
    double active_goal_start_distance_ = 0.0;
    double active_goal_start_path_distance_ = 0.0;
    bool pending_goal_is_branch_escape_ = false;
    double pending_goal_unknown_exposure_ = 0.0;
    bool active_goal_is_branch_escape_ = false;
    double active_goal_unknown_exposure_ = 0.0;

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

    struct DeadZoneEntry
    {
        double x;
        double y;
    };

    std::vector<BlacklistEntry> blacklist_;
    std::vector<DeadZoneEntry> dead_zones_;

    const double BLACKLIST_RADIUS = 0.30;
    const double DEAD_ZONE_RADIUS = 0.45;
    const double BLACKLIST_TTL_SUCCESS = 6.0;
    const double BLACKLIST_TTL_FAILURE = 30.0;
    const double BLACKLIST_TTL_CANCEL = 25.0;

    const double GOAL_CLEARANCE = 0.100;
    const double PATH_CLEARANCE = 0.095;
    const double PATH_BOTTLENECK_CLEARANCE = 0.100;
    const double GOAL_STANDOFF = 0.28;
    const double GOAL_SEARCH_RADIUS = 0.32;
    const double PREFERRED_GOAL_CLEARANCE = 0.18;
    const double PATH_BOTTLENECK_WEIGHT = 2.2;
    const double FRONTIER_CLUSTER_LENGTH_WEIGHT = 0.2;
    const double WALL_CLEARANCE_COMFORT = 0.16;
    const double WALL_CLEARANCE_PENALTY_WEIGHT = 9.0;

    const double MIN_GOAL_DISTANCE = 0.35;
    const double MIN_FINAL_GOAL_DISTANCE = 0.35;
    const double MIN_FINAL_GOAL_CLEARANCE = 0.102;

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
    const double RELAXED_BACKTRACK_ALLOWANCE = 0.45;
    const double START_PATH_BACKTRACK_ALLOWANCE = 0.30;
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
    const double KEEP_MOVING_BLACKLIST_RADIUS = 0.16;
    const double KEEP_MOVING_BACKTRACK_ALLOWANCE = 0.25;
    const double OPEN_SPACE_RADIUS = 0.40;
    const double OPEN_SPACE_MIN_RATIO = 0.40;
    const double FORWARD_ALIGNMENT_WEIGHT = 1.20;
    const double OPEN_SPACE_WEIGHT = 1.80;
    const double OPEN_RAY_MAX_DISTANCE = 1.10;
    const double OPEN_RAY_LINE_CLEARANCE = 0.085;
    const double OPEN_RAY_GOAL_CLEARANCE = 0.112;
    const double LOOKAHEAD_PIVOT_CLEARANCE = 0.085;
    const double LOOKAHEAD_GOAL_CLEARANCE = 0.112;

    const double STUCK_GOAL_IMPROVEMENT = 0.03;
    const double STUCK_TIMEOUT = 7.0;
    const double CONTACT_OSCILLATION_DISTANCE = 0.115;
    const double CONTACT_OSCILLATION_TIMEOUT = 0.80;
    const double CONTACT_REVERSE_SPEED = 0.055;
    const double CONTACT_REVERSE_DURATION = 0.55;
    const double CONTACT_REVERSE_REAR_CLEARANCE = 0.12;
    const double CONTACT_WIDE_RAY_SPEED = 0.055;
    const double CONTACT_WIDE_RAY_DURATION = 0.65;
    const double CONTACT_WIDE_RAY_MIN_DISTANCE = 0.18;
    const double CONTACT_WIDE_RAY_MAX_DISTANCE = 0.75;
    const double SCAN_FRESH_TIMEOUT = 1.50;
    const double ESCAPE_BACKUP_SPEED = 0.050;
    const double ESCAPE_BACKUP_DURATION = 0.40;
    const double ESCAPE_BACKUP_MIN_START_DISTANCE = 0.80;
    const int ESCAPE_BACKUP_MIN_FAILURES = 4;
    const double FORWARD_CREEP_SPEED = 0.080;
    const double FORWARD_CREEP_DURATION = 0.70;
    const double FORWARD_CREEP_PROJECTION_CHECK_DISTANCE = 0.30;
    const double FORWARD_CREEP_TURN_SPEED = 0.60;
    const double LOCAL_ESCAPE_RAY_MAX_DISTANCE = 0.85;
    const double LOCAL_ESCAPE_RAY_CLEARANCE = 0.060;
    const double LOCAL_ESCAPE_MIN_RAY_DISTANCE = 0.20;
    const double LOCAL_ESCAPE_MIN_SCAN_DISTANCE = 0.135;
    const double LOCAL_ESCAPE_SCAN_ONLY_DISTANCE = 0.24;
    const double LOCAL_ESCAPE_SCAN_HALF_WIDTH = 0.22;
    const double LOCAL_ESCAPE_REVERSE_SCAN_DISTANCE = 0.16;
    const double LOCAL_ESCAPE_REVERSE_SPEED = 0.045;
    const double LOCAL_ESCAPE_REVERSE_PROJECTION_ALLOWANCE = 0.05;
    const double LOCAL_ESCAPE_FORWARD_SCAN_DISTANCE = 0.16;
    const double LOCAL_ESCAPE_FORWARD_RAY_DISTANCE = 0.30;
    const double LOCAL_ESCAPE_STEP_DISTANCE = 0.45;
    const double LOCAL_ESCAPE_ALIGN_YAW = 0.35;
    const double LOCAL_ESCAPE_MAX_TURN_SPEED = 0.85;
    const int LOCAL_ESCAPE_SPIN_FAILURES_BEFORE_REVERSE = 1;
    const double BRANCH_ESCAPE_MIN_PATH_DISTANCE = 0.70;
    const double BRANCH_ESCAPE_MAX_PATH_DISTANCE = 2.00;
    const double BRANCH_ESCAPE_MAX_START_PATH_LOSS = 1.40;
    const double BRANCH_ESCAPE_MIN_CLEARANCE = 0.120;
    const double BRANCH_ESCAPE_MIN_BOTTLENECK = 0.105;
    const double BRANCH_ESCAPE_UNKNOWN_LOOKAHEAD = 0.70;
    const double BRANCH_ESCAPE_UNKNOWN_WEIGHT = 3.20;
    const double CANDIDATE_MARKER_LIFETIME = 2.5;

    const double ESCAPE_MIN_DISTANCE = 0.22;
    const double ESCAPE_MAX_DISTANCE = 0.55;
    const double ESCAPE_MIN_CLEARANCE = 0.080;
    const double ESCAPE_LINE_CLEARANCE = 0.070;

    const int MIN_FRONTIER_CLUSTER_SIZE = 6;
    const int MAX_FRONTIER_CANDIDATES_PER_CLUSTER = 48;

    double current_goal_x_ = 0.0;
    double current_goal_y_ = 0.0;
    double goal_start_x_ = 0.0;
    double goal_start_y_ = 0.0;
    double goal_start_projection_ = 0.0;
    double goal_start_path_distance_ = 0.0;
    int consecutive_goal_failures_ = 0;
    bool contact_oscillation_active_ = false;
    rclcpp::Time contact_oscillation_since_;
    bool pending_contact_recovery_ = false;
    std::string pending_contact_recovery_reason_;
    rclcpp::Time pending_contact_recovery_since_;
    int local_escape_spin_failures_ = 0;
    bool start_metrics_cache_valid_ = false;
    int64_t cached_start_map_stamp_ns_ = 0;
    uint32_t cached_start_map_width_ = 0;
    uint32_t cached_start_map_height_ = 0;
    double cached_start_map_resolution_ = 0.0;
    double cached_start_map_origin_x_ = 0.0;
    double cached_start_map_origin_y_ = 0.0;
    std::vector<int> cached_start_distances_;
    std::vector<double> cached_start_bottleneck_clearances_;

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
                    "시작점 방향 되감기 감지. 현재 목표 취소: 현재거리=%.2f, 최대거리=%.2f, 현재투영=%.2f, 최대투영=%.2f, 현재경로거리=%.2f, 최대경로거리=%.2f, goal경로거리=%.2f",
                    distanceFromStart(robot_x_, robot_y_),
                    max_start_distance_,
                    forwardProjection(robot_x_, robot_y_),
                    max_forward_projection_,
                    current_start_path_distance_,
                    max_start_path_distance_,
                    active_goal_start_path_distance_
                );

                cancelCurrentGoalAndBlacklist();
            }

            return;
        }

        processPendingContactRecovery();
        if (pending_contact_recovery_ || escape_active_) {
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
            start_metrics_cache_valid_ = false;
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

    bool isScanFresh() const
    {
        return latest_scan_ &&
               !latest_scan_->ranges.empty() &&
               latest_scan_->angle_increment > 0.0 &&
               (this->now() - last_scan_time_).seconds() <= SCAN_FRESH_TIMEOUT;
    }

    double scanSectorMin(double base_link_yaw, double half_width) const
    {
        if (!latest_scan_ ||
            latest_scan_->ranges.empty() ||
            latest_scan_->angle_increment <= 0.0 ||
            (this->now() - last_scan_time_).seconds() > SCAN_FRESH_TIMEOUT) {
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

    double nearestContactDistance() const
    {
        const double front_contact = scanSectorMin(
            0.0,
            LOCAL_ESCAPE_SCAN_HALF_WIDTH
        );
        const double left_corner_contact = scanSectorMin(
            3.14159265358979323846 / 4.0,
            LOCAL_ESCAPE_SCAN_HALF_WIDTH
        );
        const double right_corner_contact = scanSectorMin(
            -3.14159265358979323846 / 4.0,
            LOCAL_ESCAPE_SCAN_HALF_WIDTH
        );
        const double left_side_contact = scanSectorMin(
            3.14159265358979323846 / 2.0,
            LOCAL_ESCAPE_SCAN_HALF_WIDTH
        );
        const double right_side_contact = scanSectorMin(
            -3.14159265358979323846 / 2.0,
            LOCAL_ESCAPE_SCAN_HALF_WIDTH
        );

        return std::min(
            std::min(front_contact, std::min(left_corner_contact, right_corner_contact)),
            std::min(left_side_contact, right_side_contact)
        );
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

        if (blacklist_.size() > 32) {
            blacklist_.erase(blacklist_.begin(), blacklist_.begin() + 8);
        }

        publishBlacklistMarkers();
    }

    void addDeadZone(double wx, double wy, const std::string& reason)
    {
        for (const auto& dz : dead_zones_) {
            if (std::hypot(wx - dz.x, wy - dz.y) < DEAD_ZONE_RADIUS * 0.60) {
                return;
            }
        }

        dead_zones_.push_back({wx, wy});

        if (dead_zones_.size() > 24) {
            dead_zones_.erase(dead_zones_.begin(), dead_zones_.begin() + 4);
        }

        RCLCPP_WARN(
            this->get_logger(),
            "dead-zone 등록: x=%.2f, y=%.2f, radius=%.2f, reason=%s",
            wx,
            wy,
            DEAD_ZONE_RADIUS,
            reason.c_str()
        );

        visualization_msgs::msg::MarkerArray markers;
        markers.markers.push_back(makeDeleteAllCandidateMarkers());
        appendDeadZoneMarkers(markers);
        publishCandidateMarkers(markers);
        publishBlacklistMarkers();
    }

    bool isBlacklistedPoint(double wx, double wy, double radius) const
    {
        for (const auto& dz : dead_zones_) {
            if (std::hypot(wx - dz.x, wy - dz.y) < DEAD_ZONE_RADIUS) {
                return true;
            }
        }

        for (const auto& bp : blacklist_) {
            if (std::hypot(wx - bp.x, wy - bp.y) < radius) {
                return true;
            }
        }

        return false;
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

        // 막힌 포켓에서 빠져나와 옆/앞쪽 branch로 가려면 현재 위치 기준으로
        // 잠깐 얕은 graph-depth를 지나야 할 수 있다. 목표 자체가 최대 진입 깊이
        // 근처이거나 더 깊다면 그 중간 후퇴는 goal 취소 조건으로 보지 않는다.
        if (goal_in_progress_ &&
            active_goal_start_path_distance_ + START_PATH_CANCEL_BACKTRACK >=
            max_start_path_distance_) {
            return false;
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

        const std::vector<int>* start_distances = nullptr;
        const std::vector<double>* start_bottleneck_clearances = nullptr;

        if (!getStartReachableMetrics(
                map,
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

        if ((*start_distances)[index] < 0) {
            return true;
        }

        const double start_path_distance =
            (*start_distances)[index] * map->info.resolution;

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

    double unknownExposureFrom(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        double wx,
        double wy,
        double yaw,
        double max_distance)
    {
        const int width = static_cast<int>(map->info.width);
        const int height = static_cast<int>(map->info.height);
        const double step = std::max(0.02, static_cast<double>(map->info.resolution));
        const std::vector<double> angle_offsets = {
            0.0,
            0.26,
            -0.26,
            0.52,
            -0.52
        };

        double best_exposure = 0.0;

        for (const double angle_offset : angle_offsets) {
            const double ray_yaw = yaw + angle_offset;

            for (double distance = step; distance <= max_distance; distance += step) {
                const double rx = wx + distance * std::cos(ray_yaw);
                const double ry = wy + distance * std::sin(ray_yaw);

                int mx = 0;
                int my = 0;

                if (!worldToMap(map, rx, ry, mx, my)) {
                    break;
                }

                if (mx < 0 || mx >= width || my < 0 || my >= height) {
                    break;
                }

                const int value = map->data[my * width + mx];

                if (value == -1) {
                    const double exposure = 1.0 - (distance / max_distance);
                    best_exposure = std::max(best_exposure, std::max(0.20, exposure));
                    break;
                }

                if (value >= 50) {
                    break;
                }
            }
        }

        return best_exposure;
    }

    visualization_msgs::msg::Marker makeBranchMarker(
        int id,
        double wx,
        double wy,
        double score,
        double unknown_exposure,
        bool selected)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = selected ? "branch_selected" : "branch_candidates";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = wx;
        marker.pose.position.y = wy;
        marker.pose.position.z = selected ? 0.08 : 0.04;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = selected ? 0.18 : 0.07;
        marker.scale.y = selected ? 0.18 : 0.07;
        marker.scale.z = selected ? 0.18 : 0.07;
        marker.color.a = selected ? 1.0 : 0.65;

        if (selected) {
            marker.color.r = 1.0;
            marker.color.g = 0.82;
            marker.color.b = 0.05;
        } else {
            marker.color.r = 0.10;
            marker.color.g = std::clamp(0.35 + unknown_exposure, 0.35, 1.0);
            marker.color.b = 1.0;
        }

        marker.lifetime = rclcpp::Duration::from_seconds(2.0);
        (void)score;
        return marker;
    }

    void publishBranchMarkers(const visualization_msgs::msg::MarkerArray& markers)
    {
        if (branch_marker_pub_) {
            branch_marker_pub_->publish(markers);
        }
    }

    visualization_msgs::msg::Marker makeDeleteAllCandidateMarkers()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.action = visualization_msgs::msg::Marker::DELETEALL;
        return marker;
    }

    visualization_msgs::msg::Marker makeCandidateSphere(
        const std::string& ns,
        int id,
        double wx,
        double wy,
        double red,
        double green,
        double blue,
        double alpha,
        double scale,
        double z = 0.07)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = wx;
        marker.pose.position.y = wy;
        marker.pose.position.z = z;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;
        marker.color.r = red;
        marker.color.g = green;
        marker.color.b = blue;
        marker.color.a = alpha;
        marker.lifetime = rclcpp::Duration::from_seconds(CANDIDATE_MARKER_LIFETIME);
        return marker;
    }

    visualization_msgs::msg::Marker makeCandidateRay(
        const std::string& ns,
        int id,
        double yaw,
        double distance,
        double red,
        double green,
        double blue,
        double alpha)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = robot_x_;
        marker.pose.position.y = robot_y_;
        marker.pose.position.z = 0.10;
        marker.pose.orientation.z = std::sin(yaw * 0.5);
        marker.pose.orientation.w = std::cos(yaw * 0.5);
        marker.scale.x = std::max(0.06, distance);
        marker.scale.y = 0.025;
        marker.scale.z = 0.025;
        marker.color.r = red;
        marker.color.g = green;
        marker.color.b = blue;
        marker.color.a = alpha;
        marker.lifetime = rclcpp::Duration::from_seconds(CANDIDATE_MARKER_LIFETIME);
        return marker;
    }

    visualization_msgs::msg::Marker makeDeadZoneMarker(int id, const DeadZoneEntry& dz)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "dead_zones";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = dz.x;
        marker.pose.position.y = dz.y;
        marker.pose.position.z = 0.02;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = DEAD_ZONE_RADIUS * 2.0;
        marker.scale.y = DEAD_ZONE_RADIUS * 2.0;
        marker.scale.z = 0.04;
        marker.color.r = 1.0;
        marker.color.g = 0.05;
        marker.color.b = 0.02;
        marker.color.a = 0.22;
        marker.lifetime = rclcpp::Duration::from_seconds(CANDIDATE_MARKER_LIFETIME);
        return marker;
    }

    visualization_msgs::msg::Marker makeBlacklistMarker(int id, const BlacklistEntry& bp)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "blacklist";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = bp.x;
        marker.pose.position.y = bp.y;
        marker.pose.position.z = 0.055;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = BLACKLIST_RADIUS * 2.0;
        marker.scale.y = BLACKLIST_RADIUS * 2.0;
        marker.scale.z = 0.08;
        marker.color.r = 0.01;
        marker.color.g = 0.01;
        marker.color.b = 0.01;
        marker.color.a = 0.74;
        marker.lifetime = rclcpp::Duration::from_seconds(1.5);
        return marker;
    }

    visualization_msgs::msg::Marker makeBlacklistLabelMarker(int id, const BlacklistEntry& bp)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "blacklist_labels";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = bp.x;
        marker.pose.position.y = bp.y;
        marker.pose.position.z = 0.24;
        marker.pose.orientation.w = 1.0;
        marker.scale.z = 0.12;
        marker.color.r = 1.0;
        marker.color.g = 0.88;
        marker.color.b = 0.05;
        marker.color.a = 1.0;
        marker.text = "BLACKLIST";
        marker.lifetime = rclcpp::Duration::from_seconds(1.5);
        return marker;
    }

    void appendDeadZoneMarkers(visualization_msgs::msg::MarkerArray& markers)
    {
        int id = 0;
        for (const auto& dz : dead_zones_) {
            markers.markers.push_back(makeDeadZoneMarker(id++, dz));
        }
    }

    void appendBlacklistMarkers(visualization_msgs::msg::MarkerArray& markers)
    {
        int id = 0;
        for (const auto& bp : blacklist_) {
            markers.markers.push_back(makeBlacklistMarker(id, bp));
            markers.markers.push_back(makeBlacklistLabelMarker(id, bp));
            ++id;
        }
    }

    void publishBlacklistMarkers()
    {
        if (!blacklist_marker_pub_) {
            return;
        }

        pruneExpiredBlacklist();

        visualization_msgs::msg::MarkerArray markers;
        markers.markers.push_back(makeDeleteAllCandidateMarkers());
        appendDeadZoneMarkers(markers);
        appendBlacklistMarkers(markers);
        blacklist_marker_pub_->publish(markers);
    }

    void publishCandidateMarkers(visualization_msgs::msg::MarkerArray markers)
    {
        if (!candidate_marker_pub_) {
            return;
        }

        candidate_marker_pub_->publish(markers);
    }

    void publishSelectedGoalMarker(double wx, double wy, const std::string& ns)
    {
        visualization_msgs::msg::MarkerArray markers;
        markers.markers.push_back(makeDeleteAllCandidateMarkers());
        appendDeadZoneMarkers(markers);
        markers.markers.push_back(makeCandidateSphere(
            ns,
            10000,
            wx,
            wy,
            1.0,
            0.85,
            0.0,
            1.0,
            0.18,
            0.10
        ));
        publishCandidateMarkers(markers);
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

    bool getStartReachableMetrics(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        const std::vector<int>*& distances,
        const std::vector<double>*& bottleneck_clearances)
    {
        if (!is_start_position_saved_) {
            return false;
        }

        const int64_t map_stamp_ns = rclcpp::Time(map->header.stamp).nanoseconds();
        const bool cache_matches =
            start_metrics_cache_valid_ &&
            cached_start_map_stamp_ns_ == map_stamp_ns &&
            cached_start_map_width_ == map->info.width &&
            cached_start_map_height_ == map->info.height &&
            std::abs(cached_start_map_resolution_ - map->info.resolution) < 1e-9 &&
            std::abs(cached_start_map_origin_x_ - map->info.origin.position.x) < 1e-9 &&
            std::abs(cached_start_map_origin_y_ - map->info.origin.position.y) < 1e-9;

        if (!cache_matches) {
            if (!computeReachableMetricsFrom(
                    map,
                    start_x_,
                    start_y_,
                    false,
                    cached_start_distances_,
                    cached_start_bottleneck_clearances_)) {
                start_metrics_cache_valid_ = false;
                return false;
            }

            cached_start_map_stamp_ns_ = map_stamp_ns;
            cached_start_map_width_ = map->info.width;
            cached_start_map_height_ = map->info.height;
            cached_start_map_resolution_ = map->info.resolution;
            cached_start_map_origin_x_ = map->info.origin.position.x;
            cached_start_map_origin_y_ = map->info.origin.position.y;
            start_metrics_cache_valid_ = true;
        }

        distances = &cached_start_distances_;
        bottleneck_clearances = &cached_start_bottleneck_clearances_;
        return true;
    }

    bool updateStartPathProgress(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map)
    {
        if (!is_start_position_saved_) {
            return false;
        }

        const int width = static_cast<int>(map->info.width);
        const double resolution = map->info.resolution;

        const std::vector<int>* start_distances = nullptr;
        const std::vector<double>* start_bottleneck_clearances = nullptr;

        if (!getStartReachableMetrics(
                map,
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

        if ((*start_distances)[robot_index] < 0) {
            return false;
        }

        current_start_path_distance_ = (*start_distances)[robot_index] * resolution;
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
        pending_goal_is_branch_escape_ = false;
        pending_goal_unknown_exposure_ = 0.0;

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
                    found_goal = findBestFrontier(latest_map_, next_goal, true);
                    allow_branch_escape_goal = found_goal;
                }

                if (!found_goal) {
                    found_goal = makeLookaheadTurnGoal(latest_map_, next_goal) ||
                                 makeOpenRayGoal(latest_map_, next_goal);
                    allow_branch_escape_goal = found_goal;
                }

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
            visualization_msgs::msg::MarkerArray markers;
            markers.markers.push_back(makeDeleteAllCandidateMarkers());
            appendDeadZoneMarkers(markers);
            publishCandidateMarkers(markers);

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
            publishSelectedGoalMarker(
                next_goal.pose.position.x,
                next_goal.pose.position.y,
                "blocked_goal"
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

    bool startWideRayRecovery(const std::string& reason)
    {
        if (escape_active_ || goal_in_progress_ || canceling_goal_) {
            return false;
        }

        if (!latest_map_ || !updateRobotPose()) {
            return false;
        }

        const bool scan_ready = isScanFresh();

        if (!scan_ready) {
            return false;
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
            -pi * 2.0 / 3.0,
            pi * 3.0 / 4.0,
            -pi * 3.0 / 4.0
        };

        bool found_direction = false;
        double best_score = -std::numeric_limits<double>::max();
        double best_yaw = robot_yaw_;
        double best_ray = 0.0;
        double best_scan_clearance = 0.0;
        double best_projected_next = current_projection;

        for (const double offset : angle_offsets) {
            const double yaw = robot_yaw_ + offset;
            const double scan_clearance = scanSectorMin(
                offset,
                LOCAL_ESCAPE_SCAN_HALF_WIDTH
            );
            const double ray_distance = rayClearDistance(
                latest_map_,
                yaw,
                CONTACT_WIDE_RAY_MAX_DISTANCE,
                LOCAL_ESCAPE_RAY_CLEARANCE
            );

            if (scan_clearance < CONTACT_OSCILLATION_DISTANCE &&
                ray_distance < CONTACT_WIDE_RAY_MIN_DISTANCE) {
                continue;
            }

            const double step_distance = std::min(
                LOCAL_ESCAPE_STEP_DISTANCE,
                std::max(0.0, ray_distance - 0.04)
            );

            if (step_distance <= 0.05) {
                continue;
            }

            const double wx = robot_x_ + step_distance * std::cos(yaw);
            const double wy = robot_y_ + step_distance * std::sin(yaw);
            const double projected_next = forwardProjection(wx, wy);

            if (hasLeftEntrance() &&
                (isInEntranceReturnZone(wx, wy) || projected_next < START_LINE_MARGIN)) {
                continue;
            }

            const double projection_loss =
                std::max(0.0, current_projection - projected_next);
            double score = 0.0;
            score += 2.20 * ray_distance;
            score += 1.40 * std::min(scan_clearance, CONTACT_WIDE_RAY_MAX_DISTANCE);
            score -= 0.45 * std::abs(offset);
            score -= 0.90 * projection_loss;

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
            return false;
        }

        const double yaw_error = std::atan2(
            std::sin(best_yaw - robot_yaw_),
            std::cos(best_yaw - robot_yaw_)
        );

        recovery_linear_speed_ = CONTACT_WIDE_RAY_SPEED;
        if (std::abs(yaw_error) > LOCAL_ESCAPE_ALIGN_YAW) {
            recovery_linear_speed_ = CONTACT_WIDE_RAY_SPEED * 0.35;
        }

        recovery_angular_speed_ = std::clamp(
            1.20 * yaw_error,
            -LOCAL_ESCAPE_MAX_TURN_SPEED,
            LOCAL_ESCAPE_MAX_TURN_SPEED
        );

        geometry_msgs::msg::Twist stop;
        escape_cmd_pub_->publish(stop);

        escape_active_ = true;
        escape_until_ = this->now() + rclcpp::Duration::from_seconds(CONTACT_WIDE_RAY_DURATION);
        local_escape_spin_failures_ = 0;

        RCLCPP_WARN(
            this->get_logger(),
            "접촉 복구: 가장 넓은 ray 방향으로 짧게 이동합니다. linear=%.3f, angular=%.3f, map_ray=%.2f, scan=%.2f, proj=%.2f -> %.2f, reason=%s",
            recovery_linear_speed_,
            recovery_angular_speed_,
            best_ray,
            best_scan_clearance,
            current_projection,
            best_projected_next,
            reason.c_str()
        );

        return true;
    }

    void startContactReverseRecovery(const std::string& reason)
    {
        if (escape_active_ || goal_in_progress_ || canceling_goal_) {
            return;
        }

        const double contact_distance = nearestContactDistance();

        if (contact_distance >= CONTACT_OSCILLATION_DISTANCE) {
            return;
        }

        const double rear_clearance = scanSectorMin(
            3.14159265358979323846,
            LOCAL_ESCAPE_SCAN_HALF_WIDTH
        );

        if (rear_clearance < CONTACT_REVERSE_REAR_CLEARANCE) {
            RCLCPP_WARN(
                this->get_logger(),
                "접촉 후진 복구 생략: rear=%.3f, contact=%.3f, reason=%s",
                rear_clearance,
                contact_distance,
                reason.c_str()
            );
            startWideRayRecovery(reason + " rear blocked");
            return;
        }

        geometry_msgs::msg::Twist stop;
        escape_cmd_pub_->publish(stop);

        escape_active_ = true;
        escape_until_ = this->now() + rclcpp::Duration::from_seconds(CONTACT_REVERSE_DURATION);
        recovery_linear_speed_ = -CONTACT_REVERSE_SPEED;
        recovery_angular_speed_ = 0.0;
        local_escape_spin_failures_ = 0;

        RCLCPP_WARN(
            this->get_logger(),
            "접촉 goal 실패 후 짧은 후진 복구: %.2fs 동안 %.3fm/s, contact=%.3f, rear=%.3f, reason=%s",
            CONTACT_REVERSE_DURATION,
            CONTACT_REVERSE_SPEED,
            contact_distance,
            rear_clearance,
            reason.c_str()
        );
    }

    void requestContactRecovery(const std::string& reason)
    {
        if (!pending_contact_recovery_) {
            pending_contact_recovery_since_ = this->now();
        }

        pending_contact_recovery_ = true;
        pending_contact_recovery_reason_ = reason;

        if (goal_in_progress_ && !canceling_goal_) {
            cancelCurrentGoalAndBlacklist();
        }
    }

    void processPendingContactRecovery()
    {
        if (!pending_contact_recovery_ ||
            escape_active_ ||
            goal_in_progress_ ||
            canceling_goal_) {
            return;
        }

        const std::string reason = pending_contact_recovery_reason_;

        startContactReverseRecovery(reason);
        if (escape_active_) {
            pending_contact_recovery_ = false;
            return;
        }

        if (startWideRayRecovery(reason + " pending wide ray")) {
            pending_contact_recovery_ = false;
            return;
        }

        if ((this->now() - pending_contact_recovery_since_).seconds() > 1.50) {
            RCLCPP_WARN(
                this->get_logger(),
                "접촉 복구 시작 실패. 새 goal 재탐색으로 전환합니다. reason=%s",
                reason.c_str()
            );
            pending_contact_recovery_ = false;
            trySendNextGoal();
        }
    }

    void startForwardCreep(const std::string& reason)
    {
        if (escape_active_ || goal_in_progress_ || canceling_goal_) {
            return;
        }

        if (!updateRobotPose()) {
            return;
        }

        if (nearestContactDistance() < CONTACT_OSCILLATION_DISTANCE) {
            startContactReverseRecovery(reason + " contact");
            if (escape_active_) {
                return;
            }

            if (startWideRayRecovery(reason + " contact")) {
                return;
            }
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
        const bool scan_ready = isScanFresh();
        double best_score = -std::numeric_limits<double>::max();
        double best_yaw = robot_yaw_;
        double best_ray = 0.0;
        double best_scan_clearance = 0.0;
        double best_projected_next = current_projection;
        double best_scan_only_score = -std::numeric_limits<double>::max();
        double best_scan_only_yaw = robot_yaw_;
        double best_scan_only_clearance = 0.0;
        double best_scan_only_projection = current_projection;
        visualization_msgs::msg::MarkerArray local_markers;
        local_markers.markers.push_back(makeDeleteAllCandidateMarkers());
        appendDeadZoneMarkers(local_markers);
        int local_marker_id = 0;

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
            local_markers.markers.push_back(makeCandidateRay(
                "local_forward",
                local_marker_id++,
                robot_yaw_,
                std::min(front_scan_clearance, LOCAL_ESCAPE_RAY_MAX_DISTANCE),
                forward_keeps_progress ? 0.10 : 1.0,
                forward_keeps_progress ? 0.95 : 0.20,
                0.25,
                0.80
            ));

            if (front_scan_clearance >= LOCAL_ESCAPE_FORWARD_SCAN_DISTANCE &&
                (front_ray_distance >= LOCAL_ESCAPE_FORWARD_RAY_DISTANCE ||
                 front_scan_clearance >= LOCAL_ESCAPE_SCAN_ONLY_DISTANCE) &&
                forward_keeps_progress) {
                local_escape_spin_failures_ = 0;
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
                local_markers.markers.push_back(makeCandidateSphere(
                    "local_selected",
                    10000,
                    robot_x_ + 0.32 * std::cos(robot_yaw_),
                    robot_y_ + 0.32 * std::sin(robot_yaw_),
                    1.0,
                    0.85,
                    0.0,
                    1.0,
                    0.14,
                    0.12
                ));
                publishCandidateMarkers(local_markers);
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
                best_scan_only_projection = scan_step_projection;
            }

            if (scan_clearance < LOCAL_ESCAPE_MIN_SCAN_DISTANCE) {
                local_markers.markers.push_back(makeCandidateRay(
                    "local_rejected_scan",
                    local_marker_id++,
                    yaw,
                    std::max(0.08, std::min(scan_clearance, LOCAL_ESCAPE_RAY_MAX_DISTANCE)),
                    1.0,
                    0.12,
                    0.08,
                    0.45
                ));
                continue;
            }

            const double ray_distance = rayClearDistance(
                latest_map_,
                yaw,
                LOCAL_ESCAPE_RAY_MAX_DISTANCE,
                LOCAL_ESCAPE_RAY_CLEARANCE
            );

            if (ray_distance < LOCAL_ESCAPE_MIN_RAY_DISTANCE) {
                if (scan_clearance < LOCAL_ESCAPE_SCAN_ONLY_DISTANCE) {
                    local_markers.markers.push_back(makeCandidateRay(
                        "local_rejected_map",
                        local_marker_id++,
                        yaw,
                        std::max(0.08, std::min(scan_clearance, LOCAL_ESCAPE_RAY_MAX_DISTANCE)),
                        1.0,
                        0.45,
                        0.05,
                        0.50
                    ));
                    continue;
                }

                const double projected_next = scan_step_projection;

                if (hasLeftEntrance() &&
                    projected_next + NO_RETURN_PROJECTION_BACKTRACK_ALLOWANCE <
                    max_forward_projection_ - 0.35) {
                    local_markers.markers.push_back(makeCandidateRay(
                        "local_rejected_return",
                        local_marker_id++,
                        yaw,
                        std::min(scan_clearance, LOCAL_ESCAPE_RAY_MAX_DISTANCE),
                        0.95,
                        0.05,
                        0.95,
                        0.45
                    ));
                    continue;
                }

                double score = 0.0;
                score += 1.80 * scan_clearance;
                score += 1.00 * (projected_next - current_projection);
                score -= 0.30 * std::abs(offset);

                if (!found_direction || score > best_score) {
                    found_direction = true;
                    best_score = score;
                    best_yaw = yaw;
                    best_ray = ray_distance;
                    best_scan_clearance = scan_clearance;
                    best_projected_next = projected_next;
                }

                local_markers.markers.push_back(makeCandidateRay(
                    "local_scan_only",
                    local_marker_id++,
                    yaw,
                    std::min(scan_clearance, LOCAL_ESCAPE_RAY_MAX_DISTANCE),
                    0.15,
                    1.0,
                    0.15,
                    0.75
                ));
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
                local_markers.markers.push_back(makeCandidateRay(
                    "local_rejected_return",
                    local_marker_id++,
                    yaw,
                    step_distance,
                    0.95,
                    0.05,
                    0.95,
                    0.45
                ));
                continue;
            }

            if (hasLeftEntrance() &&
                projected_next + NO_RETURN_PROJECTION_BACKTRACK_ALLOWANCE <
                max_forward_projection_ - 0.25) {
                local_markers.markers.push_back(makeCandidateRay(
                    "local_rejected_return",
                    local_marker_id++,
                    yaw,
                    step_distance,
                    0.95,
                    0.05,
                    0.95,
                    0.45
                ));
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

            local_markers.markers.push_back(makeCandidateRay(
                "local_map_scan",
                local_marker_id++,
                yaw,
                step_distance,
                0.05,
                0.85,
                1.0,
                0.70
            ));
        }

        if (!found_direction) {
            local_escape_spin_failures_++;
            best_yaw = scan_ready ? best_scan_only_yaw : start_yaw_;
            best_ray = 0.0;
            best_scan_clearance = best_scan_only_clearance;
            best_projected_next = best_scan_only_projection;
        } else {
            local_escape_spin_failures_ = 0;
        }

        const double yaw_error = std::atan2(
            std::sin(best_yaw - robot_yaw_),
            std::cos(best_yaw - robot_yaw_)
        );

        if (!found_direction &&
            local_escape_spin_failures_ >= LOCAL_ESCAPE_SPIN_FAILURES_BEFORE_REVERSE) {
            if (startWideRayRecovery(reason + " repeated spin")) {
                local_escape_spin_failures_ = 0;
                publishCandidateMarkers(local_markers);
                return;
            }

            if (scan_ready) {
                const double rear_clearance = scanSectorMin(
                    3.14159265358979323846,
                    LOCAL_ESCAPE_SCAN_HALF_WIDTH
                );
                const double reverse_x =
                    robot_x_ -
                    FORWARD_CREEP_PROJECTION_CHECK_DISTANCE * std::cos(robot_yaw_);
                const double reverse_y =
                    robot_y_ -
                    FORWARD_CREEP_PROJECTION_CHECK_DISTANCE * std::sin(robot_yaw_);
                const double reverse_projection =
                    forwardProjection(reverse_x, reverse_y);
                const bool contact_escape =
                    nearestContactDistance() < CONTACT_OSCILLATION_DISTANCE * 1.25;
                const bool reverse_avoids_entrance =
                    !hasLeftEntrance() ||
                    (!isInEntranceReturnZone(reverse_x, reverse_y) &&
                     reverse_projection >= START_LINE_MARGIN);

                if (rear_clearance > LOCAL_ESCAPE_REVERSE_SCAN_DISTANCE &&
                    (reverse_avoids_entrance || contact_escape)) {
                    geometry_msgs::msg::Twist stop;
                    escape_cmd_pub_->publish(stop);

                    escape_active_ = true;
                    escape_until_ =
                        this->now() + rclcpp::Duration::from_seconds(FORWARD_CREEP_DURATION);
                    recovery_linear_speed_ = -LOCAL_ESCAPE_REVERSE_SPEED;
                    recovery_angular_speed_ = 0.0;
                    local_escape_spin_failures_ = 0;

                    RCLCPP_WARN(
                        this->get_logger(),
                        "local escape 도리도리 반복 감지. 짧은 후진으로 전환: rear=%.2f, proj=%.2f -> %.2f, reason=%s",
                        rear_clearance,
                        current_projection,
                        reverse_projection,
                        reason.c_str()
                    );
                    publishCandidateMarkers(local_markers);
                    return;
                }
            }
        }

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
            const bool contact_escape =
                nearestContactDistance() < CONTACT_OSCILLATION_DISTANCE * 1.25;
            const bool reverse_keeps_progress =
                !hasLeftEntrance() ||
                contact_escape ||
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
        if (found_direction) {
            local_markers.markers.push_back(makeCandidateSphere(
                "local_selected",
                10000,
                robot_x_ + 0.34 * std::cos(best_yaw),
                robot_y_ + 0.34 * std::sin(best_yaw),
                1.0,
                0.85,
                0.0,
                1.0,
                0.14,
                0.12
            ));
        }
        publishCandidateMarkers(local_markers);

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
        addDeadZone(current_goal_x_, current_goal_y_, "goal canceled");
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

                    if (isBlacklistedPoint(wx, wy, BLACKLIST_RADIUS)) {
                        continue;
                    }

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

                if (isBlacklistedPoint(wx, wy, BLACKLIST_RADIUS)) {
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
        const std::vector<int>* start_distances = nullptr;
        const std::vector<double>* start_bottleneck_clearances = nullptr;

        if (!computeReachableMetrics(
                map,
                reachable_distances,
                reachable_bottleneck_clearances)) {
            return false;
        }

        if (!getStartReachableMetrics(
                map,
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

        auto selectGoal = [&](bool relaxed, int& best_mx, int& best_my,
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
                        (*start_distances)[index] < 0) {
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

                    const double start_path_distance = (*start_distances)[index] * resolution;

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

                    if (isBlacklistedPoint(wx, wy, BLACKLIST_RADIUS)) {
                        continue;
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
        const std::vector<int>* start_distances = nullptr;
        const std::vector<double>* start_bottleneck_clearances = nullptr;

        if (!computeReachableMetricsFrom(
                map,
                robot_x_,
                robot_y_,
                false,
                reachable_distances,
                reachable_bottleneck_clearances)) {
            return false;
        }

        if (!getStartReachableMetrics(
                map,
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
        double best_unknown_exposure = 0.0;
        visualization_msgs::msg::MarkerArray branch_markers;
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.frame_id = "map";
        clear_marker.header.stamp = this->now();
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        branch_markers.markers.push_back(clear_marker);
        int branch_marker_id = 1;

        for (int my = 1; my < height - 1; ++my) {
            for (int mx = 1; mx < width - 1; ++mx) {
                const int index = my * width + mx;

                if (map->data[index] != 0 ||
                    reachable_distances[index] < 0 ||
                    (*start_distances)[index] < 0) {
                    continue;
                }

                const double path_distance = reachable_distances[index] * resolution;

                if (path_distance < BRANCH_ESCAPE_MIN_PATH_DISTANCE ||
                    path_distance > BRANCH_ESCAPE_MAX_PATH_DISTANCE) {
                    continue;
                }

                const double wx = mapToWorldX(map, mx);
                const double wy = mapToWorldY(map, my);

                if (isBlacklistedPoint(wx, wy, BLACKLIST_RADIUS)) {
                    continue;
                }

                if (isInEntranceReturnZone(wx, wy) ||
                    distanceFromStart(wx, wy) < ENTRANCE_NO_RETURN_WORLD_RADIUS) {
                    continue;
                }

                const double start_path_distance = (*start_distances)[index] * resolution;

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
                const double unknown_exposure = unknownExposureFrom(
                    map,
                    wx,
                    wy,
                    angle,
                    BRANCH_ESCAPE_UNKNOWN_LOOKAHEAD);

                double score = 0.0;
                score += 2.40 * open_ratio;
                score += 2.20 * clearance;
                score += 2.00 * bottleneck;
                score += 1.10 * path_distance;
                score += 0.70 * start_path_distance;
                score += 0.70 * projection;
                score += BRANCH_ESCAPE_UNKNOWN_WEIGHT * unknown_exposure;
                score -= 1.80 * projection_loss;
                score -= 1.15 * start_path_loss;
                score -= 0.25 * angle_diff;

                if (branch_marker_id < 180) {
                    branch_markers.markers.push_back(makeBranchMarker(
                        branch_marker_id++,
                        wx,
                        wy,
                        score,
                        unknown_exposure,
                        false));
                }

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
                    best_unknown_exposure = unknown_exposure;
                }
            }
        }

        if (!found) {
            publishBranchMarkers(branch_markers);
            return false;
        }

        branch_markers.markers.push_back(makeBranchMarker(
            10000,
            mapToWorldX(map, best_mx),
            mapToWorldY(map, best_my),
            best_score,
            best_unknown_exposure,
            true));
        publishBranchMarkers(branch_markers);

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
        pending_goal_is_branch_escape_ = true;
        pending_goal_unknown_exposure_ = best_unknown_exposure;

        RCLCPP_WARN(
            this->get_logger(),
            "포켓 branch 탈출 goal: x=%.2f, y=%.2f, score=%.2f, path=%.2f, start_path=%.2f, proj=%.2f, clearance=%.2f, bottleneck=%.2f, open=%.2f, unknown=%.2f",
            current_goal_x_,
            current_goal_y_,
            best_score,
            best_path_distance,
            best_start_path_distance,
            best_projection,
            best_clearance,
            best_bottleneck,
            best_open_ratio,
            best_unknown_exposure
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

                if (isBlacklistedPoint(wx, wy, BLACKLIST_RADIUS)) {
                    continue;
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
        const std::vector<int>* start_distances = nullptr;
        const std::vector<double>* start_bottleneck_clearances = nullptr;

        if (!computeReachableMetrics(
                map,
                reachable_distances,
                reachable_bottleneck_clearances)) {
            return false;
        }

        if (!getStartReachableMetrics(
                map,
                start_distances,
                start_bottleneck_clearances)) {
            return false;
        }

        int robot_mx = 0;
        int robot_my = 0;
        double current_start_path = current_start_path_distance_;

        if (worldToMap(map, robot_x_, robot_y_, robot_mx, robot_my)) {
            const int robot_index = robot_my * width + robot_mx;

            if ((*start_distances)[robot_index] >= 0) {
                current_start_path = (*start_distances)[robot_index] * resolution;
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

        const double current_projection = forwardProjection(robot_x_, robot_y_);

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

                // 큰 frontier는 대표 후보만 보면서 3번의 점수 구조는 유지한다.
                const int candidate_stride = std::max(
                    1,
                    static_cast<int>(
                        std::ceil(
                            static_cast<double>(cluster.size()) /
                            MAX_FRONTIER_CANDIDATES_PER_CLUSTER
                        )
                    )
                );

                for (size_t cluster_i = 0; cluster_i < cluster.size(); ++cluster_i) {
                    if (candidate_stride > 1 &&
                        cluster_i % static_cast<size_t>(candidate_stride) != 0 &&
                        cluster_i + 1 != cluster.size()) {
                        continue;
                    }

                    const int frontier_index = cluster[cluster_i];
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

                    if (reachable_distances[goal_index] < 0 ||
                        (*start_distances)[goal_index] < 0) {
                        continue;
                    }

                    const double goal_wx = origin_x + (goal_mx + 0.5) * resolution;
                    const double goal_wy = origin_y + (goal_my + 0.5) * resolution;

                    if (isInEntranceReturnZone(goal_wx, goal_wy)) {
                        continue;
                    }

                    const double current_path_distance = reachable_distances[goal_index] * resolution;
                    const double start_path_distance =
                        (*start_distances)[goal_index] * resolution;
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

                    if (isBlacklistedPoint(goal_wx, goal_wy, BLACKLIST_RADIUS)) {
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
                    const double goal_projection = forwardProjection(goal_wx, goal_wy);
                    const double projection_gain = goal_projection - current_projection;

                    // 점수는 "가까운 열린 공간"이 아니라 "시작점에서 graph상 더 깊은 frontier"를 고르게 만든다.
                    // 그래서 특정 맵의 출구 방향을 하드코딩하지 않아도, 미로를 계속 진행하는 방향을 선택한다.
                    double score = 0.0;
                    score += 5.00 * start_path_distance;
                    score += 2.00 * std::max(0.0, max_depth_gain);
                    score += 1.30 * std::max(0.0, depth_gain);
                    score += 1.70 * projection_gain;
                    score += 2.40 * path_bottleneck_clearance;
                    score += 1.20 * clearance;
                    score += 0.55 * cluster_length;
                    score += 0.80 * open_ratio;
                    score -= 0.45 * current_path_distance;
                    score -= 0.35 * angle_diff;
                    score -= (allow_backtrack ? 3.50 : 6.00) * backtrack_amount;
                    score -= 5.00 * std::max(0.0, -projection_gain);
                    score -= 7.00 * std::max(0.0, WALL_CLEARANCE_COMFORT - clearance);
                    score -= 7.00 * std::max(0.0, WALL_CLEARANCE_COMFORT - path_bottleneck_clearance);

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

        current_goal_x_ = goal_pose.pose.position.x;
        current_goal_y_ = goal_pose.pose.position.y;

        goal_pub_->publish(goal_pose);
        publishSelectedGoalMarker(current_goal_x_, current_goal_y_, "selected_goal");

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goal_pose;

        auto send_goal_options =
            rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            [this](const GoalHandleNav2::SharedPtr& goal_handle) {
                if (!goal_handle) {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Nav2가 목표를 거부했습니다. 해당 goal만 잠깐 막고 재탐색합니다"
                    );

                    consecutive_goal_failures_++;
                    addBlacklistPoint(current_goal_x_, current_goal_y_, BLACKLIST_TTL_FAILURE);
                    addDeadZone(current_goal_x_, current_goal_y_, "Nav2 rejected goal");
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
        contact_oscillation_active_ = false;
        local_escape_spin_failures_ = 0;
        goal_start_x_ = robot_x_;
        goal_start_y_ = robot_y_;
        goal_start_projection_ = forwardProjection(robot_x_, robot_y_);
        goal_start_path_distance_ = current_start_path_distance_;
        active_goal_projection_ = forwardProjection(current_goal_x_, current_goal_y_);
        active_goal_start_distance_ = distanceFromStart(current_goal_x_, current_goal_y_);
        active_goal_start_path_distance_ = current_start_path_distance_;
        active_goal_is_branch_escape_ = pending_goal_is_branch_escape_;
        active_goal_unknown_exposure_ = pending_goal_unknown_exposure_;

        if (latest_map_) {
            const std::vector<int>* start_distances = nullptr;
            const std::vector<double>* start_bottleneck_clearances = nullptr;
            int goal_mx = 0;
            int goal_my = 0;

            if (getStartReachableMetrics(
                    latest_map_,
                    start_distances,
                    start_bottleneck_clearances) &&
                worldToMap(latest_map_, current_goal_x_, current_goal_y_, goal_mx, goal_my)) {
                const int width = static_cast<int>(latest_map_->info.width);
                const int goal_index = goal_my * width + goal_mx;

                if ((*start_distances)[goal_index] >= 0) {
                    active_goal_start_path_distance_ =
                        (*start_distances)[goal_index] * latest_map_->info.resolution;
                }
            }
        }

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

        processPendingContactRecovery();
        if (pending_contact_recovery_ || escape_active_) {
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

        if (isBacktrackingTowardEntrance()) {
            RCLCPP_WARN(
                this->get_logger(),
                "입구 복귀 방향 이동 감지. 현재 목표 취소: 시작거리=%.2f, 최대시작거리=%.2f, 현재투영=%.2f, 최대투영=%.2f, 현재경로거리=%.2f, 최대경로거리=%.2f, goal경로거리=%.2f",
                start_distance,
                max_start_distance_,
                projection,
                max_forward_projection_,
                current_start_path_distance_,
                max_start_path_distance_,
                active_goal_start_path_distance_
            );

            cancelCurrentGoalAndBlacklist();
            return;
        }

        const double goal_distance =
            std::hypot(current_goal_x_ - robot_x_, current_goal_y_ - robot_y_);

        if (goal_distance + STUCK_GOAL_IMPROVEMENT < best_goal_distance_) {
            best_goal_distance_ = goal_distance;
            last_progress_time_ = this->now();
            contact_oscillation_active_ = false;
            return;
        }

        const double contact_distance = nearestContactDistance();
        const bool near_contact =
            contact_distance < CONTACT_OSCILLATION_DISTANCE;

        if (near_contact) {
            if (!contact_oscillation_active_) {
                contact_oscillation_active_ = true;
                contact_oscillation_since_ = this->now();
            } else if ((this->now() - contact_oscillation_since_).seconds() >
                       CONTACT_OSCILLATION_TIMEOUT) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "접촉 후 제자리 회전 정체 감지. 현재 목표 취소: contact=%.3f, goal_distance=%.2f",
                    contact_distance,
                    goal_distance
                );

                contact_oscillation_active_ = false;
                requestContactRecovery("contact stall while following goal");
                return;
            }
        } else {
            contact_oscillation_active_ = false;
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
        bool should_contact_reverse_after_result = false;
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
                addDeadZone(current_goal_x_, current_goal_y_, "succeeded without progress");
                should_escape_after_result = canUseBackupRecovery();
                should_contact_reverse_after_result =
                    nearestContactDistance() < CONTACT_OSCILLATION_DISTANCE;
                prefer_local_motion_after_result = true;
                continue_reason = "succeeded with almost no movement";
            } else if (active_goal_is_branch_escape_ &&
                       active_goal_unknown_exposure_ < 0.25) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "branch goal 도착했지만 미탐색 확장이 거의 없습니다. 포켓 내부 성공으로 보고 blacklist합니다: x=%.2f, y=%.2f, moved=%.2f, projection_gain=%.2f, path_gain=%.2f, unknown=%.2f",
                    current_goal_x_,
                    current_goal_y_,
                    moved_distance,
                    projection_gain,
                    path_gain,
                    active_goal_unknown_exposure_
                );

                consecutive_goal_failures_++;
                addBlacklistPoint(current_goal_x_, current_goal_y_, BLACKLIST_TTL_FAILURE);
                addBlacklistPoint(robot_x_, robot_y_, BLACKLIST_TTL_FAILURE);
                addDeadZone(current_goal_x_, current_goal_y_, "branch success without unknown expansion");
                addDeadZone(robot_x_, robot_y_, "robot pose inside rejected pocket");
                should_escape_after_result = canUseBackupRecovery();
                should_contact_reverse_after_result =
                    nearestContactDistance() < CONTACT_OSCILLATION_DISTANCE;
                prefer_local_motion_after_result = true;
                continue_reason = "branch success without unknown expansion";
            } else {
                RCLCPP_INFO(
                    this->get_logger(),
                    "목표 도착: x=%.2f, y=%.2f, moved=%.2f, projection_gain=%.2f, path_gain=%.2f, branch=%d, unknown=%.2f",
                    current_goal_x_,
                    current_goal_y_,
                    moved_distance,
                    projection_gain,
                    path_gain,
                    active_goal_is_branch_escape_ ? 1 : 0,
                    active_goal_unknown_exposure_
                );
                consecutive_goal_failures_ = 0;
                addBlacklistPoint(current_goal_x_, current_goal_y_, BLACKLIST_TTL_SUCCESS);
                continue_reason = "goal succeeded";
            }
        } else if (canceling_goal_) {
            RCLCPP_WARN(this->get_logger(), "정체된 목표 취소 완료");
            consecutive_goal_failures_++;
            should_escape_after_result = canUseBackupRecovery();
            should_contact_reverse_after_result =
                nearestContactDistance() < CONTACT_OSCILLATION_DISTANCE;
            prefer_local_motion_after_result = true;
            continue_reason = "stuck goal canceled";
        } else {
            RCLCPP_WARN(
                this->get_logger(),
                "Nav2 목표 실패. 같은 지점을 blacklist에 넣고 재탐색합니다: x=%.2f, y=%.2f",
                current_goal_x_,
                current_goal_y_
            );

            consecutive_goal_failures_++;
            // 실패한 goal은 relaxed frontier에서도 제외해 좌우 dead-end 왕복을 줄인다.
            addBlacklistPoint(current_goal_x_, current_goal_y_, BLACKLIST_TTL_FAILURE);
            addDeadZone(current_goal_x_, current_goal_y_, "Nav2 goal failed");
            should_escape_after_result = canUseBackupRecovery();
            should_contact_reverse_after_result =
                nearestContactDistance() < CONTACT_OSCILLATION_DISTANCE;
            prefer_local_motion_after_result = true;
            continue_reason = "Nav2 goal failed";
        }

        active_goal_handle_.reset();
        canceling_goal_ = false;
        goal_in_progress_ = false;

        processPendingContactRecovery();
        if (escape_active_) {
            return;
        }

        if (should_contact_reverse_after_result) {
            startContactReverseRecovery(continue_reason);
            if (escape_active_) {
                return;
            }
        }

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
