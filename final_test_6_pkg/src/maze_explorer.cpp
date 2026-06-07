#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using std::placeholders::_1;

class MazeExplorer : public rclcpp::Node
{
public:
  MazeExplorer() : Node("maze_explorer")
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");
    input_direction_topic_ = declare_parameter<std::string>(
      "input_direction_topic",
      "/maze_explorer/direction_input");
    nav_goal_viz_topic_ = declare_parameter<std::string>("nav_goal_viz_topic", "/maze_explorer/goal_pose");
    direction_viz_topic_ = declare_parameter<std::string>("direction_viz_topic", "/maze_explorer/direction_pose");
    nav_cmd_topic_ = declare_parameter<std::string>("nav_cmd_topic", "/cmd_vel_nav");

    lookahead_distance_ = declare_parameter<double>("lookahead_distance", 0.85);
    max_goal_distance_ = declare_parameter<double>("max_goal_distance", 1.20);
    min_goal_distance_ = declare_parameter<double>("min_goal_distance", 0.28);
    goal_standoff_ = declare_parameter<double>("goal_standoff", 0.08);
    path_clearance_ = declare_parameter<double>("path_clearance", 0.085);
    goal_clearance_ = declare_parameter<double>("goal_clearance", 0.105);
    scan_standoff_ = declare_parameter<double>("scan_standoff", 0.13);
    scan_half_width_ = declare_parameter<double>("scan_half_width", 0.18);
    scan_timeout_ = declare_parameter<double>("scan_timeout", 0.60);
    stuck_timeout_ = declare_parameter<double>("stuck_timeout", 6.0);
    progress_improvement_ = declare_parameter<double>("progress_improvement", 0.03);
    max_consecutive_failures_ = declare_parameter<int>("max_consecutive_failures", 3);
    laser_yaw_offset_ = declare_parameter<double>("laser_yaw_offset", 3.14159);
    direct_linear_speed_ = declare_parameter<double>("direct_linear_speed", 0.065);
    direct_turn_gain_ = declare_parameter<double>("direct_turn_gain", 1.25);
    direct_max_angular_ = declare_parameter<double>("direct_max_angular", 0.65);
    direct_align_yaw_ = declare_parameter<double>("direct_align_yaw", 0.45);
    direct_stop_distance_ = declare_parameter<double>("direct_stop_distance", 0.16);
    direct_slow_distance_ = declare_parameter<double>("direct_slow_distance", 0.34);
    goal_reached_tolerance_ = declare_parameter<double>("goal_reached_tolerance", 0.20);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&MazeExplorer::mapCallback, this, _1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(), std::bind(&MazeExplorer::scanCallback, this, _1));

    direction_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      input_direction_topic_, 10, std::bind(&MazeExplorer::directionCallback, this, _1));

    nav_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(nav_goal_viz_topic_, 10);
    direction_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(direction_viz_topic_, 10);
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(nav_cmd_topic_, 10);

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(300),
      std::bind(&MazeExplorer::controlLoop, this));

    watchdog_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MazeExplorer::watchdogCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "final_test_6 goal follower loaded. RViz 2D Goal Pose on %s is treated as a target pose.",
      input_direction_topic_.c_str());
  }

private:
  std::string map_frame_;
  std::string robot_frame_;
  std::string input_direction_topic_;
  std::string nav_goal_viz_topic_;
  std::string direction_viz_topic_;
  std::string nav_cmd_topic_;

  double lookahead_distance_;
  double max_goal_distance_;
  double min_goal_distance_;
  double goal_standoff_;
  double path_clearance_;
  double goal_clearance_;
  double scan_standoff_;
  double scan_half_width_;
  double scan_timeout_;
  double stuck_timeout_;
  double progress_improvement_;
  double laser_yaw_offset_;
  double direct_linear_speed_;
  double direct_turn_gain_;
  double direct_max_angular_;
  double direct_align_yaw_;
  double direct_stop_distance_;
  double direct_slow_distance_;
  double goal_reached_tolerance_;
  int max_consecutive_failures_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr direction_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr direction_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  rclcpp::Time last_scan_time_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  GoalHandleNav2::SharedPtr active_goal_handle_;

  bool direction_active_ = false;
  bool goal_in_progress_ = false;
  bool canceling_goal_ = false;
  double target_x_ = 0.0;
  double target_y_ = 0.0;
  double target_yaw_ = 0.0;
  double direction_yaw_ = 0.0;
  double robot_x_ = 0.0;
  double robot_y_ = 0.0;
  double robot_yaw_ = 0.0;
  double current_goal_x_ = 0.0;
  double current_goal_y_ = 0.0;
  double best_goal_distance_ = std::numeric_limits<double>::max();
  int consecutive_failures_ = 0;
  uint64_t direction_sequence_ = 0;
  uint64_t active_goal_sequence_ = 0;
  rclcpp::Time last_progress_time_;

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    latest_map_ = msg;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    latest_scan_ = msg;
    last_scan_time_ = now();
  }

  void directionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!updateRobotPose()) {
      return;
    }

    target_x_ = msg->pose.position.x;
    target_y_ = msg->pose.position.y;
    target_yaw_ = yawFromPose(*msg);
    direction_yaw_ = targetBearing();
    direction_active_ = true;
    consecutive_failures_ = 0;
    direction_sequence_++;

    publishZero();
    publishSelectedDirection();

    RCLCPP_INFO(
      get_logger(),
      "새 목표 수신: target=(%.2f, %.2f), final_yaw=%.2f. Nav2가 가능하면 목표점까지 회피 주행합니다.",
      target_x_,
      target_y_,
      target_yaw_);

    if (goal_in_progress_) {
      cancelCurrentGoal("new direction");
      return;
    }

    trySendDirectionalGoal();
  }

  void controlLoop()
  {
    if (!direction_active_) {
      return;
    }

    if (!updateRobotPose()) {
      return;
    }

    if (distanceToTarget() <= goal_reached_tolerance_) {
      direction_active_ = false;
      publishZero();
      RCLCPP_INFO(get_logger(), "클릭 목표 근처에 도착했습니다.");
      return;
    }

    direction_yaw_ = targetBearing();
    publishSelectedDirection();

    if (goal_in_progress_ || canceling_goal_) {
      return;
    }

    trySendDirectionalGoal();
  }

  bool updateRobotPose()
  {
    geometry_msgs::msg::TransformStamped transform;

    try {
      transform = tf_buffer_->lookupTransform(map_frame_, robot_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "로봇 pose TF 대기 중: %s -> %s (%s)",
        map_frame_.c_str(), robot_frame_.c_str(), ex.what());
      return false;
    }

    robot_x_ = transform.transform.translation.x;
    robot_y_ = transform.transform.translation.y;
    robot_yaw_ = yawFromQuaternion(transform.transform.rotation);
    return true;
  }

  bool trySendDirectionalGoal()
  {
    if (!direction_active_ || goal_in_progress_ || canceling_goal_) {
      return false;
    }

    if (!updateRobotPose()) {
      return false;
    }

    direction_yaw_ = targetBearing();
    geometry_msgs::msg::PoseStamped goal;
    double chosen_distance = 0.0;
    double map_clear = 0.0;
    double scan_clear = 0.0;

    if (!latest_map_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "/map 대기 중입니다. scan 기반 직접 전진으로 방향을 유지합니다.");
      return publishDirectDriveCommand("map waiting", map_clear, scan_clear);
    }

    if (consecutive_failures_ >= max_consecutive_failures_ ||
        !makeDirectionalGoal(goal, chosen_distance, map_clear, scan_clear)) {
      return publishDirectDriveCommand("outside mapped free space", map_clear, scan_clear);
    }

    sendGoal(goal);

    RCLCPP_INFO(
      get_logger(),
      "클릭 목표 Nav2 전송: x=%.2f, y=%.2f, final_yaw=%.2f, distance=%.2f, target_clear=%.2f, scan_clear=%.2f",
      goal.pose.position.x,
      goal.pose.position.y,
      target_yaw_,
      chosen_distance,
      map_clear,
      scan_clear);

    return true;
  }

  bool publishDirectDriveCommand(
    const std::string& reason,
    double map_clear,
    double scan_clear)
  {
    (void)map_clear;
    scan_clear = scanClearanceInDirection(direction_yaw_);

    if (!hasFreshScan()) {
      publishZero();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "scan이 없어서 직접 전진을 보류합니다. reason=%s",
        reason.c_str());
      return false;
    }

    if (std::isfinite(scan_clear) && scan_clear < direct_stop_distance_) {
      publishZero();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "선택 방향 전방이 막혀 정지합니다. scan_clear=%.2f, reason=%s",
        scan_clear, reason.c_str());
      return false;
    }

    const double yaw_error = normalizeAngle(direction_yaw_ - robot_yaw_);
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::clamp(
      direct_turn_gain_ * yaw_error,
      -direct_max_angular_,
      direct_max_angular_);

    if (std::abs(yaw_error) < direct_align_yaw_) {
      cmd.linear.x = direct_linear_speed_;

      if (std::isfinite(scan_clear) && scan_clear < direct_slow_distance_) {
        const double window = std::max(0.01, direct_slow_distance_ - direct_stop_distance_);
        const double scale = std::clamp(
          (scan_clear - direct_stop_distance_) / window,
          0.25,
          1.0);
        cmd.linear.x *= scale;
      }
    }

    cmd_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Nav2가 아직 못 가는 목표 방향 scan 전진: linear=%.3f, angular=%.3f, yaw_error=%.2f, target_dist=%.2f, scan_clear=%.2f, reason=%s",
      cmd.linear.x,
      cmd.angular.z,
      yaw_error,
      distanceToTarget(),
      scan_clear,
      reason.c_str());

    return true;
  }

  bool makeDirectionalGoal(
    geometry_msgs::msg::PoseStamped& goal,
    double& chosen_distance,
    double& map_clear,
    double& scan_clear)
  {
    map_clear = rayClearDistance(direction_yaw_, max_goal_distance_, path_clearance_);
    scan_clear = scanClearanceInDirection(direction_yaw_);
    chosen_distance = distanceToTarget();

    if (chosen_distance <= goal_reached_tolerance_) {
      direction_active_ = false;
      publishZero();
      RCLCPP_INFO(get_logger(), "클릭 목표 근처에 도착했습니다.");
      return false;
    }

    int mx = 0;
    int my = 0;
    if (!worldToMap(target_x_, target_y_, mx, my)) {
      return false;
    }

    const int index = my * static_cast<int>(latest_map_->info.width) + mx;
    if (latest_map_->data[index] >= 50) {
      return false;
    }

    if (nearestObstacleDistance(mx, my, goal_clearance_) < goal_clearance_) {
      return false;
    }

    goal.header.frame_id = map_frame_;
    goal.header.stamp = now();
    goal.pose.position.x = target_x_;
    goal.pose.position.y = target_y_;
    goal.pose.position.z = 0.0;
    setYaw(goal, target_yaw_);
    return true;
  }

  void sendGoal(const geometry_msgs::msg::PoseStamped& goal_pose)
  {
    if (!nav_client_->wait_for_action_server(std::chrono::milliseconds(200))) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Nav2 action server 대기 중입니다.");
      return;
    }

    current_goal_x_ = goal_pose.pose.position.x;
    current_goal_y_ = goal_pose.pose.position.y;
    best_goal_distance_ = std::hypot(current_goal_x_ - robot_x_, current_goal_y_ - robot_y_);
    last_progress_time_ = now();
    active_goal_sequence_ = direction_sequence_;

    nav_goal_pub_->publish(goal_pose);

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goal_pose;

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    const uint64_t goal_sequence = active_goal_sequence_;

    options.goal_response_callback =
      [this, goal_sequence](const GoalHandleNav2::SharedPtr& goal_handle) {
        if (goal_sequence != active_goal_sequence_ || goal_sequence != direction_sequence_) {
          if (goal_handle) {
            nav_client_->async_cancel_goal(goal_handle);
          } else if (goal_sequence == active_goal_sequence_) {
            goal_in_progress_ = false;
            canceling_goal_ = false;
            active_goal_handle_.reset();
            if (direction_active_) {
              trySendDirectionalGoal();
            }
          }
          return;
        }

        if (!goal_handle) {
          RCLCPP_WARN(get_logger(), "Nav2가 방향 추종 goal을 거부했습니다.");
          goal_in_progress_ = false;
          canceling_goal_ = false;
          consecutive_failures_++;
          if (direction_active_) {
            trySendDirectionalGoal();
          }
          return;
        }

        active_goal_handle_ = goal_handle;
      };

    options.result_callback =
      [this, goal_sequence](const GoalHandleNav2::WrappedResult& result) {
        resultCallback(goal_sequence, result);
      };

    goal_in_progress_ = true;
    canceling_goal_ = false;
    active_goal_handle_.reset();
    nav_client_->async_send_goal(goal_msg, options);
  }

  void resultCallback(uint64_t goal_sequence, const GoalHandleNav2::WrappedResult& result)
  {
    const bool is_current_goal = goal_sequence == active_goal_sequence_;
    const bool was_canceling = canceling_goal_;

    if (is_current_goal) {
      goal_in_progress_ = false;
      canceling_goal_ = false;
      active_goal_handle_.reset();
    }

    if (goal_sequence != direction_sequence_) {
      if (is_current_goal && direction_active_) {
        trySendDirectionalGoal();
      }
      return;
    }

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      consecutive_failures_ = 0;
      direction_active_ = false;
      publishZero();
      RCLCPP_INFO(get_logger(), "클릭 목표 도착: target=(%.2f, %.2f)", target_x_, target_y_);
      return;
    } else if (was_canceling) {
      RCLCPP_INFO(get_logger(), "현재 Nav2 goal 취소 완료. 목표를 다시 평가합니다.");
    } else {
      consecutive_failures_++;
      RCLCPP_WARN(
        get_logger(),
        "Nav2 목표 실패. 맵/unknown 상황을 다시 보고 보조 전진을 시도합니다: failures=%d/%d",
        consecutive_failures_, max_consecutive_failures_);
    }

    if (direction_active_) {
      trySendDirectionalGoal();
    }
  }

  void watchdogCallback()
  {
    if (!goal_in_progress_ || canceling_goal_) {
      return;
    }

    if (!updateRobotPose()) {
      return;
    }

    const double goal_distance = std::hypot(current_goal_x_ - robot_x_, current_goal_y_ - robot_y_);
    if (goal_distance + progress_improvement_ < best_goal_distance_) {
      best_goal_distance_ = goal_distance;
      last_progress_time_ = now();
      return;
    }

    if ((now() - last_progress_time_).seconds() < stuck_timeout_) {
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "방향 추종 goal 접근 정체. 현재 step을 취소하고 짧은 step으로 재시도합니다. 남은거리=%.2f",
      goal_distance);

    consecutive_failures_++;
    cancelCurrentGoal("stuck directional step");
  }

  void cancelCurrentGoal(const std::string& reason)
  {
    publishZero();
    canceling_goal_ = true;

    if (active_goal_handle_) {
      RCLCPP_WARN(get_logger(), "현재 방향 추종 goal 취소: %s", reason.c_str());
      nav_client_->async_cancel_goal(active_goal_handle_);
      return;
    }

    if (!goal_in_progress_) {
      canceling_goal_ = false;
    }
  }

  void publishZero()
  {
    geometry_msgs::msg::Twist stop;
    for (int i = 0; i < 3; ++i) {
      cmd_pub_->publish(stop);
    }
  }

  void publishSelectedDirection()
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = map_frame_;
    pose.header.stamp = now();

    if (updateRobotPose()) {
      pose.pose.position.x = robot_x_;
      pose.pose.position.y = robot_y_;
    }

    setYaw(pose, direction_yaw_);
    direction_pub_->publish(pose);
  }

  double distanceToTarget() const
  {
    return std::hypot(target_x_ - robot_x_, target_y_ - robot_y_);
  }

  double targetBearing() const
  {
    if (distanceToTarget() < 1e-4) {
      return target_yaw_;
    }

    return std::atan2(target_y_ - robot_y_, target_x_ - robot_x_);
  }

  bool worldToMap(double wx, double wy, int& mx, int& my) const
  {
    if (!latest_map_) {
      return false;
    }

    mx = static_cast<int>((wx - latest_map_->info.origin.position.x) / latest_map_->info.resolution);
    my = static_cast<int>((wy - latest_map_->info.origin.position.y) / latest_map_->info.resolution);

    return mx >= 0 &&
           mx < static_cast<int>(latest_map_->info.width) &&
           my >= 0 &&
           my < static_cast<int>(latest_map_->info.height);
  }

  bool hasObstacleWithin(int mx, int my, double clearance) const
  {
    const int width = static_cast<int>(latest_map_->info.width);
    const int height = static_cast<int>(latest_map_->info.height);
    const int radius_cells = std::max(
      1,
      static_cast<int>(std::ceil(clearance / latest_map_->info.resolution)));

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

        if (latest_map_->data[cy * width + cx] >= 50) {
          return true;
        }
      }
    }

    return false;
  }

  double nearestObstacleDistance(int mx, int my, double max_distance) const
  {
    const int width = static_cast<int>(latest_map_->info.width);
    const int height = static_cast<int>(latest_map_->info.height);
    const int radius_cells = std::max(
      1,
      static_cast<int>(std::ceil(max_distance / latest_map_->info.resolution)));

    double best = max_distance;

    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }

        const double distance = std::hypot(
          dx * latest_map_->info.resolution,
          dy * latest_map_->info.resolution);

        if (distance > best) {
          continue;
        }

        const int cx = mx + dx;
        const int cy = my + dy;

        if (cx < 0 || cx >= width || cy < 0 || cy >= height) {
          best = distance;
          continue;
        }

        if (latest_map_->data[cy * width + cx] >= 50) {
          best = distance;
        }
      }
    }

    return best;
  }

  double rayClearDistance(double yaw, double max_distance, double clearance) const
  {
    if (!latest_map_) {
      return 0.0;
    }

    const double step = std::max(0.03, static_cast<double>(latest_map_->info.resolution));
    double last_safe = 0.0;

    for (double distance = step; distance <= max_distance; distance += step) {
      const double wx = robot_x_ + distance * std::cos(yaw);
      const double wy = robot_y_ + distance * std::sin(yaw);

      int mx = 0;
      int my = 0;
      if (!worldToMap(wx, wy, mx, my)) {
        break;
      }

      const int index = my * static_cast<int>(latest_map_->info.width) + mx;
      if (latest_map_->data[index] >= 50 || hasObstacleWithin(mx, my, clearance)) {
        break;
      }

      last_safe = distance;
    }

    return last_safe;
  }

  double scanClearanceInDirection(double map_yaw) const
  {
    if (!hasFreshScan()) {
      return std::numeric_limits<double>::infinity();
    }

    const double base_link_yaw = normalizeAngle(map_yaw - robot_yaw_);
    const double scan_center = normalizeAngle(base_link_yaw + laser_yaw_offset_);
    double best = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < latest_scan_->ranges.size(); ++i) {
      const double angle = normalizeAngle(
        latest_scan_->angle_min +
        static_cast<double>(i) * latest_scan_->angle_increment);

      if (std::abs(normalizeAngle(angle - scan_center)) > scan_half_width_) {
        continue;
      }

      const double range = latest_scan_->ranges[i];
      if (std::isfinite(range) &&
          range >= std::max(0.04f, latest_scan_->range_min) &&
          range <= latest_scan_->range_max) {
        best = std::min(best, range);
      }
    }

    return best;
  }

  bool hasFreshScan() const
  {
    return latest_scan_ &&
           !latest_scan_->ranges.empty() &&
           latest_scan_->angle_increment > 0.0 &&
           (now() - last_scan_time_).seconds() <= scan_timeout_;
  }

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  static double yawFromPose(const geometry_msgs::msg::PoseStamped& pose)
  {
    return yawFromQuaternion(pose.pose.orientation);
  }

  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
  {
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  static void setYaw(geometry_msgs::msg::PoseStamped& pose, double yaw)
  {
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    pose.pose.orientation.w = std::cos(yaw * 0.5);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MazeExplorer>());
  rclcpp::shutdown();
  return 0;
}
