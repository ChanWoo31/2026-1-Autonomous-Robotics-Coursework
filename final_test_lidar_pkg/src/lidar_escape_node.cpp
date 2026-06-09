#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

double degToRad(double degrees)
{
  return degrees * kPi / 180.0;
}

double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double angleDistance(double a, double b)
{
  return std::abs(normalizeAngle(a - b));
}

double distance2d(double ax, double ay, double bx, double by)
{
  const double dx = ax - bx;
  const double dy = ay - by;
  return std::sqrt(dx * dx + dy * dy);
}
}  // namespace

class LidarEscapeNode : public rclcpp::Node
{
public:
  LidarEscapeNode()
  : Node("lidar_escape_node")
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

    front_angle_offset_ = declare_parameter<double>("front_angle_offset", 0.0);
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);
    scan_timeout_sec_ = declare_parameter<double>("scan_timeout_sec", 0.8);

    max_linear_speed_ = declare_parameter<double>("max_linear_speed", 0.20);
    min_linear_speed_ = declare_parameter<double>("min_linear_speed", 0.045);
    backtrack_linear_speed_ = declare_parameter<double>("backtrack_linear_speed", 0.10);
    backup_speed_ = declare_parameter<double>("backup_speed", 0.075);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 1.35);
    steering_gain_ = declare_parameter<double>("steering_gain", 1.55);
    wall_balance_gain_ = declare_parameter<double>("wall_balance_gain", 0.22);

    hard_stop_distance_ = declare_parameter<double>("hard_stop_distance", 0.145);
    stop_distance_ = declare_parameter<double>("stop_distance", 0.19);
    slow_distance_ = declare_parameter<double>("slow_distance", 0.62);
    side_stop_distance_ = declare_parameter<double>("side_stop_distance", 0.135);
    rear_stop_distance_ = declare_parameter<double>("rear_stop_distance", 0.16);

    search_half_fov_ = degToRad(declare_parameter<double>("search_fov_deg", 155.0) * 0.5);
    branch_half_fov_ = degToRad(declare_parameter<double>("branch_fov_deg", 180.0) * 0.5);
    open_ray_min_depth_ = declare_parameter<double>("open_ray_min_depth", 0.46);
    branch_min_depth_ = declare_parameter<double>("branch_min_depth", 0.85);
    dead_end_depth_ = declare_parameter<double>("dead_end_depth", 0.52);
    ray_score_max_range_ = declare_parameter<double>("ray_score_max_range", 3.5);
    min_segment_width_ = degToRad(declare_parameter<double>("min_segment_width_deg", 8.0));
    branch_min_segment_width_ =
      degToRad(declare_parameter<double>("branch_min_segment_width_deg", 12.0));
    segment_gap_ = degToRad(declare_parameter<double>("segment_gap_deg", 4.0));

    center_preference_weight_ = declare_parameter<double>("center_preference_weight", 0.90);
    depth_weight_ = declare_parameter<double>("depth_weight", 1.00);
    width_weight_ = declare_parameter<double>("width_weight", 0.35);
    turn_penalty_weight_ = declare_parameter<double>("turn_penalty_weight", 0.30);
    untried_branch_bonus_ = declare_parameter<double>("untried_branch_bonus", 1.20);
    tried_branch_penalty_ = declare_parameter<double>("tried_branch_penalty", 0.85);
    blocked_branch_penalty_ = declare_parameter<double>("blocked_branch_penalty", 2.20);

    branch_record_radius_ = declare_parameter<double>("branch_record_radius", 0.42);
    branch_reached_radius_ = declare_parameter<double>("branch_reached_radius", 0.30);
    branch_option_match_ = degToRad(declare_parameter<double>("branch_option_match_deg", 28.0));
    branch_option_separation_ =
      degToRad(declare_parameter<double>("branch_option_separation_deg", 32.0));

    breadcrumb_spacing_ = declare_parameter<double>("breadcrumb_spacing", 0.16);
    breadcrumb_reached_radius_ = declare_parameter<double>("breadcrumb_reached_radius", 0.22);
    backup_distance_ = declare_parameter<double>("backup_distance", 0.32);
    backup_timeout_sec_ = declare_parameter<double>("backup_timeout_sec", 3.0);
    align_yaw_tolerance_ = degToRad(declare_parameter<double>("align_yaw_tolerance_deg", 10.0));

    escaped_clearance_ = declare_parameter<double>("escaped_clearance", 2.2);
    escaped_hold_sec_ = declare_parameter<double>("escaped_hold_sec", 1.5);
    stop_when_escaped_ = declare_parameter<bool>("stop_when_escaped", false);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarEscapeNode::scanCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20,
      std::bind(&LidarEscapeNode::odomCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_)));
    control_timer_ = create_wall_timer(
      std::max(std::chrono::milliseconds(10), period),
      std::bind(&LidarEscapeNode::controlLoop, this));

    mode_enter_time_ = now();
    escaped_since_ = now();

    RCLCPP_INFO(
      get_logger(),
      "lidar_escape_node ready: scan=%s odom=%s cmd_vel=%s front_offset=%.3f",
      scan_topic_.c_str(), odom_topic_.c_str(), cmd_vel_topic_.c_str(), front_angle_offset_);
  }

private:
  enum class Mode
  {
    Explore,
    Backup,
    Backtrack,
    AlignBranch,
    Escaped
  };

  struct Pose2D
  {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
  };

  struct RaySegment
  {
    double left = 0.0;
    double right = 0.0;
    double center = 0.0;
    double mean_range = 0.0;
    double max_range = 0.0;
    double width = 0.0;
    int count = 0;
    double score = 0.0;
  };

  struct BranchOption
  {
    double yaw = 0.0;
    bool tried = false;
    bool blocked = false;
  };

  struct BranchPoint
  {
    Pose2D pose;
    std::vector<BranchOption> options;
    std::size_t breadcrumb_index = 0;
    rclcpp::Time last_seen;
  };

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    latest_scan_ = msg;
    last_scan_time_ = now();
    have_scan_ = true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    pose_.x = msg->pose.pose.position.x;
    pose_.y = msg->pose.pose.position.y;

    const auto & q = msg->pose.pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    pose_.yaw = std::atan2(siny_cosp, cosy_cosp);
    have_odom_ = true;
  }

  void controlLoop()
  {
    geometry_msgs::msg::Twist cmd;

    if (!readyForControl()) {
      cmd_pub_->publish(cmd);
      return;
    }

    if (mode_ == Mode::Explore) {
      updateBreadcrumb();
    }

    if (stop_when_escaped_ && updateEscapedState()) {
      mode_ = Mode::Escaped;
    }

    switch (mode_) {
      case Mode::Explore:
        cmd = makeExploreCommand();
        break;
      case Mode::Backup:
        cmd = makeBackupCommand();
        break;
      case Mode::Backtrack:
        cmd = makeBacktrackCommand();
        break;
      case Mode::AlignBranch:
        cmd = makeAlignBranchCommand();
        break;
      case Mode::Escaped:
        cmd = geometry_msgs::msg::Twist();
        break;
    }

    applyFinalSafety(cmd);
    cmd_pub_->publish(cmd);
  }

  bool readyForControl()
  {
    if (!have_scan_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for LiDAR scan.");
      return false;
    }
    if (!have_odom_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for OpenCR odometry.");
      return false;
    }
    if ((now() - last_scan_time_).seconds() > scan_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "LiDAR scan timed out; stopping.");
      return false;
    }
    return true;
  }

  geometry_msgs::msg::Twist makeExploreCommand()
  {
    auto branch_segments =
      findOpenSegments(branch_half_fov_, branch_min_depth_, branch_min_segment_width_);
    updateBranchMemory(branch_segments);

    const auto dead_end_segments =
      findOpenSegments(kPi * 0.5, dead_end_depth_, min_segment_width_);
    if (dead_end_segments.empty() && sectorMax(-kPi * 0.5, kPi * 0.5) < dead_end_depth_) {
      startDeadEndRecovery();
      return makeBackupCommand();
    }

    auto segments = findOpenSegments(search_half_fov_, open_ray_min_depth_, min_segment_width_);
    if (segments.empty()) {
      const double best_angle = bestRayAngle(-search_half_fov_, search_half_fov_);
      geometry_msgs::msg::Twist cmd;
      cmd.angular.z = clamp(steering_gain_ * best_angle, -max_angular_speed_, max_angular_speed_);
      return cmd;
    }

    const int nearby_branch = findNearbyBranch(branch_record_radius_);
    const int best_index = selectBestSegment(segments, nearby_branch);
    const RaySegment & selected = segments[static_cast<std::size_t>(best_index)];
    const double selected_yaw = normalizeAngle(pose_.yaw + selected.center);

    maybeCommitBranchChoice(nearby_branch, selected_yaw);

    geometry_msgs::msg::Twist cmd;
    const double front_min = sectorMin(degToRad(-14.0), degToRad(14.0));
    const double front_wide_min = sectorMin(degToRad(-30.0), degToRad(30.0));
    const double speed_scale =
      clamp((front_wide_min - stop_distance_) / std::max(0.01, slow_distance_ - stop_distance_), 0.0, 1.0);
    const double turn_scale = clamp(std::cos(std::abs(selected.center)), 0.18, 1.0);

    if (std::abs(selected.center) < degToRad(70.0) && front_min > stop_distance_) {
      cmd.linear.x = clamp(max_linear_speed_ * speed_scale * turn_scale, 0.0, max_linear_speed_);
      if (cmd.linear.x > 0.0) {
        cmd.linear.x = std::max(min_linear_speed_, cmd.linear.x);
      }
    }

    const double wall_balance = computeWallBalance();
    cmd.angular.z = steering_gain_ * selected.center + wall_balance;

    const double left_close = sectorMin(degToRad(35.0), degToRad(95.0));
    const double right_close = sectorMin(degToRad(-95.0), degToRad(-35.0));
    if (left_close < side_stop_distance_) {
      cmd.angular.z -= 0.35;
      cmd.linear.x *= 0.65;
    }
    if (right_close < side_stop_distance_) {
      cmd.angular.z += 0.35;
      cmd.linear.x *= 0.65;
    }

    cmd.angular.z = clamp(cmd.angular.z, -max_angular_speed_, max_angular_speed_);
    return cmd;
  }

  geometry_msgs::msg::Twist makeBackupCommand()
  {
    geometry_msgs::msg::Twist cmd;
    const double backed_distance =
      distance2d(pose_.x, pose_.y, backup_start_pose_.x, backup_start_pose_.y);
    const double elapsed = (now() - backup_start_time_).seconds();
    const double rear_min = sectorMin(degToRad(165.0), degToRad(-165.0));

    if (backed_distance >= backup_distance_ || elapsed > backup_timeout_sec_ ||
      rear_min < rear_stop_distance_)
    {
      if (target_branch_id_ >= 0) {
        startBacktrackToBranch(target_branch_id_);
        return makeBacktrackCommand();
      }
      changeMode(Mode::Explore);
      return makeExploreCommand();
    }

    if (rear_min > rear_stop_distance_) {
      cmd.linear.x = -backup_speed_;
    }
    cmd.angular.z = clamp(backup_turn_sign_ * max_angular_speed_ * 0.35, -max_angular_speed_, max_angular_speed_);
    return cmd;
  }

  geometry_msgs::msg::Twist makeBacktrackCommand()
  {
    if (backtrack_branch_id_ < 0 ||
      backtrack_branch_id_ >= static_cast<int>(branches_.size()))
    {
      changeMode(Mode::Explore);
      return makeExploreCommand();
    }

    BranchPoint & branch = branches_[static_cast<std::size_t>(backtrack_branch_id_)];
    if (distance2d(pose_.x, pose_.y, branch.pose.x, branch.pose.y) <= branch_reached_radius_) {
      beginAlignToUntried(backtrack_branch_id_);
      return makeAlignBranchCommand();
    }

    Pose2D target = branch.pose;
    if (!breadcrumbs_.empty()) {
      const std::size_t target_index =
        std::min(branch.breadcrumb_index, breadcrumbs_.size() - 1);
      while (backtrack_index_ > target_index &&
        distance2d(
          pose_.x, pose_.y, breadcrumbs_[backtrack_index_].x,
          breadcrumbs_[backtrack_index_].y) < breadcrumb_reached_radius_)
      {
        --backtrack_index_;
      }
      target = breadcrumbs_[std::max(target_index, backtrack_index_)];
    }

    return driveTowardPoint(target.x, target.y, backtrack_linear_speed_);
  }

  geometry_msgs::msg::Twist makeAlignBranchCommand()
  {
    geometry_msgs::msg::Twist cmd;
    if (align_branch_id_ < 0 ||
      align_branch_id_ >= static_cast<int>(branches_.size()))
    {
      changeMode(Mode::Explore);
      return cmd;
    }

    BranchPoint & branch = branches_[static_cast<std::size_t>(align_branch_id_)];
    if (align_option_index_ < 0 ||
      align_option_index_ >= static_cast<int>(branch.options.size()))
    {
      beginAlignToUntried(align_branch_id_);
      return cmd;
    }

    BranchOption & option = branch.options[static_cast<std::size_t>(align_option_index_)];
    const double yaw_error = normalizeAngle(option.yaw - pose_.yaw);
    const double target_clearance = sectorMin(yaw_error - degToRad(12.0), yaw_error + degToRad(12.0));

    if (target_clearance < stop_distance_) {
      option.blocked = true;
      beginAlignToUntried(align_branch_id_);
      return cmd;
    }

    if (std::abs(yaw_error) > align_yaw_tolerance_) {
      cmd.angular.z = clamp(steering_gain_ * yaw_error, -max_angular_speed_, max_angular_speed_);
      return cmd;
    }

    active_branch_id_ = align_branch_id_;
    active_option_index_ = align_option_index_;
    changeMode(Mode::Explore);

    cmd.linear.x = std::min(min_linear_speed_, max_linear_speed_);
    cmd.angular.z = clamp(steering_gain_ * yaw_error, -max_angular_speed_, max_angular_speed_);
    return cmd;
  }

  geometry_msgs::msg::Twist driveTowardPoint(double target_x, double target_y, double speed_limit)
  {
    geometry_msgs::msg::Twist cmd;
    const double target_yaw = std::atan2(target_y - pose_.y, target_x - pose_.x);
    const double yaw_error = normalizeAngle(target_yaw - pose_.yaw);
    const double front_min = sectorMin(degToRad(-14.0), degToRad(14.0));
    const double front_wide_min = sectorMin(degToRad(-32.0), degToRad(32.0));

    if (front_min < hard_stop_distance_) {
      cmd.angular.z = chooseOpenTurnSign() * max_angular_speed_ * 0.55;
      return cmd;
    }

    if (std::abs(yaw_error) > degToRad(55.0)) {
      cmd.angular.z = clamp(steering_gain_ * yaw_error, -max_angular_speed_, max_angular_speed_);
      return cmd;
    }

    const double speed_scale =
      clamp((front_wide_min - stop_distance_) / std::max(0.01, slow_distance_ - stop_distance_), 0.0, 1.0);
    cmd.linear.x = clamp(speed_limit * speed_scale, 0.0, speed_limit);
    if (cmd.linear.x > 0.0) {
      cmd.linear.x = std::max(min_linear_speed_, cmd.linear.x);
    }
    cmd.angular.z = clamp(steering_gain_ * yaw_error + computeWallBalance(),
      -max_angular_speed_, max_angular_speed_);
    return cmd;
  }

  void startDeadEndRecovery()
  {
    markActiveOptionBlocked();
    const int branch_id = findBacktrackBranch();
    target_branch_id_ = branch_id;
    backup_start_pose_ = pose_;
    backup_start_time_ = now();
    backup_turn_sign_ = chooseOpenTurnSign();
    changeMode(Mode::Backup);

    if (branch_id >= 0) {
      RCLCPP_INFO(get_logger(), "Dead end: backing up, then returning to branch %d.", branch_id);
    } else {
      RCLCPP_INFO(get_logger(), "Dead end: backing up, no untried branch recorded.");
    }
  }

  void startBacktrackToBranch(int branch_id)
  {
    if (branch_id < 0 || branch_id >= static_cast<int>(branches_.size())) {
      changeMode(Mode::Explore);
      return;
    }
    backtrack_branch_id_ = branch_id;
    const BranchPoint & branch = branches_[static_cast<std::size_t>(branch_id)];
    if (!breadcrumbs_.empty()) {
      backtrack_index_ = breadcrumbs_.size() - 1;
      const std::size_t target_index =
        std::min(branch.breadcrumb_index, breadcrumbs_.size() - 1);
      while (backtrack_index_ > target_index &&
        distance2d(
          pose_.x, pose_.y, breadcrumbs_[backtrack_index_].x,
          breadcrumbs_[backtrack_index_].y) < breadcrumb_reached_radius_)
      {
        --backtrack_index_;
      }
    }
    changeMode(Mode::Backtrack);
  }

  void beginAlignToUntried(int branch_id)
  {
    if (branch_id < 0 || branch_id >= static_cast<int>(branches_.size())) {
      changeMode(Mode::Explore);
      return;
    }

    BranchPoint & branch = branches_[static_cast<std::size_t>(branch_id)];
    int best_option = -1;
    double best_score = -std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < branch.options.size(); ++i) {
      const BranchOption & option = branch.options[i];
      if (option.tried || option.blocked) {
        continue;
      }
      const double rel = normalizeAngle(option.yaw - pose_.yaw);
      const double clearance = sectorMean(rel - degToRad(12.0), rel + degToRad(12.0));
      const double front_bonus = std::cos(rel) * 0.2;
      const double score = clearance + front_bonus;
      if (score > best_score) {
        best_score = score;
        best_option = static_cast<int>(i);
      }
    }

    if (best_option < 0) {
      removeBranchFromStack(branch_id);
      const int next_branch = findBacktrackBranch();
      if (next_branch >= 0) {
        target_branch_id_ = next_branch;
        startBacktrackToBranch(next_branch);
        return;
      }
      changeMode(Mode::Explore);
      return;
    }

    branch.options[static_cast<std::size_t>(best_option)].tried = true;
    align_branch_id_ = branch_id;
    align_option_index_ = best_option;
    pushBranchStack(branch_id);
    changeMode(Mode::AlignBranch);
    RCLCPP_INFO(
      get_logger(), "Trying branch %d option %d.", branch_id, best_option);
  }

  void updateBranchMemory(const std::vector<RaySegment> & branch_segments)
  {
    if (branch_segments.size() < 2) {
      return;
    }

    int branch_id = findNearbyBranch(branch_record_radius_);
    if (branch_id < 0) {
      BranchPoint branch;
      branch.pose = pose_;
      branch.breadcrumb_index = breadcrumbs_.empty() ? 0 : breadcrumbs_.size() - 1;
      branch.last_seen = now();
      branches_.push_back(branch);
      branch_id = static_cast<int>(branches_.size()) - 1;
      RCLCPP_INFO(get_logger(), "Recorded branch %d.", branch_id);
    }

    BranchPoint & branch = branches_[static_cast<std::size_t>(branch_id)];
    branch.pose.x = 0.75 * branch.pose.x + 0.25 * pose_.x;
    branch.pose.y = 0.75 * branch.pose.y + 0.25 * pose_.y;
    branch.pose.yaw = pose_.yaw;
    branch.last_seen = now();

    for (const auto & segment : branch_segments) {
      const double yaw = normalizeAngle(pose_.yaw + segment.center);
      bool exists = false;
      for (const auto & option : branch.options) {
        if (angleDistance(option.yaw, yaw) < branch_option_separation_) {
          exists = true;
          break;
        }
      }
      if (!exists) {
        BranchOption option;
        option.yaw = yaw;
        branch.options.push_back(option);
        RCLCPP_INFO(
          get_logger(), "Branch %d learned option %zu yaw %.2f.",
          branch_id, branch.options.size() - 1, yaw);
      }
    }
  }

  void maybeCommitBranchChoice(int branch_id, double selected_yaw)
  {
    if (branch_id < 0 || branch_id >= static_cast<int>(branches_.size())) {
      return;
    }

    BranchPoint & branch = branches_[static_cast<std::size_t>(branch_id)];
    if (branch.options.size() < 2) {
      return;
    }

    int option_index = closestBranchOption(branch, selected_yaw, branch_option_match_);
    if (option_index < 0) {
      return;
    }

    BranchOption & option = branch.options[static_cast<std::size_t>(option_index)];
    if (option.blocked) {
      return;
    }
    if (!option.tried || active_branch_id_ != branch_id) {
      option.tried = true;
      active_branch_id_ = branch_id;
      active_option_index_ = option_index;
      pushBranchStack(branch_id);
    }
  }

  int selectBestSegment(std::vector<RaySegment> & segments, int nearby_branch)
  {
    int best_index = 0;
    double best_score = -std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < segments.size(); ++i) {
      RaySegment & segment = segments[i];
      const double depth = std::min(segment.max_range, ray_score_max_range_);
      const double mean_depth = std::min(segment.mean_range, ray_score_max_range_);
      const double center_bonus = std::cos(segment.center);
      const double width_bonus = std::min(segment.width, degToRad(70.0));
      double score =
        depth_weight_ * (0.72 * depth + 0.28 * mean_depth) +
        center_preference_weight_ * center_bonus +
        width_weight_ * width_bonus -
        turn_penalty_weight_ * std::abs(segment.center);

      if (nearby_branch >= 0 &&
        nearby_branch < static_cast<int>(branches_.size()))
      {
        const BranchPoint & branch = branches_[static_cast<std::size_t>(nearby_branch)];
        const double yaw = normalizeAngle(pose_.yaw + segment.center);
        const int option_index = closestBranchOption(branch, yaw, branch_option_match_);
        if (option_index >= 0) {
          const BranchOption & option = branch.options[static_cast<std::size_t>(option_index)];
          if (!option.tried && !option.blocked) {
            score += untried_branch_bonus_;
          }
          if (option.tried) {
            score -= tried_branch_penalty_;
          }
          if (option.blocked) {
            score -= blocked_branch_penalty_;
          }
        }
      }

      segment.score = score;
      if (score > best_score) {
        best_score = score;
        best_index = static_cast<int>(i);
      }
    }

    return best_index;
  }

  std::vector<RaySegment> findOpenSegments(
    double half_fov, double min_depth, double min_width) const
  {
    std::vector<std::pair<double, double>> samples;
    if (!latest_scan_) {
      return {};
    }

    samples.reserve(latest_scan_->ranges.size());
    for (std::size_t i = 0; i < latest_scan_->ranges.size(); ++i) {
      const double range = cleanedRange(latest_scan_->ranges[i]);
      if (range <= 0.0 || range < min_depth) {
        continue;
      }
      const double rel = relativeAngleForIndex(i);
      if (rel < -half_fov || rel > half_fov) {
        continue;
      }
      samples.emplace_back(rel, range);
    }

    std::sort(samples.begin(), samples.end(), [](const auto & a, const auto & b) {
      return a.first < b.first;
    });

    std::vector<RaySegment> segments;
    if (samples.empty()) {
      return segments;
    }

    std::size_t start = 0;
    for (std::size_t i = 1; i <= samples.size(); ++i) {
      const bool end_segment =
        i == samples.size() || (samples[i].first - samples[i - 1].first) > segment_gap_;
      if (!end_segment) {
        continue;
      }

      RaySegment segment;
      segment.left = samples[start].first;
      segment.right = samples[i - 1].first;
      segment.width = segment.right - segment.left;
      segment.count = static_cast<int>(i - start);

      double weighted_angle_sum = 0.0;
      double weight_sum = 0.0;
      double range_sum = 0.0;
      double max_range = 0.0;
      for (std::size_t j = start; j < i; ++j) {
        const double weight = std::min(samples[j].second, ray_score_max_range_);
        weighted_angle_sum += samples[j].first * weight;
        weight_sum += weight;
        range_sum += samples[j].second;
        max_range = std::max(max_range, samples[j].second);
      }
      segment.center = weight_sum > 0.0 ?
        weighted_angle_sum / weight_sum : 0.5 * (segment.left + segment.right);
      segment.mean_range = range_sum / std::max(1, segment.count);
      segment.max_range = max_range;

      if (segment.width >= min_width || segment.count >= 3) {
        segments.push_back(segment);
      }
      start = i;
    }

    return segments;
  }

  double cleanedRange(float raw) const
  {
    if (!latest_scan_) {
      return 0.0;
    }
    if (std::isinf(raw)) {
      return latest_scan_->range_max;
    }
    if (std::isnan(raw) || raw < latest_scan_->range_min) {
      return 0.0;
    }
    return clamp(static_cast<double>(raw), latest_scan_->range_min, latest_scan_->range_max);
  }

  double relativeAngleForIndex(std::size_t index) const
  {
    const double scan_angle =
      latest_scan_->angle_min + static_cast<double>(index) * latest_scan_->angle_increment;
    return normalizeAngle(scan_angle - front_angle_offset_);
  }

  bool angleInInterval(double angle, double left, double right) const
  {
    angle = normalizeAngle(angle);
    left = normalizeAngle(left);
    right = normalizeAngle(right);
    if (left <= right) {
      return angle >= left && angle <= right;
    }
    return angle >= left || angle <= right;
  }

  double sectorMin(double left, double right) const
  {
    double result = std::numeric_limits<double>::infinity();
    if (!latest_scan_) {
      return result;
    }
    for (std::size_t i = 0; i < latest_scan_->ranges.size(); ++i) {
      const double rel = relativeAngleForIndex(i);
      if (!angleInInterval(rel, left, right)) {
        continue;
      }
      const double range = cleanedRange(latest_scan_->ranges[i]);
      if (range > 0.0) {
        result = std::min(result, range);
      }
    }
    if (!std::isfinite(result)) {
      return latest_scan_->range_max;
    }
    return result;
  }

  double sectorMax(double left, double right) const
  {
    double result = 0.0;
    if (!latest_scan_) {
      return result;
    }
    for (std::size_t i = 0; i < latest_scan_->ranges.size(); ++i) {
      const double rel = relativeAngleForIndex(i);
      if (!angleInInterval(rel, left, right)) {
        continue;
      }
      result = std::max(result, cleanedRange(latest_scan_->ranges[i]));
    }
    return result;
  }

  double sectorMean(double left, double right) const
  {
    double sum = 0.0;
    int count = 0;
    if (!latest_scan_) {
      return 0.0;
    }
    for (std::size_t i = 0; i < latest_scan_->ranges.size(); ++i) {
      const double rel = relativeAngleForIndex(i);
      if (!angleInInterval(rel, left, right)) {
        continue;
      }
      const double range = cleanedRange(latest_scan_->ranges[i]);
      if (range > 0.0) {
        sum += range;
        ++count;
      }
    }
    if (count == 0) {
      return latest_scan_->range_max;
    }
    return sum / static_cast<double>(count);
  }

  double bestRayAngle(double left, double right) const
  {
    double best_angle = 0.0;
    double best_score = -std::numeric_limits<double>::infinity();
    if (!latest_scan_) {
      return best_angle;
    }
    for (std::size_t i = 0; i < latest_scan_->ranges.size(); ++i) {
      const double rel = relativeAngleForIndex(i);
      if (!angleInInterval(rel, left, right)) {
        continue;
      }
      const double range = cleanedRange(latest_scan_->ranges[i]);
      const double score =
        std::min(range, ray_score_max_range_) +
        0.45 * std::cos(rel) -
        0.18 * std::abs(rel);
      if (score > best_score) {
        best_score = score;
        best_angle = rel;
      }
    }
    return best_angle;
  }

  double computeWallBalance() const
  {
    const double left = sectorMean(degToRad(28.0), degToRad(80.0));
    const double right = sectorMean(degToRad(-80.0), degToRad(-28.0));
    const double diff = clamp(left - right, -0.8, 0.8);
    return wall_balance_gain_ * diff;
  }

  double chooseOpenTurnSign() const
  {
    const double left =
      0.65 * sectorMean(degToRad(20.0), degToRad(115.0)) +
      0.35 * sectorMax(degToRad(20.0), degToRad(115.0));
    const double right =
      0.65 * sectorMean(degToRad(-115.0), degToRad(-20.0)) +
      0.35 * sectorMax(degToRad(-115.0), degToRad(-20.0));
    return left >= right ? 1.0 : -1.0;
  }

  void applyFinalSafety(geometry_msgs::msg::Twist & cmd) const
  {
    if (!latest_scan_) {
      cmd = geometry_msgs::msg::Twist();
      return;
    }

    if (cmd.linear.x > 0.0) {
      const double front_min = sectorMin(degToRad(-12.0), degToRad(12.0));
      const double front_wide_min = sectorMin(degToRad(-28.0), degToRad(28.0));
      if (front_min < hard_stop_distance_ || front_wide_min < stop_distance_) {
        cmd.linear.x = 0.0;
      } else if (front_wide_min < slow_distance_) {
        const double scale =
          clamp((front_wide_min - stop_distance_) /
          std::max(0.01, slow_distance_ - stop_distance_), 0.0, 1.0);
        cmd.linear.x *= scale;
      }
    }

    if (cmd.linear.x < 0.0) {
      const double rear_min = sectorMin(degToRad(165.0), degToRad(-165.0));
      if (rear_min < rear_stop_distance_) {
        cmd.linear.x = 0.0;
      }
    }

    cmd.linear.x = clamp(cmd.linear.x, -backup_speed_, max_linear_speed_);
    cmd.angular.z = clamp(cmd.angular.z, -max_angular_speed_, max_angular_speed_);
  }

  bool updateEscapedState()
  {
    const double front = sectorMin(degToRad(-45.0), degToRad(45.0));
    const double left = sectorMin(degToRad(45.0), degToRad(115.0));
    const double right = sectorMin(degToRad(-115.0), degToRad(-45.0));
    const bool clear = front > escaped_clearance_ && left > escaped_clearance_ * 0.65 &&
      right > escaped_clearance_ * 0.65;

    if (!clear) {
      escaped_since_ = now();
      return false;
    }

    if ((now() - escaped_since_).seconds() > escaped_hold_sec_) {
      RCLCPP_INFO_ONCE(get_logger(), "Open space detected; escape complete.");
      return true;
    }
    return false;
  }

  void updateBreadcrumb()
  {
    if (breadcrumbs_.empty() ||
      distance2d(pose_.x, pose_.y, breadcrumbs_.back().x, breadcrumbs_.back().y) >=
      breadcrumb_spacing_)
    {
      breadcrumbs_.push_back(pose_);
    }
  }

  int findNearbyBranch(double radius) const
  {
    int best_id = -1;
    double best_dist = radius;
    for (std::size_t i = 0; i < branches_.size(); ++i) {
      const double dist = distance2d(pose_.x, pose_.y, branches_[i].pose.x, branches_[i].pose.y);
      if (dist <= best_dist) {
        best_dist = dist;
        best_id = static_cast<int>(i);
      }
    }
    return best_id;
  }

  int closestBranchOption(const BranchPoint & branch, double yaw, double max_error) const
  {
    int best_index = -1;
    double best_error = max_error;
    for (std::size_t i = 0; i < branch.options.size(); ++i) {
      const double error = angleDistance(branch.options[i].yaw, yaw);
      if (error < best_error) {
        best_error = error;
        best_index = static_cast<int>(i);
      }
    }
    return best_index;
  }

  int findBacktrackBranch() const
  {
    for (auto it = branch_stack_.rbegin(); it != branch_stack_.rend(); ++it) {
      const int id = *it;
      if (id < 0 || id >= static_cast<int>(branches_.size())) {
        continue;
      }
      const BranchPoint & branch = branches_[static_cast<std::size_t>(id)];
      for (const auto & option : branch.options) {
        if (!option.tried && !option.blocked) {
          return id;
        }
      }
    }

    for (std::size_t i = 0; i < branches_.size(); ++i) {
      for (const auto & option : branches_[i].options) {
        if (!option.tried && !option.blocked) {
          return static_cast<int>(i);
        }
      }
    }
    return -1;
  }

  void markActiveOptionBlocked()
  {
    if (active_branch_id_ >= 0 &&
      active_branch_id_ < static_cast<int>(branches_.size()))
    {
      BranchPoint & branch = branches_[static_cast<std::size_t>(active_branch_id_)];
      if (active_option_index_ >= 0 &&
        active_option_index_ < static_cast<int>(branch.options.size()))
      {
        BranchOption & option = branch.options[static_cast<std::size_t>(active_option_index_)];
        option.tried = true;
        option.blocked = true;
        active_branch_id_ = -1;
        active_option_index_ = -1;
        return;
      }
    }

    if (!branch_stack_.empty()) {
      const int id = branch_stack_.back();
      if (id >= 0 && id < static_cast<int>(branches_.size())) {
        BranchPoint & branch = branches_[static_cast<std::size_t>(id)];
        const double yaw_from_branch =
          std::atan2(pose_.y - branch.pose.y, pose_.x - branch.pose.x);
        const int option_index = closestBranchOption(branch, yaw_from_branch, branch_option_match_);
        if (option_index >= 0) {
          BranchOption & option = branch.options[static_cast<std::size_t>(option_index)];
          option.tried = true;
          option.blocked = true;
        }
      }
    }
  }

  void pushBranchStack(int branch_id)
  {
    if (branch_id < 0) {
      return;
    }
    if (!branch_stack_.empty() && branch_stack_.back() == branch_id) {
      return;
    }
    branch_stack_.push_back(branch_id);
  }

  void removeBranchFromStack(int branch_id)
  {
    branch_stack_.erase(
      std::remove(branch_stack_.begin(), branch_stack_.end(), branch_id),
      branch_stack_.end());
  }

  void changeMode(Mode mode)
  {
    if (mode_ == mode) {
      return;
    }
    mode_ = mode;
    mode_enter_time_ = now();
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  rclcpp::Time last_scan_time_;
  bool have_scan_ = false;
  bool have_odom_ = false;

  Pose2D pose_;
  std::vector<Pose2D> breadcrumbs_;
  std::vector<BranchPoint> branches_;
  std::vector<int> branch_stack_;

  Mode mode_ = Mode::Explore;
  rclcpp::Time mode_enter_time_;
  rclcpp::Time escaped_since_;

  int active_branch_id_ = -1;
  int active_option_index_ = -1;
  int target_branch_id_ = -1;
  int backtrack_branch_id_ = -1;
  int align_branch_id_ = -1;
  int align_option_index_ = -1;
  std::size_t backtrack_index_ = 0;

  Pose2D backup_start_pose_;
  rclcpp::Time backup_start_time_;
  double backup_turn_sign_ = 1.0;

  std::string scan_topic_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;

  double front_angle_offset_ = 0.0;
  double control_rate_hz_ = 20.0;
  double scan_timeout_sec_ = 0.8;

  double max_linear_speed_ = 0.20;
  double min_linear_speed_ = 0.045;
  double backtrack_linear_speed_ = 0.10;
  double backup_speed_ = 0.075;
  double max_angular_speed_ = 1.35;
  double steering_gain_ = 1.55;
  double wall_balance_gain_ = 0.22;

  double hard_stop_distance_ = 0.145;
  double stop_distance_ = 0.19;
  double slow_distance_ = 0.62;
  double side_stop_distance_ = 0.135;
  double rear_stop_distance_ = 0.16;

  double search_half_fov_ = degToRad(77.5);
  double branch_half_fov_ = degToRad(90.0);
  double open_ray_min_depth_ = 0.46;
  double branch_min_depth_ = 0.85;
  double dead_end_depth_ = 0.52;
  double ray_score_max_range_ = 3.5;
  double min_segment_width_ = degToRad(8.0);
  double branch_min_segment_width_ = degToRad(12.0);
  double segment_gap_ = degToRad(4.0);

  double center_preference_weight_ = 0.90;
  double depth_weight_ = 1.00;
  double width_weight_ = 0.35;
  double turn_penalty_weight_ = 0.30;
  double untried_branch_bonus_ = 1.20;
  double tried_branch_penalty_ = 0.85;
  double blocked_branch_penalty_ = 2.20;

  double branch_record_radius_ = 0.42;
  double branch_reached_radius_ = 0.30;
  double branch_option_match_ = degToRad(28.0);
  double branch_option_separation_ = degToRad(32.0);

  double breadcrumb_spacing_ = 0.16;
  double breadcrumb_reached_radius_ = 0.22;
  double backup_distance_ = 0.32;
  double backup_timeout_sec_ = 3.0;
  double align_yaw_tolerance_ = degToRad(10.0);

  double escaped_clearance_ = 2.2;
  double escaped_hold_sec_ = 1.5;
  bool stop_when_escaped_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarEscapeNode>());
  rclcpp::shutdown();
  return 0;
}
