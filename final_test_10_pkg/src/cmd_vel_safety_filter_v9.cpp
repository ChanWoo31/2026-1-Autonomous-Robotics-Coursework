#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class CmdVelSafetyFilter : public rclcpp::Node
{
public:
  CmdVelSafetyFilter() : Node("cmd_vel_safety_filter")
  {
    nav_cmd_topic_ = this->declare_parameter<std::string>("nav_cmd_topic", "/cmd_vel_nav");
    safe_cmd_topic_ = this->declare_parameter<std::string>("safe_cmd_topic", "/cmd_vel");
    scan_topic_ = this->declare_parameter<std::string>("scan_topic", "/scan");

    max_linear_ = this->declare_parameter<double>("max_linear", 0.16);
    max_reverse_ = this->declare_parameter<double>("max_reverse", 0.06);
    max_angular_ = this->declare_parameter<double>("max_angular", 1.50);

    // 사용자는 base_link -> laser yaw를 3.14159로 주고 있음.
    // 따라서 LaserScan raw frame 기준 로봇 정면은 보통 0도가 아니라 pi 근처다.
    // 만약 필터가 계속 이상하게 막으면 실행 시 front_angle_offset:=0.0으로 바꿔서 비교한다.
    front_angle_offset_ = this->declare_parameter<double>("front_angle_offset", 3.14159);

    front_stop_ = this->declare_parameter<double>("front_stop", 0.070);
    front_slow_ = this->declare_parameter<double>("front_slow", 0.180);
    side_stop_ = this->declare_parameter<double>("side_stop", 0.075);
    side_slow_ = this->declare_parameter<double>("side_slow", 0.110);
    front_half_width_deg_ = this->declare_parameter<double>("front_half_width_deg", 14.0);
    wide_half_width_deg_ = this->declare_parameter<double>("wide_half_width_deg", 24.0);
    corner_angle_deg_ = this->declare_parameter<double>("corner_angle_deg", 38.0);
    corner_half_width_deg_ = this->declare_parameter<double>("corner_half_width_deg", 10.0);
    side_angle_deg_ = this->declare_parameter<double>("side_angle_deg", 82.0);
    side_half_width_deg_ = this->declare_parameter<double>("side_half_width_deg", 12.0);
    side_turn_stop_ = this->declare_parameter<double>("side_turn_stop", 0.075);
    side_turn_slow_ = this->declare_parameter<double>("side_turn_slow", 0.125);
    corner_stop_ = this->declare_parameter<double>("corner_stop", 0.085);
    corner_slow_ = this->declare_parameter<double>("corner_slow", 0.145);
    side_linear_stop_ = this->declare_parameter<double>("side_linear_stop", 0.055);
    side_linear_slow_ = this->declare_parameter<double>("side_linear_slow", 0.095);
    side_imbalance_allowance_ = this->declare_parameter<double>("side_imbalance_allowance", 0.030);
    wall_nudge_angular_ = this->declare_parameter<double>("wall_nudge_angular", 0.18);
    pure_turn_linear_epsilon_ = this->declare_parameter<double>("pure_turn_linear_epsilon", 0.010);
    preserve_turn_angular_ = this->declare_parameter<double>("preserve_turn_angular", 0.080);
    min_creep_linear_ = this->declare_parameter<double>("min_creep_linear", 0.045);

    command_timeout_ = this->declare_parameter<double>("command_timeout", 0.35);
    scan_timeout_ = this->declare_parameter<double>("scan_timeout", 0.60);

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      nav_cmd_topic_, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_cmd_ = *msg;
        last_cmd_time_ = this->now();
        have_cmd_ = true;
      });

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_scan_ = *msg;
        last_scan_time_ = this->now();
        have_scan_ = true;
      });

    safe_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(safe_cmd_topic_, 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&CmdVelSafetyFilter::timerCallback, this));

    RCLCPP_WARN(
      this->get_logger(),
      "cmd_vel_safety_filter_v11_hybrid loaded: %s -> %s, scan=%s, front_offset=%.3f rad",
      nav_cmd_topic_.c_str(), safe_cmd_topic_.c_str(), scan_topic_.c_str(), front_angle_offset_);
  }

private:
  std::string nav_cmd_topic_;
  std::string safe_cmd_topic_;
  std::string scan_topic_;

  double max_linear_;
  double max_reverse_;
  double max_angular_;
  double front_angle_offset_;
  double front_stop_;
  double front_slow_;
  double side_stop_;
  double side_slow_;
  double front_half_width_deg_;
  double wide_half_width_deg_;
  double corner_angle_deg_;
  double corner_half_width_deg_;
  double side_angle_deg_;
  double side_half_width_deg_;
  double side_turn_stop_;
  double side_turn_slow_;
  double corner_stop_;
  double corner_slow_;
  double side_linear_stop_;
  double side_linear_slow_;
  double side_imbalance_allowance_;
  double wall_nudge_angular_;
  double pure_turn_linear_epsilon_;
  double preserve_turn_angular_;
  double min_creep_linear_;
  double command_timeout_;
  double scan_timeout_;

  std::mutex mutex_;
  geometry_msgs::msg::Twist last_cmd_;
  sensor_msgs::msg::LaserScan last_scan_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_scan_time_;
  bool have_cmd_ = false;
  bool have_scan_ = false;
  bool blocked_since_valid_ = false;
  bool release_active_ = false;
  rclcpp::Time blocked_since_;
  rclcpp::Time release_until_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  static double clamp(double v, double lo, double hi)
  {
    return std::max(lo, std::min(hi, v));
  }

  static double deg2rad(double deg)
  {
    return deg * M_PI / 180.0;
  }

  static double normalizeAngle(double a)
  {
    while (a > M_PI) {
      a -= 2.0 * M_PI;
    }
    while (a < -M_PI) {
      a += 2.0 * M_PI;
    }
    return a;
  }

  bool validRange(const sensor_msgs::msg::LaserScan& scan, double r) const
  {
    if (!std::isfinite(r)) {
      return false;
    }
    if (r < std::max(0.04f, scan.range_min)) {
      return false;
    }
    if (r > scan.range_max) {
      return false;
    }
    return true;
  }

  double sectorMinCentered(
    const sensor_msgs::msg::LaserScan& scan,
    double center,
    double half_width) const
  {
    if (scan.ranges.empty() || scan.angle_increment <= 0.0) {
      return std::numeric_limits<double>::infinity();
    }

    center = normalizeAngle(center);
    double best = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const double angle = normalizeAngle(scan.angle_min + static_cast<double>(i) * scan.angle_increment);
      const double diff = std::abs(normalizeAngle(angle - center));

      if (diff > half_width) {
        continue;
      }

      const double r = scan.ranges[i];
      if (validRange(scan, r)) {
        best = std::min(best, r);
      }
    }

    return best;
  }

  double slowdownScale(double distance, double stop_distance, double slow_distance) const
  {
    if (distance <= stop_distance) {
      return 0.0;
    }

    if (distance >= slow_distance) {
      return 1.0;
    }

    const double window = std::max(0.01, slow_distance - stop_distance);
    return clamp((distance - stop_distance) / window, 0.30, 1.0);
  }

  double awayFromCloseWallDirection(double left_distance, double right_distance) const
  {
    if (!std::isfinite(left_distance) && !std::isfinite(right_distance)) {
      return 0.0;
    }

    if (!std::isfinite(left_distance)) {
      return 1.0;
    }

    if (!std::isfinite(right_distance)) {
      return -1.0;
    }

    if (std::abs(left_distance - right_distance) < side_imbalance_allowance_) {
      return 0.0;
    }

    return left_distance < right_distance ? -1.0 : 1.0;
  }

  bool publishReleaseReverse(
    const rclcpp::Time& now,
    double rear_clearance,
    double left_distance,
    double right_distance,
    const char* reason,
    bool start_release)
  {
    const double release_rear_clearance = std::max(0.16, front_stop_ + 0.06);
    if (rear_clearance < release_rear_clearance) {
      return false;
    }

    geometry_msgs::msg::Twist release_cmd;
    release_cmd.linear.x = -std::min(0.045, max_reverse_);
    release_cmd.angular.z =
      awayFromCloseWallDirection(left_distance, right_distance) *
      std::min(0.12, max_angular_);

    if (start_release) {
      release_active_ = true;
      release_until_ = now + rclcpp::Duration::from_seconds(0.40);
      blocked_since_valid_ = false;
    }
    safe_pub_->publish(release_cmd);

    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "근접 정지 release 후진: rear=%.3f, linear=%.3f, angular=%.3f, reason=%s",
      rear_clearance,
      release_cmd.linear.x,
      release_cmd.angular.z,
      reason);

    return true;
  }

  void limitTurnTowardWall(
    geometry_msgs::msg::Twist& cmd,
    double left_distance,
    double right_distance,
    double stop_distance,
    double slow_distance) const
  {
    if (std::abs(cmd.linear.x) <= pure_turn_linear_epsilon_ &&
        std::abs(cmd.angular.z) >= preserve_turn_angular_) {
      return;
    }

    if (left_distance < stop_distance && cmd.angular.z > 0.0) {
      cmd.angular.z = 0.0;
    } else if (left_distance < slow_distance && cmd.angular.z > 0.0) {
      cmd.angular.z *= slowdownScale(left_distance, stop_distance, slow_distance);
    }

    if (right_distance < stop_distance && cmd.angular.z < 0.0) {
      cmd.angular.z = 0.0;
    } else if (right_distance < slow_distance && cmd.angular.z < 0.0) {
      cmd.angular.z *= slowdownScale(right_distance, stop_distance, slow_distance);
    }
  }

  void keepCenteredInNarrowPassage(
    geometry_msgs::msg::Twist& cmd,
    double left_distance,
    double right_distance) const
  {
    const double side_min = std::min(left_distance, right_distance);
    const double side_max = std::max(left_distance, right_distance);
    const double imbalance = side_max - side_min;

    if (side_min >= side_linear_slow_ || imbalance < side_imbalance_allowance_) {
      return;
    }

    if (std::abs(cmd.linear.x) <= pure_turn_linear_epsilon_ &&
        std::abs(cmd.angular.z) >= preserve_turn_angular_) {
      return;
    }

    const double away_from_close_wall =
      (left_distance < right_distance) ? -1.0 : 1.0;

    if (side_min < side_linear_stop_) {
      if (cmd.linear.x > 0.0) {
        cmd.linear.x = 0.0;
      }
      cmd.angular.z = away_from_close_wall *
        std::max(std::abs(cmd.angular.z), wall_nudge_angular_);
      return;
    }

    const double proximity_scale =
      1.0 - slowdownScale(side_min, side_linear_stop_, side_linear_slow_);
    const double imbalance_scale = std::clamp(
      imbalance / std::max(side_imbalance_allowance_ * 3.0, 0.001),
      0.30,
      1.0);
    const double nudge_scale = std::max(proximity_scale, imbalance_scale);

    cmd.angular.z += away_from_close_wall * wall_nudge_angular_ * nudge_scale;
  }

  void publishZero()
  {
    geometry_msgs::msg::Twist zero;
    safe_pub_->publish(zero);
  }

  void timerCallback()
  {
    geometry_msgs::msg::Twist cmd;
    sensor_msgs::msg::LaserScan scan;
    rclcpp::Time cmd_time;
    rclcpp::Time scan_time;
    bool have_cmd;
    bool have_scan;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      cmd = last_cmd_;
      scan = last_scan_;
      cmd_time = last_cmd_time_;
      scan_time = last_scan_time_;
      have_cmd = have_cmd_;
      have_scan = have_scan_;
    }

    const auto now = this->now();
    if (!have_cmd || (now - cmd_time).seconds() > command_timeout_) {
      publishZero();
      return;
    }

    if (!have_scan || (now - scan_time).seconds() > scan_timeout_) {
      publishZero();
      return;
    }

    // 회전 명령은 Nav2가 낸 값을 그대로 둔다.
    // 이 필터는 절대 새로운 회전 명령을 만들지 않는다.
    cmd.linear.x = clamp(cmd.linear.x, -max_reverse_, max_linear_);
    cmd.linear.y = 0.0;
    cmd.angular.z = clamp(cmd.angular.z, -max_angular_, max_angular_);

    const double front_narrow = sectorMinCentered(
      scan,
      front_angle_offset_,
      deg2rad(front_half_width_deg_));

    const double front_wide = sectorMinCentered(
      scan,
      front_angle_offset_,
      deg2rad(wide_half_width_deg_));

    const double left_corner = sectorMinCentered(
      scan,
      front_angle_offset_ + deg2rad(corner_angle_deg_),
      deg2rad(corner_half_width_deg_));

    const double right_corner = sectorMinCentered(
      scan,
      front_angle_offset_ - deg2rad(corner_angle_deg_),
      deg2rad(corner_half_width_deg_));

    const double left_side = sectorMinCentered(
      scan,
      front_angle_offset_ + deg2rad(side_angle_deg_),
      deg2rad(side_half_width_deg_));

    const double right_side = sectorMinCentered(
      scan,
      front_angle_offset_ - deg2rad(side_angle_deg_),
      deg2rad(side_half_width_deg_));

    const double rear_clearance = sectorMinCentered(
      scan,
      front_angle_offset_ + M_PI,
      deg2rad(wide_half_width_deg_));

    const double front_min = std::min(front_narrow, front_wide);
    const double corner_min = std::min(left_corner, right_corner);
    const double slow_min = std::min(front_min, corner_min);
    const double side_min = std::min(left_side, right_side);

    if (release_active_) {
      if (now < release_until_ &&
          publishReleaseReverse(
            now,
            rear_clearance,
            left_side,
            right_side,
            "active release",
            false)) {
        return;
      }
      release_active_ = false;
    }

    const bool wants_forward = cmd.linear.x > pure_turn_linear_epsilon_;
    const bool wants_reverse = cmd.linear.x < -pure_turn_linear_epsilon_;
    const bool hard_contact =
      front_narrow < front_stop_ ||
      corner_min < corner_stop_ ||
      side_min < side_linear_stop_;
    const bool hard_forward_blocked =
      wants_forward &&
      (hard_contact || front_wide < side_stop_);
    const bool contact_release_needed =
      hard_contact && !wants_reverse;

    if (hard_forward_blocked || contact_release_needed) {
      if (!blocked_since_valid_) {
        blocked_since_ = now;
        blocked_since_valid_ = true;
      } else if ((now - blocked_since_).seconds() >= 0.50 &&
                 publishReleaseReverse(
                   now,
                   rear_clearance,
                   left_side,
                   right_side,
                   contact_release_needed ? "hard contact near obstacle" : "forward blocked near obstacle",
                   true)) {
        return;
      }
    } else {
      blocked_since_valid_ = false;
    }

    if (cmd.linear.x > 0.0 && (front_narrow < front_stop_ || front_wide < side_stop_)) {
      cmd.linear.x = 0.0;
    } else if (cmd.linear.x > 0.0 && corner_min < corner_stop_) {
      cmd.linear.x = 0.0;
    } else if (cmd.linear.x > 0.0 && slow_min < std::max(front_slow_, corner_slow_)) {
      const double active_slow_limit =
        (corner_min < corner_slow_) ? corner_slow_ :
        ((front_wide < side_slow_) ? side_slow_ : front_slow_);
      const double active_stop_limit =
        (corner_min < corner_slow_) ? corner_stop_ :
        ((front_wide < side_slow_) ? side_stop_ : front_stop_);
      const double scale = slowdownScale(slow_min, active_stop_limit, active_slow_limit);
      cmd.linear.x *= scale;
      if (front_narrow > front_stop_ &&
          front_wide > side_stop_ &&
          corner_min > corner_stop_) {
        cmd.linear.x = std::max(
          cmd.linear.x,
          std::min(min_creep_linear_, max_linear_));
      }
    }

    if (front_wide < side_stop_) {
      const double turn_limit =
        (std::abs(cmd.linear.x) <= pure_turn_linear_epsilon_) ? 1.45 : 0.80;
      cmd.angular.z = clamp(cmd.angular.z, -turn_limit, turn_limit);
    }

    limitTurnTowardWall(
      cmd,
      left_corner,
      right_corner,
      corner_stop_,
      corner_slow_);

    limitTurnTowardWall(
      cmd,
      left_side,
      right_side,
      side_turn_stop_,
      side_turn_slow_);

    keepCenteredInNarrowPassage(cmd, left_side, right_side);

    cmd.angular.z = clamp(cmd.angular.z, -max_angular_, max_angular_);
    safe_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "v11 hybrid filter: front=(%.3f, %.3f), corner=(%.3f, %.3f), side=(%.3f, %.3f), in=(%.3f, %.3f), out=(%.3f, %.3f)",
      front_narrow,
      front_wide,
      left_corner,
      right_corner,
      left_side,
      right_side,
      last_cmd_.linear.x,
      last_cmd_.angular.z,
      cmd.linear.x,
      cmd.angular.z);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelSafetyFilter>());
  rclcpp::shutdown();
  return 0;
}
