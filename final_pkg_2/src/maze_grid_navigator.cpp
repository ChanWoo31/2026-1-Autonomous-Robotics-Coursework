#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

using std::placeholders::_1;
using namespace std::chrono_literals;

class MazeGridNavigator : public rclcpp::Node
{
public:
    MazeGridNavigator() : Node("maze_grid_navigator")
    {
        occupied_threshold_ = this->declare_parameter<int>("occupied_threshold", 50);
        free_threshold_ = this->declare_parameter<int>("free_threshold", 20);
        planning_clearance_ = this->declare_parameter<double>("planning_clearance", 0.085);
        goal_clearance_ = this->declare_parameter<double>("goal_clearance", 0.10);
        preferred_clearance_ = this->declare_parameter<double>("preferred_clearance", 0.18);
        clearance_search_radius_ = this->declare_parameter<double>("clearance_search_radius", 0.45);
        min_target_distance_ = this->declare_parameter<double>("min_target_distance", 0.42);
        reached_target_distance_ = this->declare_parameter<double>("reached_target_distance", 0.18);
        replan_period_ = this->declare_parameter<double>("replan_period", 0.70);
        lookahead_distance_ = this->declare_parameter<double>("lookahead_distance", 0.36);
        max_linear_speed_ = this->declare_parameter<double>("max_linear_speed", 0.13);
        min_linear_speed_ = this->declare_parameter<double>("min_linear_speed", 0.045);
        max_angular_speed_ = this->declare_parameter<double>("max_angular_speed", 0.85);
        front_stop_distance_ = this->declare_parameter<double>("front_stop_distance", 0.23);
        front_critical_distance_ = this->declare_parameter<double>("front_critical_distance", 0.16);
        side_guard_distance_ = this->declare_parameter<double>("side_guard_distance", 0.105);
        stuck_timeout_ = this->declare_parameter<double>("stuck_timeout", 4.0);
        entrance_arm_distance_ = this->declare_parameter<double>("entrance_arm_distance", 1.15);
        entrance_block_radius_ = this->declare_parameter<double>("entrance_block_radius", 0.70);
        progress_backtrack_allowance_ = this->declare_parameter<double>("progress_backtrack_allowance", 0.35);
        min_exit_path_distance_ = this->declare_parameter<double>("min_exit_path_distance", 2.20);
        exit_open_distance_ = this->declare_parameter<double>("exit_open_distance", 1.15);
        exit_side_open_distance_ = this->declare_parameter<double>("exit_side_open_distance", 0.72);
        exit_run_distance_ = this->declare_parameter<double>("exit_run_distance", 1.35);
        max_scan_use_range_ = this->declare_parameter<double>("max_scan_use_range", 2.4);
        stop_after_exit_ = this->declare_parameter<bool>("stop_after_exit", true);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map",
            rclcpp::QoS(1).transient_local().reliable(),
            std::bind(&MazeGridNavigator::mapCallback, this, _1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SensorDataQoS(),
            std::bind(&MazeGridNavigator::scanCallback, this, _1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/maze_grid_path",
            rclcpp::QoS(1).transient_local().reliable());
        target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/maze_grid_target",
            rclcpp::QoS(1).transient_local().reliable());

        last_plan_time_ = this->now();
        last_progress_time_ = this->now();
        last_status_time_ = this->now();
        mode_started_time_ = this->now();

        control_timer_ = this->create_wall_timer(
            50ms,
            std::bind(&MazeGridNavigator::controlLoop, this));

        RCLCPP_INFO(
            this->get_logger(),
            "maze_grid_navigator ready: map A*/BFS + direct pure pursuit controller");
    }

private:
    enum class Mode
    {
        NAVIGATING,
        EXIT_RUN,
        FINISHED
    };

    struct Point2D
    {
        double x = 0.0;
        double y = 0.0;
    };

    struct Candidate
    {
        int index = -1;
        double score = -std::numeric_limits<double>::infinity();
        double start_distance = 0.0;
        double current_distance = 0.0;
        double clearance = 0.0;
        int cluster_size = 0;
        std::string reason;
    };

    struct QueueNode
    {
        int index = 0;
        double cost = 0.0;
    };

    struct QueueGreater
    {
        bool operator()(const QueueNode& lhs, const QueueNode& rhs) const
        {
            return lhs.cost > rhs.cost;
        }
    };

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

    rclcpp::Time last_scan_time_;
    rclcpp::Time last_plan_time_;
    rclcpp::Time last_progress_time_;
    rclcpp::Time last_status_time_;
    rclcpp::Time mode_started_time_;

    Mode mode_ = Mode::NAVIGATING;

    bool has_map_ = false;
    bool has_scan_ = false;
    bool has_pose_ = false;
    bool start_saved_ = false;
    bool need_replan_ = true;
    bool stop_after_exit_ = true;

    int occupied_threshold_ = 50;
    int free_threshold_ = 20;
    int width_ = 0;
    int height_ = 0;
    int current_index_ = -1;
    int start_index_ = -1;
    int target_index_ = -1;
    int exit_open_count_ = 0;

    double resolution_ = 0.02;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    double planning_clearance_ = 0.085;
    double goal_clearance_ = 0.10;
    double preferred_clearance_ = 0.18;
    double clearance_search_radius_ = 0.45;
    double min_target_distance_ = 0.42;
    double reached_target_distance_ = 0.18;
    double replan_period_ = 0.70;
    double lookahead_distance_ = 0.36;
    double max_linear_speed_ = 0.13;
    double min_linear_speed_ = 0.045;
    double max_angular_speed_ = 0.85;
    double front_stop_distance_ = 0.23;
    double front_critical_distance_ = 0.16;
    double side_guard_distance_ = 0.105;
    double stuck_timeout_ = 4.0;
    double entrance_arm_distance_ = 1.15;
    double entrance_block_radius_ = 0.70;
    double progress_backtrack_allowance_ = 0.35;
    double min_exit_path_distance_ = 2.20;
    double exit_open_distance_ = 1.15;
    double exit_side_open_distance_ = 0.72;
    double exit_run_distance_ = 1.35;
    double max_scan_use_range_ = 2.4;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;
    double previous_x_ = 0.0;
    double previous_y_ = 0.0;
    double start_x_ = 0.0;
    double start_y_ = 0.0;
    double start_yaw_ = 0.0;
    double path_distance_ = 0.0;
    double max_start_grid_distance_ = 0.0;
    double current_start_grid_distance_ = 0.0;
    double last_progress_path_distance_ = 0.0;
    double last_goal_distance_ = std::numeric_limits<double>::infinity();
    double exit_start_path_distance_ = 0.0;

    Point2D current_target_;
    std::vector<double> clearance_;
    std::vector<uint8_t> traversable_;
    std::vector<Point2D> path_points_;
    std::vector<Point2D> blacklist_;

    geometry_msgs::msg::Twist last_cmd_;

    static double clamp(double value, double low, double high)
    {
        return std::max(low, std::min(high, value));
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

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        latest_map_ = msg;
        has_map_ = true;
        need_replan_ = true;
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        latest_scan_ = msg;
        last_scan_time_ = this->now();
        has_scan_ = true;
    }

    bool updateRobotPose()
    {
        geometry_msgs::msg::TransformStamped transform;

        try {
            transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            return false;
        }

        robot_x_ = transform.transform.translation.x;
        robot_y_ = transform.transform.translation.y;

        const double qx = transform.transform.rotation.x;
        const double qy = transform.transform.rotation.y;
        const double qz = transform.transform.rotation.z;
        const double qw = transform.transform.rotation.w;

        robot_yaw_ = std::atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz));

        if (!has_pose_) {
            previous_x_ = robot_x_;
            previous_y_ = robot_y_;
            has_pose_ = true;
            last_progress_time_ = this->now();
        } else {
            const double step = std::hypot(robot_x_ - previous_x_, robot_y_ - previous_y_);

            if (step < 0.40) {
                path_distance_ += step;
            }

            previous_x_ = robot_x_;
            previous_y_ = robot_y_;
        }

        if (!start_saved_) {
            start_x_ = robot_x_;
            start_y_ = robot_y_;
            start_yaw_ = robot_yaw_;
            start_saved_ = true;

            RCLCPP_INFO(
                this->get_logger(),
                "Start saved: x=%.2f, y=%.2f, yaw=%.2f",
                start_x_,
                start_y_,
                start_yaw_);
        }

        return true;
    }

    int index(int mx, int my) const
    {
        return my * width_ + mx;
    }

    bool inBounds(int mx, int my) const
    {
        return mx >= 0 && mx < width_ && my >= 0 && my < height_;
    }

    bool worldToMap(double wx, double wy, int& mx, int& my) const
    {
        if (!has_map_) {
            return false;
        }

        mx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
        my = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
        return inBounds(mx, my);
    }

    Point2D mapToWorld(int mx, int my) const
    {
        return {
            origin_x_ + (static_cast<double>(mx) + 0.5) * resolution_,
            origin_y_ + (static_cast<double>(my) + 0.5) * resolution_};
    }

    Point2D indexToWorld(int idx) const
    {
        return mapToWorld(idx % width_, idx / width_);
    }

    double distanceFromStart(double wx, double wy) const
    {
        return std::hypot(wx - start_x_, wy - start_y_);
    }

    double forwardProjection(double wx, double wy) const
    {
        return ((wx - start_x_) * std::cos(start_yaw_)) +
               ((wy - start_y_) * std::sin(start_yaw_));
    }

    bool hasLeftEntrance() const
    {
        return path_distance_ > entrance_arm_distance_ ||
               max_start_grid_distance_ > entrance_arm_distance_ ||
               distanceFromStart(robot_x_, robot_y_) > entrance_arm_distance_;
    }

    bool isEntranceForbiddenPoint(const Point2D& point) const
    {
        const double start_distance = distanceFromStart(point.x, point.y);
        const double projection = forwardProjection(point.x, point.y);

        if (projection < -0.06 && start_distance > 0.16) {
            return true;
        }

        return hasLeftEntrance() &&
               start_distance < entrance_block_radius_ &&
               std::hypot(point.x - robot_x_, point.y - robot_y_) > 0.22;
    }

    bool shouldGuardEntrance() const
    {
        return hasLeftEntrance() &&
               (distanceFromStart(robot_x_, robot_y_) < entrance_block_radius_ ||
                forwardProjection(robot_x_, robot_y_) < -0.04);
    }

    bool isOccupiedMapCell(int idx) const
    {
        const int value = static_cast<int>(latest_map_->data[idx]);
        return value >= occupied_threshold_;
    }

    bool isFreeMapCell(int idx) const
    {
        const int value = static_cast<int>(latest_map_->data[idx]);
        return value >= 0 && value <= free_threshold_;
    }

    bool preparePlanningGrid()
    {
        if (!has_map_ || !latest_map_ || !start_saved_) {
            return false;
        }

        width_ = static_cast<int>(latest_map_->info.width);
        height_ = static_cast<int>(latest_map_->info.height);
        resolution_ = latest_map_->info.resolution;
        origin_x_ = latest_map_->info.origin.position.x;
        origin_y_ = latest_map_->info.origin.position.y;

        if (width_ <= 0 || height_ <= 0 || resolution_ <= 0.0) {
            return false;
        }

        const int cell_count = width_ * height_;
        clearance_.assign(cell_count, std::numeric_limits<double>::infinity());
        traversable_.assign(cell_count, 0);

        std::priority_queue<QueueNode, std::vector<QueueNode>, QueueGreater> queue;

        for (int my = 0; my < height_; ++my) {
            for (int mx = 0; mx < width_; ++mx) {
                const int idx = index(mx, my);
                const bool boundary = mx == 0 || my == 0 || mx == width_ - 1 || my == height_ - 1;

                if (boundary || isOccupiedMapCell(idx)) {
                    clearance_[idx] = 0.0;
                    queue.push({idx, 0.0});
                }
            }
        }

        const int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
        const int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};

        while (!queue.empty()) {
            const QueueNode node = queue.top();
            queue.pop();

            if (node.cost > clearance_[node.index] + 1e-9) {
                continue;
            }

            if (node.cost > clearance_search_radius_) {
                continue;
            }

            const int mx = node.index % width_;
            const int my = node.index / width_;

            for (int i = 0; i < 8; ++i) {
                const int nx = mx + dx8[i];
                const int ny = my + dy8[i];

                if (!inBounds(nx, ny)) {
                    continue;
                }

                const double step = (i < 4 ? 1.0 : std::sqrt(2.0)) * resolution_;
                const int nidx = index(nx, ny);
                const double candidate = node.cost + step;

                if (candidate + 1e-9 < clearance_[nidx]) {
                    clearance_[nidx] = candidate;
                    queue.push({nidx, candidate});
                }
            }
        }

        for (int my = 0; my < height_; ++my) {
            for (int mx = 0; mx < width_; ++mx) {
                const int idx = index(mx, my);

                if (!isFreeMapCell(idx)) {
                    continue;
                }

                if (clearance_[idx] < planning_clearance_) {
                    continue;
                }

                const Point2D point = mapToWorld(mx, my);

                if (isEntranceForbiddenPoint(point)) {
                    continue;
                }

                traversable_[idx] = 1;
            }
        }

        int current_mx = 0;
        int current_my = 0;

        if (!worldToMap(robot_x_, robot_y_, current_mx, current_my)) {
            return false;
        }

        int start_mx = 0;
        int start_my = 0;

        if (!worldToMap(start_x_, start_y_, start_mx, start_my)) {
            return false;
        }

        current_index_ = -1;
        start_index_ = -1;

        if (!nearestTraversableCell(index(current_mx, current_my), 0.35, current_index_)) {
            return false;
        }

        if (!nearestTraversableCell(index(start_mx, start_my), 0.45, start_index_)) {
            return false;
        }

        traversable_[current_index_] = 1;
        traversable_[start_index_] = 1;

        return true;
    }

    bool nearestTraversableCell(int seed_index, double radius, int& nearest_index) const
    {
        const int seed_mx = seed_index % width_;
        const int seed_my = seed_index / width_;
        const int radius_cells = std::max(1, static_cast<int>(std::ceil(radius / resolution_)));

        bool found = false;
        double best_distance = std::numeric_limits<double>::infinity();
        int best_index = -1;

        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                const int mx = seed_mx + dx;
                const int my = seed_my + dy;

                if (!inBounds(mx, my)) {
                    continue;
                }

                const int idx = index(mx, my);

                if (!traversable_[idx]) {
                    continue;
                }

                const double distance = std::hypot(dx * resolution_, dy * resolution_);

                if (distance < best_distance) {
                    found = true;
                    best_distance = distance;
                    best_index = idx;
                }
            }
        }

        nearest_index = best_index;
        return found;
    }

    bool canMoveDiagonal(int from_mx, int from_my, int dx, int dy) const
    {
        if (std::abs(dx) + std::abs(dy) != 2) {
            return true;
        }

        const int idx_a = index(from_mx + dx, from_my);
        const int idx_b = index(from_mx, from_my + dy);

        return traversable_[idx_a] && traversable_[idx_b];
    }

    void computeDistanceField(int start_idx, std::vector<double>& distances) const
    {
        distances.assign(width_ * height_, std::numeric_limits<double>::infinity());

        if (start_idx < 0 || start_idx >= static_cast<int>(distances.size())) {
            return;
        }

        std::priority_queue<QueueNode, std::vector<QueueNode>, QueueGreater> queue;
        distances[start_idx] = 0.0;
        queue.push({start_idx, 0.0});

        const int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
        const int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};

        while (!queue.empty()) {
            const QueueNode node = queue.top();
            queue.pop();

            if (node.cost > distances[node.index] + 1e-9) {
                continue;
            }

            const int mx = node.index % width_;
            const int my = node.index / width_;

            for (int i = 0; i < 8; ++i) {
                const int nx = mx + dx8[i];
                const int ny = my + dy8[i];

                if (!inBounds(nx, ny)) {
                    continue;
                }

                const int nidx = index(nx, ny);

                if (!traversable_[nidx] || !canMoveDiagonal(mx, my, dx8[i], dy8[i])) {
                    continue;
                }

                const double step = (i < 4 ? 1.0 : std::sqrt(2.0)) * resolution_;
                const double candidate = node.cost + step;

                if (candidate + 1e-9 < distances[nidx]) {
                    distances[nidx] = candidate;
                    queue.push({nidx, candidate});
                }
            }
        }
    }

    bool isFrontierCell(int idx) const
    {
        if (!traversable_[idx]) {
            return false;
        }

        const int mx = idx % width_;
        const int my = idx / width_;
        const int dx4[4] = {1, -1, 0, 0};
        const int dy4[4] = {0, 0, 1, -1};

        for (int i = 0; i < 4; ++i) {
            const int nx = mx + dx4[i];
            const int ny = my + dy4[i];

            if (!inBounds(nx, ny)) {
                return true;
            }

            const int nidx = index(nx, ny);
            const int value = static_cast<int>(latest_map_->data[nidx]);

            if (value < 0) {
                return true;
            }
        }

        return false;
    }

    bool isBlacklisted(const Point2D& point) const
    {
        for (const Point2D& blocked : blacklist_) {
            if (std::hypot(point.x - blocked.x, point.y - blocked.y) < 0.18) {
                return true;
            }
        }

        return false;
    }

    Candidate findFrontierTarget(
        const std::vector<double>& current_distances,
        const std::vector<double>& start_distances)
    {
        Candidate best;
        std::vector<uint8_t> visited(width_ * height_, 0);
        const int dx4[4] = {1, -1, 0, 0};
        const int dy4[4] = {0, 0, 1, -1};

        for (int idx = 0; idx < width_ * height_; ++idx) {
            if (visited[idx] || !isFrontierCell(idx)) {
                continue;
            }

            std::vector<int> cluster;
            std::queue<int> queue;
            queue.push(idx);
            visited[idx] = 1;

            while (!queue.empty()) {
                const int current = queue.front();
                queue.pop();
                cluster.push_back(current);

                const int mx = current % width_;
                const int my = current / width_;

                for (int i = 0; i < 4; ++i) {
                    const int nx = mx + dx4[i];
                    const int ny = my + dy4[i];

                    if (!inBounds(nx, ny)) {
                        continue;
                    }

                    const int nidx = index(nx, ny);

                    if (visited[nidx] || !isFrontierCell(nidx)) {
                        continue;
                    }

                    visited[nidx] = 1;
                    queue.push(nidx);
                }
            }

            if (cluster.size() < 5) {
                continue;
            }

            for (const int cell : cluster) {
                if (!std::isfinite(current_distances[cell]) ||
                    !std::isfinite(start_distances[cell])) {
                    continue;
                }

                if (current_distances[cell] < min_target_distance_) {
                    continue;
                }

                if (clearance_[cell] < goal_clearance_) {
                    continue;
                }

                const Point2D point = indexToWorld(cell);

                if (isBlacklisted(point) || isEntranceForbiddenPoint(point)) {
                    continue;
                }

                if (hasLeftEntrance() &&
                    start_distances[cell] + progress_backtrack_allowance_ < max_start_grid_distance_) {
                    continue;
                }

                const double projection = forwardProjection(point.x, point.y);

                if (!hasLeftEntrance() && projection < -0.05) {
                    continue;
                }

                double score = 0.0;
                score += 2.20 * start_distances[cell];
                score -= 0.34 * current_distances[cell];
                score += 0.75 * std::min(clearance_[cell], preferred_clearance_);
                score += 0.16 * std::sqrt(static_cast<double>(cluster.size())) * resolution_;

                if (path_distance_ < 1.00) {
                    score += 0.70 * projection;
                }

                if (clearance_[cell] < preferred_clearance_) {
                    score -= 0.80 * (preferred_clearance_ - clearance_[cell]);
                }

                if (score > best.score) {
                    best.index = cell;
                    best.score = score;
                    best.start_distance = start_distances[cell];
                    best.current_distance = current_distances[cell];
                    best.clearance = clearance_[cell];
                    best.cluster_size = static_cast<int>(cluster.size());
                    best.reason = "frontier";
                }
            }
        }

        return best;
    }

    Candidate findDeepFreeTarget(
        const std::vector<double>& current_distances,
        const std::vector<double>& start_distances)
    {
        Candidate best;

        for (int idx = 0; idx < width_ * height_; ++idx) {
            if (!traversable_[idx] ||
                !std::isfinite(current_distances[idx]) ||
                !std::isfinite(start_distances[idx])) {
                continue;
            }

            if (current_distances[idx] < min_target_distance_) {
                continue;
            }

            if (clearance_[idx] < planning_clearance_) {
                continue;
            }

            const Point2D point = indexToWorld(idx);

            if (isBlacklisted(point) || isEntranceForbiddenPoint(point)) {
                continue;
            }

            if (hasLeftEntrance() &&
                start_distances[idx] + progress_backtrack_allowance_ < max_start_grid_distance_) {
                continue;
            }

            double score = 0.0;
            score += 2.00 * start_distances[idx];
            score -= 0.22 * current_distances[idx];
            score += 0.80 * std::min(clearance_[idx], preferred_clearance_);

            if (path_distance_ < 1.00) {
                score += 0.50 * forwardProjection(point.x, point.y);
            }

            if (score > best.score) {
                best.index = idx;
                best.score = score;
                best.start_distance = start_distances[idx];
                best.current_distance = current_distances[idx];
                best.clearance = clearance_[idx];
                best.cluster_size = 0;
                best.reason = "deep-free";
            }
        }

        return best;
    }

    Candidate findTarget(
        std::vector<double>& current_distances,
        std::vector<double>& start_distances)
    {
        computeDistanceField(current_index_, current_distances);
        computeDistanceField(start_index_, start_distances);

        if (current_index_ >= 0 &&
            current_index_ < static_cast<int>(start_distances.size()) &&
            std::isfinite(start_distances[current_index_])) {
            current_start_grid_distance_ = start_distances[current_index_];
            max_start_grid_distance_ = std::max(
                max_start_grid_distance_,
                current_start_grid_distance_);
        }

        Candidate best = findFrontierTarget(current_distances, start_distances);

        if (best.index >= 0) {
            return best;
        }

        best = findDeepFreeTarget(current_distances, start_distances);

        if (best.index >= 0) {
            return best;
        }

        return Candidate();
    }

    double heuristic(int a, int b) const
    {
        const int ax = a % width_;
        const int ay = a / width_;
        const int bx = b % width_;
        const int by = b / width_;
        return std::hypot((ax - bx) * resolution_, (ay - by) * resolution_);
    }

    bool lineClear(int from_idx, int to_idx) const
    {
        const int x0 = from_idx % width_;
        const int y0 = from_idx / width_;
        const int x1 = to_idx % width_;
        const int y1 = to_idx / width_;

        const int steps = std::max(std::abs(x1 - x0), std::abs(y1 - y0));

        if (steps == 0) {
            return traversable_[from_idx] && clearance_[from_idx] >= planning_clearance_;
        }

        for (int step = 0; step <= steps; ++step) {
            const double ratio = static_cast<double>(step) / static_cast<double>(steps);
            const int mx = static_cast<int>(std::round(x0 + (x1 - x0) * ratio));
            const int my = static_cast<int>(std::round(y0 + (y1 - y0) * ratio));

            if (!inBounds(mx, my)) {
                return false;
            }

            const int idx = index(mx, my);

            if (!traversable_[idx] || clearance_[idx] < planning_clearance_) {
                return false;
            }
        }

        return true;
    }

    std::vector<int> smoothPath(const std::vector<int>& raw_path) const
    {
        if (raw_path.size() <= 2) {
            return raw_path;
        }

        std::vector<int> smooth;
        size_t anchor = 0;
        smooth.push_back(raw_path.front());

        while (anchor + 1 < raw_path.size()) {
            size_t next = anchor + 1;

            for (size_t candidate = raw_path.size() - 1; candidate > anchor + 1; --candidate) {
                if (lineClear(raw_path[anchor], raw_path[candidate])) {
                    next = candidate;
                    break;
                }
            }

            smooth.push_back(raw_path[next]);
            anchor = next;
        }

        return smooth;
    }

    bool makeAStarPath(int goal_idx, std::vector<int>& path)
    {
        path.clear();

        if (current_index_ < 0 || goal_idx < 0) {
            return false;
        }

        const int cell_count = width_ * height_;
        std::vector<double> g_score(cell_count, std::numeric_limits<double>::infinity());
        std::vector<int> parent(cell_count, -1);
        std::priority_queue<QueueNode, std::vector<QueueNode>, QueueGreater> open;

        g_score[current_index_] = 0.0;
        parent[current_index_] = current_index_;
        open.push({current_index_, heuristic(current_index_, goal_idx)});

        const int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
        const int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};

        while (!open.empty()) {
            const QueueNode node = open.top();
            open.pop();
            const int current = node.index;

            if (current == goal_idx) {
                break;
            }

            const int mx = current % width_;
            const int my = current / width_;

            for (int i = 0; i < 8; ++i) {
                const int nx = mx + dx8[i];
                const int ny = my + dy8[i];

                if (!inBounds(nx, ny)) {
                    continue;
                }

                const int nidx = index(nx, ny);

                if (!traversable_[nidx] || !canMoveDiagonal(mx, my, dx8[i], dy8[i])) {
                    continue;
                }

                const double move_cost = (i < 4 ? 1.0 : std::sqrt(2.0)) * resolution_;
                const double clearance_deficit =
                    std::max(0.0, preferred_clearance_ - clearance_[nidx]);
                const double clearance_penalty = 3.2 * clearance_deficit * clearance_deficit;
                const double candidate = g_score[current] + move_cost + clearance_penalty;

                if (candidate + 1e-9 < g_score[nidx]) {
                    g_score[nidx] = candidate;
                    parent[nidx] = current;
                    open.push({nidx, candidate + heuristic(nidx, goal_idx)});
                }
            }
        }

        if (parent[goal_idx] < 0) {
            return false;
        }

        std::vector<int> raw_path;
        int cursor = goal_idx;

        while (cursor != current_index_) {
            raw_path.push_back(cursor);
            cursor = parent[cursor];

            if (cursor < 0) {
                return false;
            }
        }

        raw_path.push_back(current_index_);
        std::reverse(raw_path.begin(), raw_path.end());
        path = smoothPath(raw_path);
        return path.size() >= 2;
    }

    void publishPath(const std::vector<int>& cell_path)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->now();

        for (const int cell : cell_path) {
            const Point2D point = indexToWorld(cell);
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }

        path_pub_->publish(path_msg);

        if (!cell_path.empty()) {
            geometry_msgs::msg::PoseStamped target;
            target.header = path_msg.header;
            target.pose.position.x = current_target_.x;
            target.pose.position.y = current_target_.y;
            target.pose.orientation.w = 1.0;
            target_pub_->publish(target);
        }
    }

    bool buildPlan()
    {
        if (!preparePlanningGrid()) {
            return false;
        }

        std::vector<double> current_distances;
        std::vector<double> start_distances;
        const Candidate target = findTarget(current_distances, start_distances);

        if (target.index < 0) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "No reachable map target. Rotating to collect more scan data.");
            path_points_.clear();
            target_index_ = -1;
            return false;
        }

        std::vector<int> cell_path;

        if (!makeAStarPath(target.index, cell_path)) {
            RCLCPP_WARN(
                this->get_logger(),
                "A* failed to selected target. Blacklisting and retrying.");
            blacklist_.push_back(indexToWorld(target.index));
            target_index_ = -1;
            need_replan_ = true;
            return false;
        }

        path_points_.clear();

        for (const int cell : cell_path) {
            path_points_.push_back(indexToWorld(cell));
        }

        target_index_ = target.index;
        current_target_ = indexToWorld(target.index);
        last_goal_distance_ = std::hypot(current_target_.x - robot_x_, current_target_.y - robot_y_);
        last_plan_time_ = this->now();
        need_replan_ = false;

        publishPath(cell_path);

        RCLCPP_INFO(
            this->get_logger(),
            "New %s target: x=%.2f y=%.2f path=%.2f start=%.2f clearance=%.2f cluster=%d",
            target.reason.c_str(),
            current_target_.x,
            current_target_.y,
            target.current_distance,
            target.start_distance,
            target.clearance,
            target.cluster_size);

        return true;
    }

    double sanitizeRange(const sensor_msgs::msg::LaserScan& scan, float range) const
    {
        if (!std::isfinite(range) || range < scan.range_min) {
            return max_scan_use_range_;
        }

        return std::min(static_cast<double>(range), max_scan_use_range_);
    }

    double sectorMin(const sensor_msgs::msg::LaserScan& scan, double min_angle, double max_angle) const
    {
        if (min_angle > max_angle) {
            std::swap(min_angle, max_angle);
        }

        double best = max_scan_use_range_;
        bool found = false;

        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;

            if (angle < min_angle || angle > max_angle) {
                continue;
            }

            const float raw = scan.ranges[i];

            if (!std::isfinite(raw) || raw < scan.range_min) {
                continue;
            }

            best = std::min(best, static_cast<double>(raw));
            found = true;
        }

        return found ? best : max_scan_use_range_;
    }

    double sectorPercentile(
        const sensor_msgs::msg::LaserScan& scan,
        double min_angle,
        double max_angle,
        double percentile) const
    {
        if (min_angle > max_angle) {
            std::swap(min_angle, max_angle);
        }

        std::vector<double> values;
        values.reserve(64);

        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;

            if (angle < min_angle || angle > max_angle) {
                continue;
            }

            values.push_back(sanitizeRange(scan, scan.ranges[i]));
        }

        if (values.empty()) {
            return max_scan_use_range_;
        }

        std::sort(values.begin(), values.end());
        const size_t idx = static_cast<size_t>(
            clamp(percentile, 0.0, 1.0) * static_cast<double>(values.size() - 1));

        return values[idx];
    }

    double localClearanceAt(const sensor_msgs::msg::LaserScan& scan, double angle) const
    {
        if (scan.angle_increment <= 0.0) {
            return max_scan_use_range_;
        }

        const int center = static_cast<int>(std::round((angle - scan.angle_min) / scan.angle_increment));
        const int window = std::max(2, static_cast<int>(std::round(0.09 / scan.angle_increment)));
        double clearance = max_scan_use_range_;

        for (int i = center - window; i <= center + window; ++i) {
            if (i < 0 || i >= static_cast<int>(scan.ranges.size())) {
                continue;
            }

            clearance = std::min(clearance, sanitizeRange(scan, scan.ranges[i]));
        }

        return clearance;
    }

    double plannedTargetAngle() const
    {
        if (path_points_.empty()) {
            return 0.0;
        }

        const Point2D& point = path_points_[std::min<size_t>(1, path_points_.size() - 1)];
        const double dx = point.x - robot_x_;
        const double dy = point.y - robot_y_;
        return normalizeAngle(std::atan2(dy, dx) - robot_yaw_);
    }

    double bestOpenAngle(const sensor_msgs::msg::LaserScan& scan) const
    {
        double best_score = -std::numeric_limits<double>::infinity();
        double best_angle = 0.0;
        const double plan_angle = plannedTargetAngle();

        for (size_t i = 0; i < scan.ranges.size(); i += 2) {
            const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;

            if (angle < -1.45 || angle > 1.45) {
                continue;
            }

            const double clearance = localClearanceAt(scan, angle);

            if (clearance < side_guard_distance_) {
                continue;
            }

            const double score =
                (1.60 * clearance) +
                (0.42 * std::cos(angle)) +
                (0.35 * std::cos(normalizeAngle(angle - plan_angle))) -
                (0.12 * std::abs(angle));

            if (score > best_score) {
                best_score = score;
                best_angle = angle;
            }
        }

        return best_angle;
    }

    bool maybeStartExitRun()
    {
        if (!has_scan_ || !latest_scan_ || mode_ != Mode::NAVIGATING) {
            return false;
        }

        if (path_distance_ < min_exit_path_distance_ ||
            max_start_grid_distance_ < entrance_arm_distance_) {
            exit_open_count_ = 0;
            return false;
        }

        const auto& scan = *latest_scan_;
        const double front_open = sectorPercentile(scan, -0.32, 0.32, 0.45);
        const double wide_open = sectorPercentile(scan, -0.85, 0.85, 0.25);
        const double left_open = sectorPercentile(scan, 0.45, 1.30, 0.35);
        const double right_open = sectorPercentile(scan, -1.30, -0.45, 0.35);

        if (front_open > exit_open_distance_ &&
            wide_open > 0.85 &&
            left_open > exit_side_open_distance_ &&
            right_open > exit_side_open_distance_) {
            ++exit_open_count_;
        } else {
            exit_open_count_ = std::max(0, exit_open_count_ - 1);
        }

        if (exit_open_count_ < 10) {
            return false;
        }

        mode_ = Mode::EXIT_RUN;
        mode_started_time_ = this->now();
        exit_start_path_distance_ = path_distance_;
        RCLCPP_INFO(this->get_logger(), "Exit opening detected. Driving out.");
        return true;
    }

    bool targetReached() const
    {
        if (target_index_ < 0) {
            return false;
        }

        return std::hypot(current_target_.x - robot_x_, current_target_.y - robot_y_) <
               reached_target_distance_;
    }

    void updateProgressWatchdog()
    {
        if (mode_ != Mode::NAVIGATING || target_index_ < 0) {
            return;
        }

        const double goal_distance =
            std::hypot(current_target_.x - robot_x_, current_target_.y - robot_y_);

        if (path_distance_ > last_progress_path_distance_ + 0.08 ||
            goal_distance + 0.05 < last_goal_distance_) {
            last_progress_path_distance_ = path_distance_;
            last_goal_distance_ = goal_distance;
            last_progress_time_ = this->now();
            return;
        }

        if ((this->now() - last_progress_time_).seconds() < stuck_timeout_) {
            return;
        }

        RCLCPP_WARN(
            this->get_logger(),
            "Progress watchdog: target blacklisted and replanning.");

        blacklist_.push_back(current_target_);
        path_points_.clear();
        target_index_ = -1;
        need_replan_ = true;
        last_progress_time_ = this->now();
    }

    geometry_msgs::msg::Twist makeSearchCommand()
    {
        geometry_msgs::msg::Twist cmd;

        if (!has_scan_ || !latest_scan_) {
            return cmd;
        }

        const auto& scan = *latest_scan_;
        const double front = sectorMin(scan, -0.25, 0.25);
        const double open_angle = bestOpenAngle(scan);

        if (front > 0.55 && has_map_) {
            cmd.linear.x = 0.035;
            cmd.angular.z = clamp(0.40 * open_angle, -0.35, 0.35);
        } else {
            cmd.linear.x = 0.0;
            cmd.angular.z = open_angle >= 0.0 ? 0.45 : -0.45;
        }

        return cmd;
    }

    geometry_msgs::msg::Twist makePathCommand()
    {
        geometry_msgs::msg::Twist cmd;

        if (path_points_.size() < 2) {
            return makeSearchCommand();
        }

        size_t nearest = 0;
        double nearest_distance = std::numeric_limits<double>::infinity();

        for (size_t i = 0; i < path_points_.size(); ++i) {
            const double distance = std::hypot(path_points_[i].x - robot_x_, path_points_[i].y - robot_y_);

            if (distance < nearest_distance) {
                nearest_distance = distance;
                nearest = i;
            }
        }

        size_t lookahead_index = nearest;
        double accumulated = 0.0;

        while (lookahead_index + 1 < path_points_.size() && accumulated < lookahead_distance_) {
            const Point2D& a = path_points_[lookahead_index];
            const Point2D& b = path_points_[lookahead_index + 1];
            accumulated += std::hypot(b.x - a.x, b.y - a.y);
            ++lookahead_index;
        }

        const Point2D& target = path_points_[lookahead_index];
        const double dx = target.x - robot_x_;
        const double dy = target.y - robot_y_;
        const double local_x = std::cos(robot_yaw_) * dx + std::sin(robot_yaw_) * dy;
        const double local_y = -std::sin(robot_yaw_) * dx + std::cos(robot_yaw_) * dy;
        const double target_distance = std::hypot(local_x, local_y);
        const double heading_error = std::atan2(local_y, std::max(0.05, local_x));

        double speed = max_linear_speed_;

        if (std::abs(heading_error) > 0.85 || local_x < 0.05) {
            speed = 0.0;
        } else if (std::abs(heading_error) > 0.45) {
            speed = min_linear_speed_;
        }

        if (target_distance < 0.20 && lookahead_index + 1 >= path_points_.size()) {
            speed = min_linear_speed_;
        }

        cmd.linear.x = speed;
        cmd.angular.z = clamp(1.45 * heading_error, -max_angular_speed_, max_angular_speed_);
        return cmd;
    }

    geometry_msgs::msg::Twist makeEntranceGuardCommand()
    {
        geometry_msgs::msg::Twist cmd;
        const double yaw_error = normalizeAngle(start_yaw_ - robot_yaw_);

        if (std::abs(yaw_error) > 0.28) {
            cmd.linear.x = 0.0;
            cmd.angular.z = clamp(1.35 * yaw_error, -max_angular_speed_, max_angular_speed_);
        } else {
            cmd.linear.x = min_linear_speed_;
            cmd.angular.z = clamp(0.70 * yaw_error, -0.35, 0.35);
        }

        return cmd;
    }

    geometry_msgs::msg::Twist makeExitRunCommand()
    {
        geometry_msgs::msg::Twist cmd;

        if (!has_scan_ || !latest_scan_) {
            return cmd;
        }

        if (stop_after_exit_ &&
            (path_distance_ - exit_start_path_distance_ > exit_run_distance_ ||
             (this->now() - mode_started_time_).seconds() > 12.0)) {
            mode_ = Mode::FINISHED;
            RCLCPP_INFO(this->get_logger(), "Exit run complete. Robot stopped outside the maze.");
            return cmd;
        }

        const auto& scan = *latest_scan_;

        if (sectorMin(scan, -0.24, 0.24) < front_stop_distance_) {
            mode_ = Mode::NAVIGATING;
            need_replan_ = true;
            return makeSearchCommand();
        }

        const double open_angle = bestOpenAngle(scan);
        cmd.linear.x = std::min(0.15, max_linear_speed_ + 0.02);
        cmd.angular.z = clamp(0.45 * open_angle, -0.30, 0.30);
        return cmd;
    }

    void applyLaserSafety(geometry_msgs::msg::Twist& cmd)
    {
        if (!has_scan_ || !latest_scan_) {
            cmd.linear.x = 0.0;
            return;
        }

        const auto& scan = *latest_scan_;
        const double front = sectorMin(scan, -0.20, 0.20);
        const double front_wide = sectorMin(scan, -0.38, 0.38);
        const double left = sectorMin(scan, 0.30, 1.30);
        const double right = sectorMin(scan, -1.30, -0.30);
        const double open_angle = bestOpenAngle(scan);

        if (front < front_critical_distance_) {
            cmd.linear.x = 0.0;
            cmd.angular.z = open_angle >= 0.0 ? max_angular_speed_ : -max_angular_speed_;
        } else if (front_wide < front_stop_distance_) {
            cmd.linear.x = std::min(cmd.linear.x, min_linear_speed_);
            cmd.angular.z += clamp(0.50 * open_angle, -0.35, 0.35);
        }

        if (left < side_guard_distance_) {
            cmd.linear.x = std::min(cmd.linear.x, min_linear_speed_);
            cmd.angular.z -= 0.45;
        }

        if (right < side_guard_distance_) {
            cmd.linear.x = std::min(cmd.linear.x, min_linear_speed_);
            cmd.angular.z += 0.45;
        }

        cmd.linear.x = clamp(cmd.linear.x, 0.0, 0.15);
        cmd.angular.z = clamp(cmd.angular.z, -max_angular_speed_, max_angular_speed_);
    }

    void publishCommand(const geometry_msgs::msg::Twist& desired)
    {
        geometry_msgs::msg::Twist cmd = desired;
        cmd.linear.x = last_cmd_.linear.x + clamp(cmd.linear.x - last_cmd_.linear.x, -0.020, 0.020);
        cmd.angular.z = last_cmd_.angular.z + clamp(cmd.angular.z - last_cmd_.angular.z, -0.14, 0.14);

        last_cmd_ = cmd;
        cmd_pub_->publish(cmd);
    }

    void publishStop()
    {
        geometry_msgs::msg::Twist cmd;
        last_cmd_ = cmd;
        cmd_pub_->publish(cmd);
    }

    void controlLoop()
    {
        if (!updateRobotPose()) {
            publishStop();
            return;
        }

        if (!has_scan_ || !latest_scan_ ||
            (this->now() - last_scan_time_).seconds() > 0.70) {
            publishStop();
            return;
        }

        if (mode_ == Mode::FINISHED) {
            publishStop();
            return;
        }

        maybeStartExitRun();
        updateProgressWatchdog();

        if (mode_ == Mode::NAVIGATING && shouldGuardEntrance()) {
            need_replan_ = true;
            path_points_.clear();
            target_index_ = -1;

            geometry_msgs::msg::Twist guard_cmd = makeEntranceGuardCommand();
            applyLaserSafety(guard_cmd);
            publishCommand(guard_cmd);
            return;
        }

        if (mode_ == Mode::NAVIGATING && targetReached()) {
            blacklist_.push_back(current_target_);
            target_index_ = -1;
            path_points_.clear();
            need_replan_ = true;
        }

        const bool periodic_replan =
            (this->now() - last_plan_time_).seconds() > replan_period_;

        if (mode_ == Mode::NAVIGATING &&
            (need_replan_ || periodic_replan || path_points_.empty())) {
            buildPlan();
        }

        geometry_msgs::msg::Twist cmd;

        if (mode_ == Mode::EXIT_RUN) {
            cmd = makeExitRunCommand();
        } else if (path_points_.empty()) {
            cmd = makeSearchCommand();
        } else {
            cmd = makePathCommand();
        }

        applyLaserSafety(cmd);
        publishCommand(cmd);

        if ((this->now() - last_status_time_).seconds() > 2.5) {
            last_status_time_ = this->now();
            RCLCPP_INFO(
                this->get_logger(),
                "mode=%s path_dist=%.2f start_grid=%.2f max_start_grid=%.2f target=(%.2f, %.2f)",
                mode_ == Mode::NAVIGATING ? "navigating" : "exit_run",
                path_distance_,
                current_start_grid_distance_,
                max_start_grid_distance_,
                current_target_.x,
                current_target_.y);
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MazeGridNavigator>());
    rclcpp::shutdown();
    return 0;
}
