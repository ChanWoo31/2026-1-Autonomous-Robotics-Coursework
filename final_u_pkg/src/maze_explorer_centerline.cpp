#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using std::placeholders::_1;

class MazeExplorerCenterline : public rclcpp::Node
{
public:
    MazeExplorerCenterline() : Node("maze_explorer_centerline")
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map",
            10,
            std::bind(&MazeExplorerCenterline::mapCallback, this, _1)
        );

        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        watchdog_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MazeExplorerCenterline::watchdogCallback, this)
        );
    }

private:
    struct Point2D
    {
        double x;
        double y;
    };

    struct SearchData
    {
        std::vector<double> clearance;
        std::vector<double> cost;
        std::vector<int> previous;
        std::vector<double> bottleneck;
        std::vector<int> start_distances;
    };

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    GoalHandleNav2::SharedPtr active_goal_handle_;

    bool goal_in_progress_ = false;
    bool canceling_goal_ = false;
    bool is_start_saved_ = false;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;
    double start_x_ = 0.0;
    double start_y_ = 0.0;
    double start_yaw_ = 0.0;
    double max_start_path_distance_ = 0.0;
    double current_start_path_distance_ = 0.0;
    double max_forward_projection_ = 0.0;
    double max_start_distance_ = 0.0;
    double current_goal_x_ = 0.0;
    double current_goal_y_ = 0.0;
    double current_target_x_ = 0.0;
    double current_target_y_ = 0.0;
    double best_goal_distance_ = std::numeric_limits<double>::max();
    rclcpp::Time last_progress_time_;
    std::vector<Point2D> blacklist_;

    /*
     * Centerline explorer policy:
     * - allow narrow passages by keeping the hard clearance modest
     * - prefer the medial line by making low-clearance cells expensive
     * - send short lookahead goals on the center-biased path
     */
    const double HARD_PATH_CLEARANCE = 0.045;
    const double RELAXED_PATH_CLEARANCE = 0.035;
    const double MIN_GOAL_CLEARANCE = 0.085;
    const double RELAXED_GOAL_CLEARANCE = 0.065;
    const double MIN_BOTTLENECK_CLEARANCE = 0.075;
    const double RELAXED_BOTTLENECK_CLEARANCE = 0.055;
    const double COMFORT_CLEARANCE = 0.17;
    const double MAX_CLEARANCE = 0.45;
    const double CENTER_COST_WEIGHT = 8.0;

    const double LOOKAHEAD_DISTANCE = 0.75;
    const double RELAXED_LOOKAHEAD_DISTANCE = 0.55;
    const double CENTER_SNAP_RADIUS = 0.18;
    const double MIN_TARGET_DISTANCE = 0.70;
    const double RELAXED_MIN_TARGET_DISTANCE = 0.35;

    const double START_FORWARD_MARGIN = 0.08;
    const double START_RETURN_ARM_DISTANCE = 1.50;
    const double START_RETURN_GOAL_RADIUS = 0.65;
    const double START_PATH_BACKTRACK_ALLOWANCE = 0.35;
    const double START_PATH_CANCEL_ARM_DISTANCE = 0.90;
    const double START_PATH_CANCEL_BACKTRACK = 0.45;
    const double BLACKLIST_RADIUS = 0.16;

    const double STUCK_GOAL_IMPROVEMENT = 0.03;
    const double STUCK_TIMEOUT = 7.0;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        latest_map_ = msg;

        if (!updateRobotPose()) {
            return;
        }

        saveStartPoseOnce();
        updateProgress(*msg);

        if (goal_in_progress_) {
            if (isBacktrackingTowardEntrance()) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "centerline: 입구 방향 되감기 감지. 목표 취소: 현재경로=%.2f, 최대경로=%.2f",
                    current_start_path_distance_,
                    max_start_path_distance_
                );
                cancelCurrentGoalAndBlacklist();
            }

            return;
        }

        trySendNextGoal();
    }

    bool updateRobotPose()
    {
        geometry_msgs::msg::TransformStamped t;

        try {
            t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException&) {
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
        if (is_start_saved_) {
            return;
        }

        start_x_ = robot_x_;
        start_y_ = robot_y_;
        start_yaw_ = robot_yaw_;
        max_start_path_distance_ = 0.0;
        current_start_path_distance_ = 0.0;
        max_forward_projection_ = 0.0;
        max_start_distance_ = 0.0;
        is_start_saved_ = true;

        RCLCPP_INFO(
            this->get_logger(),
            "centerline 시작 위치 저장: x=%.2f, y=%.2f, yaw=%.2f",
            start_x_,
            start_y_,
            start_yaw_
        );
    }

    int indexOf(const nav_msgs::msg::OccupancyGrid& map, int mx, int my) const
    {
        return my * static_cast<int>(map.info.width) + mx;
    }

    bool worldToMap(
        const nav_msgs::msg::OccupancyGrid& map,
        double wx,
        double wy,
        int& mx,
        int& my) const
    {
        mx = static_cast<int>((wx - map.info.origin.position.x) / map.info.resolution);
        my = static_cast<int>((wy - map.info.origin.position.y) / map.info.resolution);

        return mx >= 0 &&
               mx < static_cast<int>(map.info.width) &&
               my >= 0 &&
               my < static_cast<int>(map.info.height);
    }

    double mapToWorldX(const nav_msgs::msg::OccupancyGrid& map, int mx) const
    {
        return map.info.origin.position.x + (mx + 0.5) * map.info.resolution;
    }

    double mapToWorldY(const nav_msgs::msg::OccupancyGrid& map, int my) const
    {
        return map.info.origin.position.y + (my + 0.5) * map.info.resolution;
    }

    bool isFreeCell(const nav_msgs::msg::OccupancyGrid& map, int mx, int my) const
    {
        const int width = static_cast<int>(map.info.width);
        const int height = static_cast<int>(map.info.height);

        if (mx <= 0 || mx >= width - 1 || my <= 0 || my >= height - 1) {
            return false;
        }

        return map.data[indexOf(map, mx, my)] == 0;
    }

    bool isObstacleSource(const nav_msgs::msg::OccupancyGrid& map, int mx, int my) const
    {
        const int width = static_cast<int>(map.info.width);
        const int height = static_cast<int>(map.info.height);

        if (mx <= 0 || mx >= width - 1 || my <= 0 || my >= height - 1) {
            return true;
        }

        return map.data[indexOf(map, mx, my)] >= 50;
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
        return max_start_distance_ >= START_RETURN_ARM_DISTANCE;
    }

    bool isInEntranceReturnZone(double wx, double wy) const
    {
        return hasLeftEntrance() && distanceFromStart(wx, wy) < START_RETURN_GOAL_RADIUS;
    }

    bool isBacktrackingTowardEntrance() const
    {
        return max_start_path_distance_ >= START_PATH_CANCEL_ARM_DISTANCE &&
               current_start_path_distance_ + START_PATH_CANCEL_BACKTRACK <
               max_start_path_distance_;
    }

    std::vector<double> computeClearanceMap(const nav_msgs::msg::OccupancyGrid& map)
    {
        const int width = static_cast<int>(map.info.width);
        const int height = static_cast<int>(map.info.height);
        const int cell_count = width * height;
        const double inf = std::numeric_limits<double>::infinity();

        std::vector<double> clearance(cell_count, inf);

        struct QueueNode
        {
            int index;
            double distance;
        };

        struct PreferNearer
        {
            bool operator()(const QueueNode& lhs, const QueueNode& rhs) const
            {
                return lhs.distance > rhs.distance;
            }
        };

        std::priority_queue<QueueNode, std::vector<QueueNode>, PreferNearer> queue;

        for (int my = 0; my < height; ++my) {
            for (int mx = 0; mx < width; ++mx) {
                if (!isObstacleSource(map, mx, my)) {
                    continue;
                }

                const int index = indexOf(map, mx, my);
                clearance[index] = 0.0;
                queue.push({index, 0.0});
            }
        }

        const int dx[8] = {0, 0, -1, 1, -1, -1, 1, 1};
        const int dy[8] = {-1, 1, 0, 0, -1, 1, -1, 1};

        while (!queue.empty()) {
            const QueueNode node = queue.top();
            queue.pop();

            if (node.distance > clearance[node.index] + 1e-9 ||
                node.distance > MAX_CLEARANCE) {
                continue;
            }

            const int mx = node.index % width;
            const int my = node.index / width;

            for (int i = 0; i < 8; ++i) {
                const int nx = mx + dx[i];
                const int ny = my + dy[i];

                if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                    continue;
                }

                const double step =
                    (dx[i] == 0 || dy[i] == 0) ? map.info.resolution
                                                : map.info.resolution * std::sqrt(2.0);
                const int neighbor = indexOf(map, nx, ny);
                const double candidate = node.distance + step;

                if (candidate >= clearance[neighbor]) {
                    continue;
                }

                clearance[neighbor] = candidate;
                queue.push({neighbor, candidate});
            }
        }

        for (double& value : clearance) {
            value = std::min(value, MAX_CLEARANCE);
        }

        return clearance;
    }

    std::vector<int> computeStartDistances(const nav_msgs::msg::OccupancyGrid& map)
    {
        const int width = static_cast<int>(map.info.width);
        const int height = static_cast<int>(map.info.height);
        std::vector<int> distances(width * height, -1);

        int start_mx = 0;
        int start_my = 0;

        if (!worldToMap(map, start_x_, start_y_, start_mx, start_my)) {
            return distances;
        }

        std::queue<int> queue;
        const int start_index = indexOf(map, start_mx, start_my);
        distances[start_index] = 0;
        queue.push(start_index);

        const int dx[4] = {0, 0, -1, 1};
        const int dy[4] = {-1, 1, 0, 0};

        while (!queue.empty()) {
            const int current = queue.front();
            queue.pop();

            const int mx = current % width;
            const int my = current / width;

            for (int i = 0; i < 4; ++i) {
                const int nx = mx + dx[i];
                const int ny = my + dy[i];

                if (!isFreeCell(map, nx, ny)) {
                    continue;
                }

                const int neighbor = indexOf(map, nx, ny);

                if (distances[neighbor] >= 0) {
                    continue;
                }

                distances[neighbor] = distances[current] + 1;
                queue.push(neighbor);
            }
        }

        return distances;
    }

    void updateProgress(const nav_msgs::msg::OccupancyGrid& map)
    {
        max_forward_projection_ = std::max(
            max_forward_projection_,
            forwardProjection(robot_x_, robot_y_)
        );
        max_start_distance_ = std::max(max_start_distance_, distanceFromStart(robot_x_, robot_y_));

        const std::vector<int> start_distances = computeStartDistances(map);

        int robot_mx = 0;
        int robot_my = 0;

        if (!worldToMap(map, robot_x_, robot_y_, robot_mx, robot_my)) {
            return;
        }

        const int robot_index = indexOf(map, robot_mx, robot_my);

        if (start_distances[robot_index] < 0) {
            return;
        }

        current_start_path_distance_ =
            start_distances[robot_index] * map.info.resolution;
        max_start_path_distance_ = std::max(
            max_start_path_distance_,
            current_start_path_distance_
        );
    }

    bool isFrontierCell(const nav_msgs::msg::OccupancyGrid& map, int mx, int my) const
    {
        if (!isFreeCell(map, mx, my)) {
            return false;
        }

        const int width = static_cast<int>(map.info.width);
        const int dx[4] = {0, 0, -1, 1};
        const int dy[4] = {-1, 1, 0, 0};

        for (int i = 0; i < 4; ++i) {
            const int neighbor_index = (my + dy[i]) * width + (mx + dx[i]);

            if (map.data[neighbor_index] == -1) {
                return true;
            }
        }

        return false;
    }

    bool buildCenterBiasedSearch(
        const nav_msgs::msg::OccupancyGrid& map,
        bool relaxed,
        SearchData& search)
    {
        const int width = static_cast<int>(map.info.width);
        const int height = static_cast<int>(map.info.height);
        const int cell_count = width * height;
        const double inf = std::numeric_limits<double>::infinity();
        const double min_clearance = relaxed ? RELAXED_PATH_CLEARANCE : HARD_PATH_CLEARANCE;

        search.clearance = computeClearanceMap(map);
        search.start_distances = computeStartDistances(map);
        search.cost.assign(cell_count, inf);
        search.previous.assign(cell_count, -1);
        search.bottleneck.assign(cell_count, 0.0);

        int robot_mx = 0;
        int robot_my = 0;

        if (!worldToMap(map, robot_x_, robot_y_, robot_mx, robot_my)) {
            return false;
        }

        struct PathNode
        {
            int index;
            double cost;
        };

        struct PreferLowerCost
        {
            bool operator()(const PathNode& lhs, const PathNode& rhs) const
            {
                return lhs.cost > rhs.cost;
            }
        };

        std::priority_queue<PathNode, std::vector<PathNode>, PreferLowerCost> queue;
        const int start_index = indexOf(map, robot_mx, robot_my);

        search.cost[start_index] = 0.0;
        search.bottleneck[start_index] = search.clearance[start_index];
        queue.push({start_index, 0.0});

        const int dx[8] = {0, 0, -1, 1, -1, -1, 1, 1};
        const int dy[8] = {-1, 1, 0, 0, -1, 1, -1, 1};

        while (!queue.empty()) {
            const PathNode node = queue.top();
            queue.pop();

            if (node.cost > search.cost[node.index] + 1e-9) {
                continue;
            }

            const int mx = node.index % width;
            const int my = node.index / width;

            for (int i = 0; i < 8; ++i) {
                const int nx = mx + dx[i];
                const int ny = my + dy[i];

                if (!isFreeCell(map, nx, ny)) {
                    continue;
                }

                const int neighbor = indexOf(map, nx, ny);
                const double clearance = search.clearance[neighbor];

                if (clearance < min_clearance) {
                    continue;
                }

                const double wx = mapToWorldX(map, nx);
                const double wy = mapToWorldY(map, ny);

                if (isInEntranceReturnZone(wx, wy)) {
                    continue;
                }

                if (forwardProjection(wx, wy) < -START_FORWARD_MARGIN) {
                    continue;
                }

                const double step =
                    (dx[i] == 0 || dy[i] == 0) ? map.info.resolution
                                                : map.info.resolution * std::sqrt(2.0);
                const double low_clearance =
                    std::max(0.0, COMFORT_CLEARANCE - clearance) / COMFORT_CLEARANCE;
                const double center_cost =
                    step * (1.0 + CENTER_COST_WEIGHT * low_clearance * low_clearance);
                const double candidate = node.cost + center_cost;

                if (candidate >= search.cost[neighbor]) {
                    continue;
                }

                search.cost[neighbor] = candidate;
                search.previous[neighbor] = node.index;
                search.bottleneck[neighbor] = std::min(
                    search.bottleneck[node.index],
                    clearance
                );
                queue.push({neighbor, candidate});
            }
        }

        return true;
    }

    bool isBlacklisted(double wx, double wy) const
    {
        for (const auto& point : blacklist_) {
            if (std::hypot(wx - point.x, wy - point.y) < BLACKLIST_RADIUS) {
                return true;
            }
        }

        return false;
    }

    void blacklistCurrentGoalAndTarget()
    {
        blacklist_.push_back({current_goal_x_, current_goal_y_});
        blacklist_.push_back({current_target_x_, current_target_y_});
    }

    bool selectTargetCell(
        const nav_msgs::msg::OccupancyGrid& map,
        const SearchData& search,
        bool relaxed,
        int& target_index,
        bool& selected_frontier)
    {
        const int width = static_cast<int>(map.info.width);
        const int height = static_cast<int>(map.info.height);
        const double inf = std::numeric_limits<double>::infinity();
        const double min_goal_clearance = relaxed ? RELAXED_GOAL_CLEARANCE : MIN_GOAL_CLEARANCE;
        const double min_bottleneck =
            relaxed ? RELAXED_BOTTLENECK_CLEARANCE : MIN_BOTTLENECK_CLEARANCE;
        const double min_target_distance =
            relaxed ? RELAXED_MIN_TARGET_DISTANCE : MIN_TARGET_DISTANCE;

        bool found_frontier = false;
        bool found_known = false;
        int best_frontier = -1;
        int best_known = -1;
        double best_frontier_score = -inf;
        double best_known_score = -inf;

        for (int my = 1; my < height - 1; ++my) {
            for (int mx = 1; mx < width - 1; ++mx) {
                const int index = indexOf(map, mx, my);

                if (map.data[index] != 0 || !std::isfinite(search.cost[index])) {
                    continue;
                }

                if (search.cost[index] < min_target_distance) {
                    continue;
                }

                if (search.clearance[index] < min_goal_clearance ||
                    search.bottleneck[index] < min_bottleneck) {
                    continue;
                }

                if (search.start_distances[index] < 0) {
                    continue;
                }

                const double wx = mapToWorldX(map, mx);
                const double wy = mapToWorldY(map, my);

                if (isBlacklisted(wx, wy)) {
                    continue;
                }

                if (isInEntranceReturnZone(wx, wy)) {
                    continue;
                }

                const double projection = forwardProjection(wx, wy);

                if (projection < START_FORWARD_MARGIN) {
                    continue;
                }

                const double start_path = search.start_distances[index] * map.info.resolution;

                if (!relaxed &&
                    hasLeftEntrance() &&
                    start_path + START_PATH_BACKTRACK_ALLOWANCE < max_start_path_distance_) {
                    continue;
                }

                const double path_length = search.cost[index];
                const double score =
                    (3.0 * start_path) +
                    (1.2 * projection) +
                    (4.0 * search.clearance[index]) +
                    (3.0 * search.bottleneck[index]) -
                    (0.20 * path_length);

                if (isFrontierCell(map, mx, my)) {
                    if (!found_frontier || score > best_frontier_score) {
                        found_frontier = true;
                        best_frontier_score = score;
                        best_frontier = index;
                    }
                } else {
                    if (!found_known || score > best_known_score) {
                        found_known = true;
                        best_known_score = score;
                        best_known = index;
                    }
                }
            }
        }

        if (found_frontier) {
            target_index = best_frontier;
            selected_frontier = true;
            return true;
        }

        if (found_known) {
            target_index = best_known;
            selected_frontier = false;
            return true;
        }

        return false;
    }

    std::vector<int> reconstructPath(const SearchData& search, int target_index) const
    {
        std::vector<int> reversed_path;
        int current = target_index;

        while (current >= 0) {
            reversed_path.push_back(current);
            current = search.previous[current];
        }

        std::reverse(reversed_path.begin(), reversed_path.end());
        return reversed_path;
    }

    int pickLookaheadCell(
        const nav_msgs::msg::OccupancyGrid& map,
        const SearchData& search,
        const std::vector<int>& path,
        bool relaxed) const
    {
        if (path.empty()) {
            return -1;
        }

        const int width = static_cast<int>(map.info.width);
        const double lookahead = relaxed ? RELAXED_LOOKAHEAD_DISTANCE : LOOKAHEAD_DISTANCE;
        double accumulated = 0.0;
        int anchor_index = path.back();

        for (std::size_t i = 1; i < path.size(); ++i) {
            const int prev = path[i - 1];
            const int current = path[i];

            const int prev_mx = prev % width;
            const int prev_my = prev / width;
            const int current_mx = current % width;
            const int current_my = current / width;

            accumulated += std::hypot(
                (current_mx - prev_mx) * map.info.resolution,
                (current_my - prev_my) * map.info.resolution
            );

            if (accumulated >= lookahead) {
                anchor_index = current;
                break;
            }
        }

        const int anchor_mx = anchor_index % width;
        const int anchor_my = anchor_index / width;
        const int search_cells = std::max(
            1,
            static_cast<int>(std::ceil(CENTER_SNAP_RADIUS / map.info.resolution))
        );

        bool found = false;
        int best_index = anchor_index;
        double best_score = -std::numeric_limits<double>::infinity();

        for (int dy = -search_cells; dy <= search_cells; ++dy) {
            for (int dx = -search_cells; dx <= search_cells; ++dx) {
                const int mx = anchor_mx + dx;
                const int my = anchor_my + dy;

                if (!isFreeCell(map, mx, my)) {
                    continue;
                }

                const int index = indexOf(map, mx, my);

                if (!std::isfinite(search.cost[index])) {
                    continue;
                }

                const double offset = std::hypot(
                    dx * map.info.resolution,
                    dy * map.info.resolution
                );

                if (offset > CENTER_SNAP_RADIUS) {
                    continue;
                }

                if (std::abs(search.cost[index] - search.cost[anchor_index]) > 0.35) {
                    continue;
                }

                const double wx = mapToWorldX(map, mx);
                const double wy = mapToWorldY(map, my);

                if (isBlacklisted(wx, wy)) {
                    continue;
                }

                const double score =
                    (5.0 * search.clearance[index]) +
                    (2.0 * search.bottleneck[index]) +
                    (0.5 * forwardProjection(wx, wy)) -
                    (1.0 * offset);

                if (!found || score > best_score) {
                    found = true;
                    best_score = score;
                    best_index = index;
                }
            }
        }

        return best_index;
    }

    bool makeCenterlineGoal(
        const nav_msgs::msg::OccupancyGrid& map,
        bool relaxed,
        geometry_msgs::msg::PoseStamped& goal)
    {
        SearchData search;

        if (!buildCenterBiasedSearch(map, relaxed, search)) {
            return false;
        }

        int target_index = -1;
        bool selected_frontier = false;

        if (!selectTargetCell(map, search, relaxed, target_index, selected_frontier)) {
            return false;
        }

        const std::vector<int> path = reconstructPath(search, target_index);

        if (path.size() < 2) {
            return false;
        }

        const int goal_index = pickLookaheadCell(map, search, path, relaxed);

        if (goal_index < 0) {
            return false;
        }

        const int width = static_cast<int>(map.info.width);
        const int goal_mx = goal_index % width;
        const int goal_my = goal_index / width;
        const double goal_x = mapToWorldX(map, goal_mx);
        const double goal_y = mapToWorldY(map, goal_my);

        const int target_mx = target_index % width;
        const int target_my = target_index / width;
        const double target_x = mapToWorldX(map, target_mx);
        const double target_y = mapToWorldY(map, target_my);
        const double yaw = std::atan2(target_y - goal_y, target_x - goal_x);

        if (isBlacklisted(goal_x, goal_y)) {
            return false;
        }

        goal.header.frame_id = "map";
        goal.header.stamp = this->now();
        goal.pose.position.x = goal_x;
        goal.pose.position.y = goal_y;
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = std::sin(yaw * 0.5);
        goal.pose.orientation.w = std::cos(yaw * 0.5);

        current_goal_x_ = goal_x;
        current_goal_y_ = goal_y;
        current_target_x_ = target_x;
        current_target_y_ = target_y;

        RCLCPP_INFO(
            this->get_logger(),
            "centerline goal: x=%.2f y=%.2f target=%s relaxed=%s clearance=%.2f bottleneck=%.2f cost=%.2f",
            current_goal_x_,
            current_goal_y_,
            selected_frontier ? "frontier" : "known",
            relaxed ? "true" : "false",
            search.clearance[goal_index],
            search.bottleneck[goal_index],
            search.cost[goal_index]
        );

        return true;
    }

    bool trySendNextGoal()
    {
        if (!latest_map_ || goal_in_progress_ || canceling_goal_) {
            return false;
        }

        if (!updateRobotPose()) {
            return false;
        }

        updateProgress(*latest_map_);

        geometry_msgs::msg::PoseStamped next_goal;

        if (!makeCenterlineGoal(*latest_map_, false, next_goal) &&
            !makeCenterlineGoal(*latest_map_, true, next_goal)) {
            RCLCPP_WARN(this->get_logger(), "centerline: 선택 가능한 중앙 경로 goal 없음");
            return false;
        }

        sendGoal(next_goal);
        return true;
    }

    void cancelCurrentGoalAndBlacklist()
    {
        blacklistCurrentGoalAndTarget();
        canceling_goal_ = true;

        if (active_goal_handle_) {
            nav_client_->async_cancel_goal(active_goal_handle_);
        } else {
            goal_in_progress_ = false;
            canceling_goal_ = false;
        }
    }

    void sendGoal(const geometry_msgs::msg::PoseStamped& goal_pose)
    {
        if (!nav_client_->wait_for_action_server(std::chrono::seconds(3))) {
            RCLCPP_WARN(this->get_logger(), "centerline: Nav2 action server 대기 실패");
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
                    RCLCPP_WARN(this->get_logger(), "centerline: Nav2 goal 거부");
                    blacklistCurrentGoalAndTarget();
                    goal_in_progress_ = false;
                    active_goal_handle_.reset();
                    trySendNextGoal();
                    return;
                }

                active_goal_handle_ = goal_handle;
            };

        send_goal_options.result_callback =
            std::bind(&MazeExplorerCenterline::resultCallback, this, _1);

        goal_in_progress_ = true;
        canceling_goal_ = false;
        best_goal_distance_ = std::hypot(current_goal_x_ - robot_x_, current_goal_y_ - robot_y_);
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

        if (latest_map_) {
            updateProgress(*latest_map_);
        }

        if (isBacktrackingTowardEntrance()) {
            RCLCPP_WARN(this->get_logger(), "centerline: 되감기 watchdog 취소");
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
            "centerline: 목표 접근 실패. 취소: x=%.2f, y=%.2f, 남은거리=%.2f",
            current_goal_x_,
            current_goal_y_,
            goal_distance
        );

        cancelCurrentGoalAndBlacklist();
    }

    void resultCallback(const GoalHandleNav2::WrappedResult& result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(
                this->get_logger(),
                "centerline: waypoint 도착 x=%.2f y=%.2f",
                current_goal_x_,
                current_goal_y_
            );
            blacklist_.push_back({current_goal_x_, current_goal_y_});
        } else if (canceling_goal_) {
            RCLCPP_WARN(this->get_logger(), "centerline: 목표 취소 완료");
        } else {
            RCLCPP_WARN(
                this->get_logger(),
                "centerline: 목표 실패. waypoint/target blacklist 추가 x=%.2f y=%.2f",
                current_goal_x_,
                current_goal_y_
            );
            blacklistCurrentGoalAndTarget();
        }

        active_goal_handle_.reset();
        canceling_goal_ = false;
        goal_in_progress_ = false;
        trySendNextGoal();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MazeExplorerCenterline>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
