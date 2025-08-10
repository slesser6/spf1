#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cmath>
#include <queue>
#include <vector>
/**
 * @class PathNode
 *
 * @brief Node that subscribes to a stereo depth image and a sun pose, and
 * publishes a planned path.
 */
class PathNode : public rclcpp::Node {
public:
  PathNode() : rclcpp::Node("path_node") {
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/stereo/depth", 10,
        std::bind(&PathNode::onDepth, this, std::placeholders::_1));

    sun_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/sun_pos", 10,
        std::bind(&PathNode::onSunPos, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
    RCLCPP_INFO(get_logger(), "PathNode started");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sun_pos_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  cv::Mat current_depth_map_;
  geometry_msgs::msg::PoseStamped::SharedPtr current_sun_pos_;

  /**
   * @brief Callback for receiving depth images.
   * @param msg Incoming depth image message.
   **/
  void onDepth(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
      current_depth_map_ = cv_ptr->image;

      if (current_sun_pos_) {
        computePath(*current_sun_pos_);
      }
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }
  /**
   * @brief Callback for receiving sun position messages.
   * @param msg Incoming sun pos message.
   **/
  void onSunPos(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_sun_pos_ = msg;
    if (!current_depth_map_.empty()) {
      computePath(*msg);
    }
  }
  /**
   * @brief Computes and publishes a path using A* algorithm.
   * @param goal The target pose to plan a path to.
   **/
  void computePath(const geometry_msgs::msg::PoseStamped &goal) {
    const int rows = current_depth_map_.rows;
    const int cols = current_depth_map_.cols;

    std::vector<std::vector<int>> grid(rows, std::vector<int>(cols, 0));
    for (int y = 0; y < rows; ++y) {
      for (int x = 0; x < cols; ++x) {
        float depth = current_depth_map_.at<float>(y, x);
        grid[y][x] = (depth > 0.5f && depth < 4.0f) ? 0 : 1;
      }
    }

    int start_x = cols / 2; // Starting x-coordinate(center)
    int start_y = rows - 1; // Starting y-coordinate(bottom)

    int goal_x =
        std::clamp(static_cast<int>(goal.pose.position.x), 0, cols - 1);
    int goal_y =
        std::clamp(static_cast<int>(goal.pose.position.y), 0, rows - 1);

    auto path_pixels = a_star(grid, start_x, start_y, goal_x, goal_y);
    publishPath(path_pixels);
  }
  /**
   * @struct Node
   * @brief Represents a cell in the A* search.
   **/
  struct Node {
    int x, y;
    float cost, priority;
    Node *parent;
    bool operator>(const Node &other) const {
      return priority > other.priority;
    }
  };
  /**
   * @brief Heuristic function for A* (Euclidean distance).
   * @param x1 x.
   * @param y1 y.
   * @param x2 Goal x.
   * @param y2 Goal y.
   **/
  float heuristic(int x1, int y1, int x2, int y2) {
    return std::hypot(x1 - x2, y1 - y2);
  }
  /**
   * @brief A* pathfinding algorithm.
   * @param grid Binary occupancy grid.
   * @param sx Start x.
   * @param sy Start y.
   * @param gx Goal x.
   * @param gy Goal y.
   * @return List of coordinates (x,y) for the path.
   **/
  std::vector<std::pair<int, int>>
  a_star(const std::vector<std::vector<int>> &grid, int sx, int sy, int gx,
         int gy) {
    int rows = grid.size(), cols = grid[0].size();
    std::vector<std::vector<bool>> visited(rows,
                                           std::vector<bool>(cols, false));

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
    pq.push({sx, sy, 0.0f, 0.0f, nullptr});

    while (!pq.empty()) {
      Node current = pq.top();
      pq.pop();

      if (current.x == gx && current.y == gy) {
        std::vector<std::pair<int, int>> path;
        for (Node *n = &current; n != nullptr; n = n->parent)
          path.emplace_back(n->x, n->y);
        std::reverse(path.begin(), path.end());
        return path;
      }

      if (visited[current.y][current.x])
        continue;
      visited[current.y][current.x] = true;

      for (auto [dx, dy] :
           std::vector<std::pair<int, int>>{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) {
        int nx = current.x + dx, ny = current.y + dy;
        if (nx >= 0 && ny >= 0 && nx < cols && ny < rows && !visited[ny][nx] &&
            grid[ny][nx] == 0) {
          float new_cost = current.cost + 1.0f;
          float priority = new_cost + heuristic(nx, ny, gx, gy);
          pq.push({nx, ny, new_cost, priority, new Node(current)});
        }
      }
    }

    return {};
  }
  /**
   * @brief Publishes the computed path as a nav_msgs::msg::Path message.
   * @param pixels List of (x,y) pixel coordinates on the image.
   **/
  void publishPath(const std::vector<std::pair<int, int>> &pixels) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = "map";

    for (auto &[x, y] : pixels) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);
  }
};

/**
 * @brief Main function for the path node.
 *
 * Initializes ROS 2, creates the PathNode, spins it, and shuts down ROS 2.
 *
 * @param argc Argument count from command line.
 * @param argv Argument vector from command line.
 * @return int Exit status code.
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathNode>());
  rclcpp::shutdown();
  return 0;
}
