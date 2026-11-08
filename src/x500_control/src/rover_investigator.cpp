#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "x500_control/terrain.hpp"

namespace
{
double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny, cosy);
}

double wrap(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

}  // namespace

class RoverInvestigator : public rclcpp::Node
{
public:
  RoverInvestigator()
  : rclcpp::Node(
      "rover_investigator",
      rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true))
  {
    dem_file_ = ps("dem_file", "");
    if (!dem_.load(dem_file_)) {
      RCLCPP_FATAL(get_logger(), "Cannot read the terrain grid '%s' (parameter dem_file)", dem_file_.c_str());
      throw std::runtime_error("terrain grid missing");
    }
    max_slope_ = pd("max_slope_deg", 20.0);
    inflate_ = pi("inflate_cells", 1);
    standoff_ = pd("standoff", 2.5);
    speed_ = pd("rover_speed", 0.35);
    yaw_gain_ = pd("rover_yaw_gain", 1.2);

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      ps("rover_odom_topic", "/perseverance/odometry"), 10,
      std::bind(&RoverInvestigator::on_odom, this, std::placeholders::_1));
    sub_poi_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/x500/poi", 10, std::bind(&RoverInvestigator::on_poi, this, std::placeholders::_1));
    pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>(
      ps("rover_cmd_topic", "/perseverance/cmd_vel"), 10);
    pub_path_ = create_publisher<nav_msgs::msg::Path>("/x500/rover_path", 10);
    pub_status_ = create_publisher<std_msgs::msg::String>("/x500/mission_status", 10);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&RoverInvestigator::tick, this));
    RCLCPP_INFO(get_logger(), "Waiting for /x500/poi and rover odometry");
  }

private:
  double pd(const std::string & name, double fallback)
  {
    if (!has_parameter(name)) {declare_parameter(name, fallback);}
    return get_parameter(name).as_double();
  }
  int pi(const std::string & name, int fallback)
  {
    if (!has_parameter(name)) {declare_parameter(name, fallback);}
    return static_cast<int>(get_parameter(name).as_int());
  }
  std::string ps(const std::string & name, const std::string & fallback)
  {
    if (!has_parameter(name)) {declare_parameter(name, fallback);}
    return get_parameter(name).as_string();
  }

  void status(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    pub_status_->publish(msg);
    RCLCPP_INFO(get_logger(), "%s", text.c_str());
  }

  void on_odom(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    pose_ = std::array<double, 3>{
      msg->pose.pose.position.x, msg->pose.pose.position.y, yaw_of(msg->pose.pose.orientation)};
  }

  void on_poi(geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (!waypoints_.empty()) {return;}
    if (!pose_) {
      poi_ = std::array<double, 2>{msg->point.x, msg->point.y};
      return;
    }
    plan(msg->point.x, msg->point.y);
  }

  void plan(double px, double py)
  {
    const XY start{(*pose_)[0], (*pose_)[1]};
    if (!dem_.contains(px, py)) {
      status("Point of interest is outside the terrain map");
      return;
    }
    if (std::hypot(start.first - px, start.second - py) < standoff_ + 0.5) {
      status("Rover is already at the point of interest");
      return;
    }
    const auto pts = dem_.plan(start, {px, py}, max_slope_, standoff_, inflate_);
    if (pts.size() < 2) {
      status("No path under the slope limit (raise max_slope_deg or lower inflate_cells)");
      return;
    }
    waypoints_.assign(pts.begin() + 1, pts.end());

    nav_msgs::msg::Path path;
    path.header.frame_id = "world";
    path.header.stamp = now();
    for (const auto & p : pts) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = p.first;
      pose.pose.position.y = p.second;
      pose.pose.position.z = dem_.height(p.first, p.second);
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    pub_path_->publish(path);
    RCLCPP_INFO(
      get_logger(), "Path planned: %zu legs, %.0f m (straight line %.0f m), goal (%.1f, %.1f)",
      waypoints_.size(), x500::Dem::length(pts), std::hypot(start.first - px, start.second - py),
      pts.back().first, pts.back().second);
    status("Path planned");
  }

  void tick()
  {
    if (poi_ && pose_ && waypoints_.empty()) {
      plan((*poi_)[0], (*poi_)[1]);
      poi_.reset();
    }
    geometry_msgs::msg::Twist cmd;
    if (waypoints_.empty() || !pose_) {
      pub_cmd_->publish(cmd);
      return;
    }
    const double gx = waypoints_.front().first;
    const double gy = waypoints_.front().second;
    const double dx = gx - (*pose_)[0];
    const double dy = gy - (*pose_)[1];
    const double dist = std::hypot(dx, dy);
    if (dist < 0.45) {
      waypoints_.erase(waypoints_.begin());
      if (waypoints_.empty()) {status("Rover arrived at the investigation standoff");}
      pub_cmd_->publish(cmd);
      return;
    }
    const double err = wrap(std::atan2(dy, dx) - (*pose_)[2]);
    if (std::abs(err) > 0.5) {
      cmd.angular.z = std::clamp(yaw_gain_ * err, -0.6, 0.6);
    } else {
      cmd.linear.x = std::min(speed_, 0.6 * dist);
      cmd.angular.z = std::clamp(yaw_gain_ * err, -0.6, 0.6);
    }
    pub_cmd_->publish(cmd);
  }

  using XY = x500::XY;
  x500::Dem dem_;
  std::string dem_file_;
  double max_slope_{20.0};
  int inflate_{1};
  double standoff_{}, speed_{}, yaw_gain_{};
  std::optional<std::array<double, 3>> pose_;
  std::optional<std::array<double, 2>> poi_;
  std::vector<std::pair<double, double>> waypoints_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_poi_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoverInvestigator>());
  rclcpp::shutdown();
  return 0;
}
