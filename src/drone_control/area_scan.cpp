#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class IngenuityAreaScan : public rclcpp::Node
{
public:
  IngenuityAreaScan()
  : Node("ingenuity_area_scan")
  {
    scan_altitude_ = declare_parameter("scan_altitude", 15.0);
    scan_length_ = declare_parameter("scan_length", 10.0);
    scan_width_ = declare_parameter("scan_width", 10.0);
    lane_spacing_ = declare_parameter("lane_spacing", 2.0);
    arrival_tolerance_ = declare_parameter("arrival_tolerance", 1.0);
    max_waypoint_speed_ = declare_parameter("max_waypoint_speed", 0.25);
    altitude_speed_tolerance_ = declare_parameter("altitude_speed_tolerance", 0.15);

    if (scan_length_ <= 0.0 || scan_width_ <= 0.0 || lane_spacing_ <= 0.0) {
      throw std::invalid_argument("scan_length, scan_width, and lane_spacing must be positive");
    }

    target_publisher_ = create_publisher<geometry_msgs::msg::Point>(
      "/ingenuity/position_target", 10);
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/ingenuity/odometry", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        updateOdometry(*message);
      });
    state_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() { advanceMission(); });
  }

private:
  enum class MissionState
  {
    WaitingForOdometry,
    Climbing,
    Scanning,
    Complete
  };

  void updateOdometry(const nav_msgs::msg::Odometry & message)
  {
    const auto & position = message.pose.pose.position;
    const double stamp = static_cast<double>(message.header.stamp.sec) +
      static_cast<double>(message.header.stamp.nanosec) * 1e-9;
    if (have_odometry_) {
      const double elapsed = stamp - previous_stamp_;
      if (elapsed > 1e-4) {
        velocity_x_ = (position.x - x_) / elapsed;
        velocity_y_ = (position.y - y_) / elapsed;
        velocity_z_ = (position.z - z_) / elapsed;
      }
    }
    x_ = position.x;
    y_ = position.y;
    z_ = position.z;
    previous_stamp_ = stamp;
    have_odometry_ = true;
  }

  void advanceMission()
  {
    if (!have_odometry_ || state_ == MissionState::Complete) {
      return;
    }

    if (state_ == MissionState::WaitingForOdometry) {
      center_x_ = x_;
      center_y_ = y_;
      publishTarget(center_x_, center_y_, scan_altitude_);
      state_ = MissionState::Climbing;
      RCLCPP_INFO(get_logger(),
        "Taking off to z=%.1f m; scan area %.1f x %.1f m, lane spacing %.1f m",
        scan_altitude_, scan_length_, scan_width_, lane_spacing_);
      return;
    }

    if (state_ == MissionState::Climbing) {
      const bool at_altitude = z_ >= scan_altitude_ - arrival_tolerance_ &&
        std::abs(velocity_z_) <= altitude_speed_tolerance_;
      if (!at_altitude) {
        return;
      }
      buildWaypoints();
      state_ = MissionState::Scanning;
      waypoint_index_ = 0;
      publishWaypoint();
      RCLCPP_INFO(get_logger(), "Altitude stable at z=%.2f m; beginning %zu-lane serpentine scan",
        z_, waypoints_.size() / 2);
      return;
    }

    if (state_ == MissionState::Scanning) {
      const auto & waypoint = waypoints_[waypoint_index_];
      const double distance = std::hypot(waypoint.x - x_, waypoint.y - y_);
      const double horizontal_speed = std::hypot(velocity_x_, velocity_y_);
      if (distance > arrival_tolerance_ || horizontal_speed > max_waypoint_speed_) {
        return;
      }

      ++waypoint_index_;
      if (waypoint_index_ == waypoints_.size()) {
        publishTarget(x_, y_, scan_altitude_);
        state_ = MissionState::Complete;
        RCLCPP_INFO(get_logger(),
          "Area scan complete; holding final position x=%.2f y=%.2f z=%.2f m",
          x_, y_, scan_altitude_);
        return;
      }
      publishWaypoint();
      RCLCPP_INFO(get_logger(), "Scan waypoint %zu/%zu reached",
        waypoint_index_, waypoints_.size());
    }
  }

  void buildWaypoints()
  {
    const double x_min = center_x_ - scan_length_ / 2.0;
    const double x_max = center_x_ + scan_length_ / 2.0;
    const double y_min = center_y_ - scan_width_ / 2.0;
    const double y_max = center_y_ + scan_width_ / 2.0;
    const auto lane_count = static_cast<std::size_t>(std::ceil(scan_width_ / lane_spacing_)) + 1;

    waypoints_.clear();
    for (std::size_t lane = 0; lane < lane_count; ++lane) {
      const double y = std::min(y_min + static_cast<double>(lane) * lane_spacing_, y_max);
      if (lane % 2 == 0) {
        waypoints_.push_back({x_min, y});
        waypoints_.push_back({x_max, y});
      } else {
        waypoints_.push_back({x_max, y});
        waypoints_.push_back({x_min, y});
      }
    }
  }

  void publishWaypoint()
  {
    const auto & waypoint = waypoints_[waypoint_index_];
    publishTarget(waypoint.x, waypoint.y, scan_altitude_);
  }

  void publishTarget(double x, double y, double z)
  {
    geometry_msgs::msg::Point target;
    target.x = x;
    target.y = y;
    target.z = z;
    target_publisher_->publish(target);
  }

  struct Waypoint
  {
    double x;
    double y;
  };

  MissionState state_{MissionState::WaitingForOdometry};
  bool have_odometry_{false};
  double x_{0.0};
  double y_{0.0};
  double z_{0.0};
  double velocity_x_{0.0};
  double velocity_y_{0.0};
  double velocity_z_{0.0};
  double previous_stamp_{0.0};
  double center_x_{0.0};
  double center_y_{0.0};
  double scan_altitude_{15.0};
  double scan_length_{10.0};
  double scan_width_{10.0};
  double lane_spacing_{2.0};
  double arrival_tolerance_{1.0};
  double max_waypoint_speed_{0.25};
  double altitude_speed_tolerance_{0.15};
  std::size_t waypoint_index_{0};
  std::vector<Waypoint> waypoints_;

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::TimerBase::SharedPtr state_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IngenuityAreaScan>());
  rclcpp::shutdown();
  return 0;
}
