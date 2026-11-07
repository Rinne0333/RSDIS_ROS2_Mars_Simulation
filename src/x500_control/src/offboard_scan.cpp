#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/distance_sensor.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr uint16_t CMD_ARM = 400;
constexpr uint16_t CMD_SET_MODE = 176;
constexpr uint8_t ARMED = 2;
constexpr uint8_t OFFBOARD = 14;

struct Vec3
{
  double n{0}, e{0}, d{0};
};

Vec3 world_to_ned(double wx, double wy, double wz, double hx, double hy, double hz)
{
  return {wy - hy, wx - hx, -(wz - hz)};
}

void ned_to_world(
  double n, double e, double d, double hx, double hy, double hz,
  double & wx, double & wy, double & wz)
{
  wx = hx + e;
  wy = hy + n;
  wz = hz - d;
}

std::vector<std::pair<double, double>> lawnmower(
  double cx, double cy, double length, double width, double spacing)
{
  std::vector<std::pair<double, double>> points;
  const double x_lo = cx - 0.5 * length;
  const double x_hi = cx + 0.5 * length;
  const double y_hi = cy + 0.5 * width;
  bool flip = false;
  for (double y = cy - 0.5 * width; y <= y_hi + 1e-6; y += spacing) {
    std::pair<double, double> a{x_lo, y};
    std::pair<double, double> b{x_hi, y};
    if (flip) {
      points.push_back(b);
      points.push_back(a);
    } else {
      points.push_back(a);
      points.push_back(b);
    }
    flip = !flip;
  }
  return points;
}
}  // namespace

class OffboardScan : public rclcpp::Node
{
public:
  OffboardScan()
  : rclcpp::Node(
      "offboard_scan",
      rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true))
  {
    hx_ = pd("spawn_x", 8.0);
    hy_ = pd("spawn_y", -2.0);
    hz_ = pd("spawn_z", 4.5);
    const double alt = pd("scan_altitude", 10.0);
    climb_ = world_to_ned(hx_, hy_, alt, hx_, hy_, hz_);
    radius_ = pd("waypoint_radius", 0.8);
    margin_ = pd("anomaly_margin", 0.45);
    need_hits_ = pi("anomaly_hits", 3);
    stop_on_detect_ = pb("stop_on_detect", false);
    detect_radius_ = pd("target_detect_radius", 6.0);

    for (const auto & xy : lawnmower(
        hx_, hy_, pd("scan_length", 12.0), pd("scan_width", 12.0), pd("lane_spacing", 4.0)))
    {
      waypoints_.push_back(world_to_ned(xy.first, xy.second, alt, hx_, hy_, hz_));
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    sub_pos_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      ps("vehicle_local_position", "/fmu/out/vehicle_local_position"), qos,
      [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {pos_ = *msg;});
    sub_status_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      ps("vehicle_status", "/fmu/out/vehicle_status"), qos,
      [this](px4_msgs::msg::VehicleStatus::SharedPtr msg) {status_ = *msg;});
    sub_range_ = create_subscription<px4_msgs::msg::DistanceSensor>(
      ps("distance_sensor", "/fmu/out/distance_sensor"), qos,
      std::bind(&OffboardScan::on_range, this, std::placeholders::_1));
    sub_target_ = create_subscription<nav_msgs::msg::Odometry>(
      ps("target_odom_topic", "/target/odometry"), 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        target_ = std::array<double, 3>{
          msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
      });

    pub_mode_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      ps("offboard_control_mode", "/fmu/in/offboard_control_mode"), qos);
    pub_sp_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      ps("trajectory_setpoint", "/fmu/in/trajectory_setpoint"), qos);
    pub_cmd_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      ps("vehicle_command", "/fmu/in/vehicle_command"), qos);
    pub_poi_ = create_publisher<geometry_msgs::msg::PointStamped>("/x500/poi", 10);

    setpoint_ = climb_;
    phase_since_ = now();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&OffboardScan::tick, this));
    RCLCPP_INFO(
      get_logger(), "Scan ready: %zu waypoints, home ENU (%.1f, %.1f, %.1f), altitude %.1f m",
      waypoints_.size(), hx_, hy_, hz_, alt);
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
  bool pb(const std::string & name, bool fallback)
  {
    if (!has_parameter(name)) {declare_parameter(name, fallback);}
    return get_parameter(name).as_bool();
  }
  std::string ps(const std::string & name, const std::string & fallback)
  {
    if (!has_parameter(name)) {declare_parameter(name, fallback);}
    return get_parameter(name).as_string();
  }

  uint64_t now_us() const {return static_cast<uint64_t>(now().nanoseconds() / 1000);}

  void on_range(px4_msgs::msg::DistanceSensor::SharedPtr msg)
  {
    if (phase_ != "scan" || !pos_) {return;}
    const double dist = msg->current_distance;
    if (dist <= 0.05 || dist > 80.0) {return;}
    ranges_.push_back(dist);
    if (ranges_.size() > 25) {ranges_.erase(ranges_.begin());}
    if (ranges_.size() < 8) {return;}
    auto ordered = ranges_;
    std::sort(ordered.begin(), ordered.end());
    const double median = ordered[ordered.size() / 2];
    const double deficit = median - dist;
    hit_run_ = deficit > margin_ ? hit_run_ + 1 : 0;
    if (hit_run_ >= need_hits_) {
      mark_poi(deficit, "lidar", std::nullopt);
      hit_run_ = 0;
    }
  }

  void mark_poi(double score, const char * source, std::optional<std::array<double, 3>> world)
  {
    double wx, wy, wz;
    if (!world) {
      ned_to_world(pos_->x, pos_->y, pos_->z, hx_, hy_, hz_, wx, wy, wz);
      wz -= ranges_.empty() ? 0.0 : ranges_.back();
    } else {
      wx = (*world)[0];
      wy = (*world)[1];
      wz = (*world)[2];
    }
    if (!best_ || score > std::get<0>(*best_)) {
      best_ = std::make_tuple(score, wx, wy, wz);
      RCLCPP_INFO(get_logger(), "POI candidate (%s) world (%.2f, %.2f) score %.2f", source, wx, wy, score);
    }
    if (stop_on_detect_ && phase_ == "scan") {
      phase_ = "done";
      setpoint_ = {pos_->x, pos_->y, climb_.d};
      publish_poi();
    }
  }

  void publish_poi()
  {
    if (poi_sent_ || !best_) {return;}
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "world";
    msg.point.x = std::get<1>(*best_);
    msg.point.y = std::get<2>(*best_);
    msg.point.z = std::get<3>(*best_);
    pub_poi_->publish(msg);
    poi_sent_ = true;
    RCLCPP_INFO(
      get_logger(), "Published /x500/poi at (%.2f, %.2f, %.2f)",
      msg.point.x, msg.point.y, msg.point.z);
  }

  void command(uint16_t command, float p1 = 0.f, float p2 = 0.f)
  {
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = now_us();
    msg.command = command;
    msg.param1 = p1;
    msg.param2 = p2;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    pub_cmd_->publish(msg);
  }

  void stream()
  {
    const uint64_t stamp = now_us();
    px4_msgs::msg::OffboardControlMode mode{};
    mode.timestamp = stamp;
    mode.position = true;
    pub_mode_->publish(mode);

    px4_msgs::msg::TrajectorySetpoint sp{};
    sp.timestamp = stamp;
    sp.position = {
      static_cast<float>(setpoint_.n),
      static_cast<float>(setpoint_.e),
      static_cast<float>(setpoint_.d)};
    sp.yaw = 0.f;
    pub_sp_->publish(sp);
  }

  bool armed() const {return status_ && status_->arming_state == ARMED;}
  bool offboard() const {return status_ && status_->nav_state == OFFBOARD;}
  double age() const {return (now() - phase_since_).seconds();}

  void go(const char * phase)
  {
    phase_ = phase;
    phase_since_ = now();
    RCLCPP_INFO(get_logger(), "phase -> %s", phase);
  }

  bool near(const Vec3 & target) const
  {
    const double xy = std::hypot(pos_->x - target.n, pos_->y - target.e);
    return xy < radius_ && std::abs(pos_->z - target.d) < std::max(radius_, 0.6);
  }

  void see_target()
  {
    if (!target_ || !pos_ || phase_ != "scan") {return;}
    double wx, wy, wz;
    ned_to_world(pos_->x, pos_->y, pos_->z, hx_, hy_, hz_, wx, wy, wz);
    if (std::hypot(wx - (*target_)[0], wy - (*target_)[1]) < detect_radius_) {
      mark_poi(10.0, "target_odom", target_);
    }
  }

  void tick()
  {
    stream();
    see_target();
    if (!pos_ || !pos_->z_valid) {return;}

    if (phase_ == "wait_pose") {
      setpoint_ = climb_;
      if (++offboard_ticks_ > 40) {go("arm");}
      return;
    }
    if (phase_ == "arm") {
      if (armed()) {go("offboard"); return;}
      if (age() > 1.0) {
        command(CMD_ARM, 1.f);
        phase_since_ = now();
        RCLCPP_INFO(get_logger(), "arm command sent");
      }
      return;
    }
    if (phase_ == "offboard") {
      if (offboard()) {
        setpoint_ = climb_;
        go("climb");
        return;
      }
      if (age() > 1.0) {
        command(CMD_SET_MODE, 1.f, 6.f);
        phase_since_ = now();
        RCLCPP_INFO(get_logger(), "offboard command sent");
      }
      return;
    }
    if (phase_ == "climb") {
      setpoint_ = climb_;
      if (near(climb_)) {go("scan");}
      return;
    }
    if (phase_ == "scan") {
      setpoint_ = waypoints_[wp_index_];
      if (near(waypoints_[wp_index_])) {
        ++wp_index_;
        if (wp_index_ >= waypoints_.size()) {
          go("done");
          setpoint_ = climb_;
          publish_poi();
          if (!best_) {RCLCPP_INFO(get_logger(), "Scan finished with no point of interest");}
        }
      }
      return;
    }
    if (phase_ == "done") {
      setpoint_ = {pos_->x, pos_->y, climb_.d};
      publish_poi();
    }
  }

  double hx_{}, hy_{}, hz_{}, radius_{}, margin_{}, detect_radius_{};
  int need_hits_{3}, hit_run_{0}, offboard_ticks_{0};
  bool stop_on_detect_{false}, poi_sent_{false};
  size_t wp_index_{0};
  Vec3 climb_{}, setpoint_{};
  std::vector<Vec3> waypoints_;
  std::vector<double> ranges_;
  std::optional<px4_msgs::msg::VehicleLocalPosition> pos_;
  std::optional<px4_msgs::msg::VehicleStatus> status_;
  std::optional<std::array<double, 3>> target_;
  std::optional<std::tuple<double, double, double, double>> best_;
  std::string phase_{"wait_pose"};
  rclcpp::Time phase_since_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_pos_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status_;
  rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::SharedPtr sub_range_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_target_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_mode_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_sp_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_poi_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardScan>());
  rclcpp::shutdown();
  return 0;
}
