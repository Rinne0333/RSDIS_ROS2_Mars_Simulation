// Turns the detector's pixel position into a world position by casting the pixel's
// ray against the terrain grid, using the drone pose from PX4. A point is only
// published once several detections land on the same spot, which also removes
// one-frame false alarms.
#include <array>
#include <cmath>
#include <deque>
#include <optional>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "x500_control/terrain.hpp"

class PoiLocator : public rclcpp::Node
{
public:
  PoiLocator()
  : rclcpp::Node(
      "poi_locator",
      rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true))
  {
    const std::string dem_file = ps("dem_file", "");
    if (!dem_.load(dem_file)) {
      RCLCPP_FATAL(get_logger(), "Cannot read the terrain grid '%s' (parameter dem_file)", dem_file.c_str());
      throw std::runtime_error("terrain grid missing");
    }
    hx_ = pd("spawn_x", 8.0);
    hy_ = pd("spawn_y", -2.0);
    hz_ = pd("spawn_z", 3.8);
    width_ = pd("image_width", 1280.0);
    height_ = pd("image_height", 960.0);
    const double hfov = pd("camera_hfov", 1.74);
    fx_ = fy_ = 0.5 * width_ / std::tan(0.5 * hfov);
    max_range_ = pd("max_ray_range", 120.0);
    hits_needed_ = pi("confirm_hits", 3);
    radius_ = pd("confirm_radius", 5.0);
    max_age_ = pd("max_pose_age", 2.0);
    ignore_ = has_parameter("ignore_blobs") ? get_parameter("ignore_blobs").as_bool() : false;
    size_min_ = pd("object_min_m", 0.8);
    size_max_ = pd("object_max_m", 4.5);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    sub_pos_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      ps("vehicle_local_position", "/fmu/out/vehicle_local_position"), qos,
      [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr m) {pos_ = *m; pos_t_ = now();});
    sub_att_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      ps("vehicle_attitude", "/fmu/out/vehicle_attitude"), qos,
      [this](px4_msgs::msg::VehicleAttitude::SharedPtr m) {att_ = *m; att_t_ = now();});
    sub_det_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/x500/detection", 10, std::bind(&PoiLocator::on_detection, this, std::placeholders::_1));
    pub_poi_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/x500/poi", rclcpp::QoS(10).transient_local());
    pub_blob_ = create_publisher<geometry_msgs::msg::PointStamped>("/x500/blob_world", 10);
    RCLCPP_INFO(
      get_logger(), "Locator ready: fx %.0f px, needs %d detections within %.1f m%s", fx_, hits_needed_,
      radius_, ignore_ ? "  [ignore_blobs: no /x500/poi will be published]" : "");
  }

private:
  double pd(const std::string & n, double f)
  {
    if (!has_parameter(n)) {declare_parameter(n, f);}
    return get_parameter(n).as_double();
  }
  int pi(const std::string & n, int f)
  {
    if (!has_parameter(n)) {declare_parameter(n, f);}
    return static_cast<int>(get_parameter(n).as_int());
  }
  std::string ps(const std::string & n, const std::string & f)
  {
    if (!has_parameter(n)) {declare_parameter(n, f);}
    return get_parameter(n).as_string();
  }

  void on_detection(geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (done_ || !pos_ || !att_) {return;}
    if ((now() - pos_t_).seconds() > max_age_ || (now() - att_t_).seconds() > max_age_) {return;}

    // Camera: looks along body down, image right = body right, image up = body forward (FRD).
    const double u = msg->point.x, v = msg->point.y;
    const std::array<double, 3> db{-(v - 0.5 * height_) / fy_, (u - 0.5 * width_) / fx_, 1.0};

    // FRD -> NED rotation from the quaternion (w, x, y, z)
    const double w = att_->q[0], x = att_->q[1], y = att_->q[2], z = att_->q[3];
    const double r[3][3] = {
      {1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
      {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
      {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}};
    std::array<double, 3> dn{};
    for (int i = 0; i < 3; ++i) {dn[i] = r[i][0] * db[0] + r[i][1] * db[1] + r[i][2] * db[2];}
    if (dn[2] < 0.1) {return;}                                  // ray not pointing down enough

    // NED (north, east, down) -> world ENU, origin at the spawn
    const std::array<double, 3> origin{hx_ + pos_->y, hy_ + pos_->x, hz_ - pos_->z};
    const std::array<double, 3> dir{dn[1], dn[0], -dn[2]};
    const auto hit = dem_.ray_hit(origin, dir, max_range_);
    if (!hit) {return;}

    // The lander has a known size, so its blob must have the pixel area that size gives at this
    // range. Terrain patches and map edges are far too large (or specks far too small).
    const double range = std::sqrt(
      std::pow((*hit)[0] - origin[0], 2) + std::pow((*hit)[1] - origin[1], 2) +
      std::pow((*hit)[2] - origin[2], 2));
    const double px_lo = fx_ * size_min_ / range, px_hi = fx_ * size_max_ / range;
    const double area = msg->point.z;
    const bool fits = area <= px_hi * px_hi && area >= 0.25 * px_lo * px_lo;

    // Every blob that lands on the ground is published for inspection, size-checked or not.
    geometry_msgs::msg::PointStamped blob;
    blob.header.stamp = now();
    blob.header.frame_id = "world";
    blob.point.x = (*hit)[0];
    blob.point.y = (*hit)[1];
    blob.point.z = (*hit)[2];
    pub_blob_->publish(blob);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "blob %.0f px at pixel (%.0f, %.0f), range %.0f m -> ground (%.1f, %.1f, %.1f); a %.1f-%.1f m object is "
      "%.0f-%.0f px here: %s",
      area, u, v, range, (*hit)[0], (*hit)[1], (*hit)[2], size_min_, size_max_, 0.25 * px_lo * px_lo,
      px_hi * px_hi, fits ? "size fits" : "size does not fit");
    if (!fits) {return;}

    recent_.push_back({(*hit)[0], (*hit)[1]});
    if (recent_.size() > 15) {recent_.pop_front();}
    double sx = 0, sy = 0;
    int n = 0;
    for (const auto & p : recent_) {
      if (std::hypot(p.first - (*hit)[0], p.second - (*hit)[1]) <= radius_) {sx += p.first; sy += p.second; ++n;}
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000, "detection -> ground (%.1f, %.1f), %d/%d agreeing",
      (*hit)[0], (*hit)[1], n, hits_needed_);
    if (n < hits_needed_ || ignore_) {return;}

    geometry_msgs::msg::PointStamped out;
    out.header.stamp = now();
    out.header.frame_id = "world";
    out.point.x = sx / n;
    out.point.y = sy / n;
    out.point.z = dem_.height(out.point.x, out.point.y);
    pub_poi_->publish(out);
    done_ = true;
    RCLCPP_INFO(
      get_logger(), "Published /x500/poi at (%.1f, %.1f, %.1f) from %d detections",
      out.point.x, out.point.y, out.point.z, n);
  }

  x500::Dem dem_;
  double hx_{}, hy_{}, hz_{}, width_{}, height_{}, fx_{}, fy_{}, max_range_{}, radius_{}, max_age_{};
  double size_min_{0.8}, size_max_{4.5};
  int hits_needed_{3};
  bool done_{false}, ignore_{false};
  std::deque<std::pair<double, double>> recent_;
  std::optional<px4_msgs::msg::VehicleLocalPosition> pos_;
  std::optional<px4_msgs::msg::VehicleAttitude> att_;
  rclcpp::Time pos_t_{0, 0, RCL_ROS_TIME}, att_t_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_pos_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr sub_att_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_det_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_poi_, pub_blob_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoiLocator>());
  rclcpp::shutdown();
  return 0;
}
