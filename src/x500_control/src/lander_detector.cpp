// Finds the lander in the downward camera image and publishes its pixel position.
//
// No simulator information is used: the terrain is almost one colour, so the image's
// median chroma is the background, and anything whose chroma differs clearly from it
// (white, grey, blue or orange hardware) and has a plausible size is a candidate.
#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class LanderDetector : public rclcpp::Node
{
public:
  LanderDetector()
  : rclcpp::Node(
      "lander_detector",
      rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true))
  {
    chroma_thresh_ = pd("chroma_thresh", 16.0);
    min_area_ = pd("min_blob_area", 25.0);
    max_area_ = pd("max_blob_area", 20000.0);
    downscale_ = std::max(1, pi("downscale", 2));
    border_ = pd("ignore_border", 0.05);
    bg_min_a_ = pd("bg_min_a", 3.0);
    bg_min_b_ = pd("bg_min_b", 8.0);
    bg_min_frac_ = pd("bg_min_fraction", 0.15);
    period_ = 1.0 / std::max(0.1, pd("process_hz", 10.0));
    debug_dir_ = ps("debug_dir", "");
    if (!debug_dir_.empty()) {std::filesystem::create_directories(debug_dir_);}

    sub_ = create_subscription<sensor_msgs::msg::Image>(
      ps("image_topic", "/x500/down_camera"), rclcpp::SensorDataQoS(),
      std::bind(&LanderDetector::on_image, this, std::placeholders::_1));
    pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/x500/detection", 10);
    pub_dbg_ = create_publisher<sensor_msgs::msg::Image>("/x500/detection_debug", 1);
    pub_mask_ = create_publisher<sensor_msgs::msg::Image>("/x500/detection_mask", 1);
    RCLCPP_INFO(get_logger(), "Detector waiting for images on %s", sub_->get_topic_name());
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

  static double median(std::vector<uchar> v)
  {
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    return v[v.size() / 2];
  }

  void on_image(sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    const rclcpp::Time t(msg->header.stamp);
    if ((t - last_).seconds() < period_) {return;}
    last_ = t;
    chroma_thresh_ = get_parameter("chroma_thresh").as_double();
    min_area_ = get_parameter("min_blob_area").as_double();
    max_area_ = get_parameter("max_blob_area").as_double();
    ++frames_;

    cv::Mat bgr;
    try {
      bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge: %s", e.what());
      return;
    }
    cv::Mat small;
    cv::resize(bgr, small, cv::Size(), 1.0 / downscale_, 1.0 / downscale_, cv::INTER_AREA);
    cv::Mat lab;
    cv::cvtColor(small, lab, cv::COLOR_BGR2Lab);
    std::vector<cv::Mat> ch;
    cv::split(lab, ch);

    // Background chroma = median over the terrain-coloured pixels (reddish-brown: a and b both
    // positive). Sky, the edge of the map and other void would otherwise drag a plain median away
    // from the ground colour and make the whole ground look like an object.
    std::vector<uchar> sa, sb, ta, tb;
    for (int y = 0; y < lab.rows; y += 4) {
      for (int x = 0; x < lab.cols; x += 4) {
        const uchar a = ch[1].at<uchar>(y, x), b = ch[2].at<uchar>(y, x);
        sa.push_back(a);
        sb.push_back(b);
        if (a > 128 + bg_min_a_ && b > 128 + bg_min_b_) {ta.push_back(a); tb.push_back(b);}
      }
    }
    const double ground_frac = static_cast<double>(ta.size()) / std::max<size_t>(1, sa.size());
    const bool have_ground = ground_frac >= bg_min_frac_;
    const double a0 = median(have_ground ? ta : sa), b0 = median(have_ground ? tb : sb);

    cv::Mat fa, fb, dist;
    ch[1].convertTo(fa, CV_32F, 1.0, -a0);
    ch[2].convertTo(fb, CV_32F, 1.0, -b0);
    cv::magnitude(fa, fb, dist);
    cv::Mat mask = dist > chroma_thresh_;
    mask.setTo(0, (ch[1] < 126) & (ch[2] > 136));          // void / sky
    const int bx = static_cast<int>(border_ * mask.cols), by = static_cast<int>(border_ * mask.rows);
    mask.rowRange(0, by).setTo(0);
    mask.rowRange(mask.rows - by, mask.rows).setTo(0);
    mask.colRange(0, bx).setTo(0);
    mask.colRange(mask.cols - bx, mask.cols).setTo(0);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, {7, 7}));

    cv::Mat labels, stats, cents;
    const int n = cv::connectedComponentsWithStats(mask, labels, stats, cents);
    int best = -1;
    double best_area = 0, too_small = 0, too_big = 0;
    for (int i = 1; i < n; ++i) {
      const double area = stats.at<int>(i, cv::CC_STAT_AREA);
      if (area >= min_area_ && area <= max_area_) {
        if (area > best_area) {best = i; best_area = area;}
      } else if (area < min_area_) {
        too_small = std::max(too_small, area);
      } else {
        too_big = std::max(too_big, area);
      }
    }
    // heartbeat: tells "no images" from "images but nothing found"
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "%d frames, ground %.0f%% of view (bg chroma %.0f, %.0f), largest blob below min %.0f px, above max %.0f px, %s",
      frames_, 100 * ground_frac, a0 - 128, b0 - 128, too_small, too_big, best >= 0 ? "BLOB FOUND" : "no blob in range");

    if (best >= 0) {
      geometry_msgs::msg::PointStamped out;
      out.header = msg->header;
      out.point.x = cents.at<double>(best, 0) * downscale_ + 0.5 * (downscale_ - 1);   // full-res pixels
      out.point.y = cents.at<double>(best, 1) * downscale_ + 0.5 * (downscale_ - 1);
      out.point.z = best_area * downscale_ * downscale_;                               // area in px
      pub_->publish(out);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000, "blob at pixel (%.0f, %.0f), %.0f px, background chroma (%.0f, %.0f)",
        out.point.x, out.point.y, out.point.z, a0 - 128, b0 - 128);
    }

    const bool want_dbg = pub_dbg_->get_subscription_count() > 0;
    if (pub_mask_->get_subscription_count() > 0) {
      cv::Mat big;
      cv::resize(mask, big, bgr.size(), 0, 0, cv::INTER_NEAREST);
      pub_mask_->publish(*cv_bridge::CvImage(msg->header, "mono8", big).toImageMsg());
    }
    if (want_dbg || (!debug_dir_.empty() && (best >= 0 || ++frame_ % 100 == 0))) {
      cv::Mat vis = bgr.clone();
      for (int i = 1; i < n; ++i) {
        const int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area < 5) {continue;}
        const cv::Rect r(
          stats.at<int>(i, cv::CC_STAT_LEFT) * downscale_, stats.at<int>(i, cv::CC_STAT_TOP) * downscale_,
          stats.at<int>(i, cv::CC_STAT_WIDTH) * downscale_, stats.at<int>(i, cv::CC_STAT_HEIGHT) * downscale_);
        const cv::Scalar col = i == best ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 255, 255);
        cv::rectangle(vis, r, col, i == best ? 3 : 1);
        cv::putText(
          vis, std::to_string(area), r.tl() + cv::Point(0, -4), cv::FONT_HERSHEY_SIMPLEX, 0.5, col, 1);
      }
      cv::putText(
        vis, "min " + std::to_string(static_cast<int>(min_area_)) + "  chroma>" +
        std::to_string(static_cast<int>(chroma_thresh_)), {10, 24}, cv::FONT_HERSHEY_SIMPLEX, 0.7,
        cv::Scalar(255, 255, 255), 2);
      if (want_dbg) {
        pub_dbg_->publish(*cv_bridge::CvImage(msg->header, "bgr8", vis).toImageMsg());
      }
      if (!debug_dir_.empty()) {
        const std::string stem = debug_dir_ + "/" + std::to_string(t.nanoseconds());
        cv::imwrite(stem + (best >= 0 ? "_hit.png" : ".png"), vis);
      }
    }
  }

  double bg_min_a_{3}, bg_min_b_{8}, bg_min_frac_{0.15};
  double chroma_thresh_{14}, min_area_{40}, max_area_{20000}, border_{0.05}, period_{0.1};
  int downscale_{2}, frame_{0}, frames_{0};
  std::string debug_dir_;
  rclcpp::Time last_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_dbg_, pub_mask_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LanderDetector>());
  rclcpp::shutdown();
  return 0;
}
