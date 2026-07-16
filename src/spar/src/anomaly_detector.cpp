// The simplest perception module that can support a behavior: find the
// biggest red blob in the camera image, use the depth image to place it in
// the world, and publish that as a labeled map-frame point on
// perception/detections. The behavior layer never sees pixels — it sees
// "anomaly at (x, y), stamped". Swap this for a YOLO node or a VLM later:
// same topic, same message, just a different label, and the tree doesn't
// change.
//
// Deliberately naive: HSV threshold + largest contour. "Anything red is
// worth a look" is a defensible v1 policy on a site where hazard things
// (cones, drums, spill markers) are red on purpose — but it will happily
// flag a red jacket. That's a feature for a course robot: make it better.

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <spar/msg/detection.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace spar {

class AnomalyDetector : public rclcpp::Node {
public:
  AnomalyDetector() : Node("anomaly_detector") {
    declare_parameter("color_topic", "sensors/camera_0/color/image");
    declare_parameter("depth_topic", "sensors/camera_0/depth/image");
    declare_parameter("info_topic", "sensors/camera_0/color/camera_info");
    declare_parameter("map_frame", "map");
    // The sim's image headers name an optical frame that isn't in the TF
    // tree. When camera_frame is set, the point is expressed in that frame
    // (x forward) instead of trusting the header. Empty = trust the header.
    declare_parameter("camera_frame", "camera_0_link");
    declare_parameter("min_blob_area_px", 400);
    declare_parameter("max_range_m", 8.0);
    // What this node writes into Detection.label. A different detector
    // sharing perception/detections (a YOLO node, a VLM) just picks its own.
    declare_parameter("label", "anomaly");
    // Red wraps around hue 0, so threshold two bands and OR them. The bands
    // are deliberately tight: orange-ish scenery lives around
    // hue 8-20 and will happily impersonate an anomaly with looser bounds.
    declare_parameter("hsv_low_1", std::vector<int64_t>{0, 150, 80});
    declare_parameter("hsv_high_1", std::vector<int64_t>{4, 255, 255});
    declare_parameter("hsv_low_2", std::vector<int64_t>{176, 150, 80});
    declare_parameter("hsv_high_2", std::vector<int64_t>{180, 255, 255});

    map_frame_ = get_parameter("map_frame").as_string();
    camera_frame_ = get_parameter("camera_frame").as_string();
    min_area_ = static_cast<int>(get_parameter("min_blob_area_px").as_int());
    max_range_ = get_parameter("max_range_m").as_double();
    label_ = get_parameter("label").as_string();
    auto hsv = [this](const std::string& param) {
      const auto v = get_parameter(param).as_integer_array();
      return cv::Scalar(v[0], v[1], v[2]);
    };
    hsv_low_1_ = hsv("hsv_low_1");
    hsv_high_1_ = hsv("hsv_high_1");
    hsv_low_2_ = hsv("hsv_low_2");
    hsv_high_2_ = hsv("hsv_high_2");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    detection_pub_ = create_publisher<spar::msg::Detection>(
        "perception/detections", 10);

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        get_parameter("info_topic").as_string(), 10,
        [this](const sensor_msgs::msg::CameraInfo& msg) { info_ = msg; });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        get_parameter("depth_topic").as_string(), rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
          depth_ = msg;
        });
    color_sub_ = create_subscription<sensor_msgs::msg::Image>(
        get_parameter("color_topic").as_string(), rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
          process(msg);
        });
  }

private:
  void process(const sensor_msgs::msg::Image::ConstSharedPtr& color) {
    if (!info_ || !depth_) return;  // camera not fully up yet

    cv::Mat bgr;
    try {
      bgr = cv_bridge::toCvShare(color, "bgr8")->image;
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "cv_bridge: %s", e.what());
      return;
    }

    // 1. Pixels: which ones are red?
    cv::Mat hsv, mask1, mask2, mask;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, hsv_low_1_, hsv_high_1_, mask1);
    cv::inRange(hsv, hsv_low_2_, hsv_high_2_, mask2);
    cv::bitwise_or(mask1, mask2, mask);

    // 2. Blob: the biggest red region, if it's big enough to be an object.
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    const auto biggest = std::max_element(
        contours.begin(), contours.end(),
        [](const auto& a, const auto& b) {
          return cv::contourArea(a) < cv::contourArea(b);
        });
    if (biggest == contours.end() || cv::contourArea(*biggest) < min_area_) {
      return;
    }
    const cv::Moments m = cv::moments(*biggest);
    const double cx = m.m10 / m.m00;
    const double cy = m.m01 / m.m00;

    // 3. Depth at the centroid -> a 3D point in the camera's optical frame.
    // The depth stream may be a different resolution than the color stream;
    // sample it in its own pixel coordinates or the range is fiction.
    const auto range = depth_at(cx / bgr.cols, cy / bgr.rows);
    if (!range || *range <= 0.1 || *range > max_range_) return;
    const double fx = info_->k[0], fy = info_->k[4];
    const double px = info_->k[2], py = info_->k[5];
    // Optical convention: x right, y down, z forward (out of the lens).
    const double ox = (cx - px) / fx * *range;
    const double oy = (cy - py) / fy * *range;
    const double oz = *range;

    geometry_msgs::msg::PointStamped in_camera;
    in_camera.header.stamp = color->header.stamp;
    if (camera_frame_.empty()) {
      in_camera.header.frame_id = color->header.frame_id;
      in_camera.point.x = ox;
      in_camera.point.y = oy;
      in_camera.point.z = oz;
    } else {
      // Re-express in the body-convention camera frame (x forward, y left,
      // z up) that actually exists in the TF tree.
      in_camera.header.frame_id = camera_frame_;
      in_camera.point.x = oz;
      in_camera.point.y = -ox;
      in_camera.point.z = -oy;
    }

    // 4. Into the map frame, so the fix outlives the camera viewpoint.
    geometry_msgs::msg::PointStamped in_map;
    try {
      in_map = tf_buffer_->transform(in_camera, map_frame_,
                                     tf2::durationFromSec(0.2));
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "anomaly seen but not localizable: %s", e.what());
      return;
    }
    spar::msg::Detection detection;
    detection.header = in_map.header;
    detection.point = in_map.point;
    detection.label = label_;
    detection_pub_->publish(detection);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "%s at map (%.1f, %.1f), range %.1fm",
                         label_.c_str(), in_map.point.x, in_map.point.y, *range);
  }

  // Median of the valid depth readings in a small patch around the centroid
  // (given as fractions of image size, so color/depth resolutions may differ) —
  // single pixels lie (NaN, zero, edge bleed), neighborhoods mostly don't.
  std::optional<double> depth_at(double u_frac, double v_frac) const {
    cv_bridge::CvImageConstPtr depth;
    try {
      depth = cv_bridge::toCvShare(depth_);
    } catch (const cv_bridge::Exception&) {
      return std::nullopt;
    }
    const cv::Mat& img = depth->image;
    std::vector<double> samples;
    for (int dy = -2; dy <= 2; ++dy) {
      for (int dx = -2; dx <= 2; ++dx) {
        const int x = static_cast<int>(u_frac * img.cols) + dx;
        const int y = static_cast<int>(v_frac * img.rows) + dy;
        if (x < 0 || y < 0 || x >= img.cols || y >= img.rows) continue;
        double d = 0.0;
        if (img.type() == CV_32FC1) {
          d = img.at<float>(y, x);
        } else if (img.type() == CV_16UC1) {
          d = img.at<uint16_t>(y, x) / 1000.0;  // mm -> m
        }
        if (std::isfinite(d) && d > 0.0) samples.push_back(d);
      }
    }
    if (samples.empty()) return std::nullopt;
    std::nth_element(samples.begin(), samples.begin() + samples.size() / 2,
                     samples.end());
    return samples[samples.size() / 2];
  }

  std::string map_frame_;
  std::string camera_frame_;
  std::string label_;
  int min_area_ = 400;
  double max_range_ = 8.0;
  cv::Scalar hsv_low_1_, hsv_high_1_, hsv_low_2_, hsv_high_2_;
  std::optional<sensor_msgs::msg::CameraInfo> info_;
  sensor_msgs::msg::Image::ConstSharedPtr depth_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<spar::msg::Detection>::SharedPtr detection_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
};

}  // namespace spar

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<spar::AnomalyDetector>());
  rclcpp::shutdown();
  return 0;
}
