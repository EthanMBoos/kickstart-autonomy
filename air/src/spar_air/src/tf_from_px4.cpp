// PX4's estimate as TF: VehicleLocalPosition + VehicleAttitude become
// map -> base_link, so the detector projects pixels through the estimated
// pose (no ground-truth TF from Unity). Also publishes the fixed
// base_link -> camera_0_link mount, which must match the Unity-side mount
// in SparPx4Link.cs (45 deg pitched down, body forward).
//
// This file owns the attitude half of the frame bug farm: PX4's quaternion
// is body-FRD relative to world-NED; the map frame is ENU and the robot
// frame FLU. Both fixed correction rotations are spelled out where they
// are applied. The position half (axis swap + pad offset) is frames.hpp.

#include <array>
#include <cmath>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "frames.hpp"

namespace spar_air {

class TfFromPx4 : public rclcpp::Node {
public:
  TfFromPx4() : Node("tf_from_px4") {
    declare_parameter("pad_x", 0.0);
    declare_parameter("pad_y", 0.0);
    declare_parameter("map_frame", "map");
    declare_parameter("base_frame", "base_link");
    declare_parameter("camera_frame", "camera_0_link");
    declare_parameter("camera_pitch_down_deg", 45.0);
    pad_x_ = get_parameter("pad_x").as_double();
    pad_y_ = get_parameter("pad_y").as_double();
    map_frame_ = get_parameter("map_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_broadcaster_ =
        std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    // The camera mount, base FLU: pitched down about +y (left), so body +x
    // rotates toward -z. Matches the Unity mount rotation in SparPx4Link.cs.
    const double half = 0.5 * get_parameter("camera_pitch_down_deg").as_double()
                        * M_PI / 180.0;
    geometry_msgs::msg::TransformStamped cam;
    cam.header.frame_id = base_frame_;
    cam.child_frame_id = get_parameter("camera_frame").as_string();
    cam.transform.rotation.w = std::cos(half);
    cam.transform.rotation.y = std::sin(half);
    static_broadcaster_->sendTransform(cam);

    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
        "fmu/out/vehicle_attitude", rclcpp::SensorDataQoS(),
        [this](const px4_msgs::msg::VehicleAttitude& msg) {
          q_ = msg.q;  // [w, x, y, z], body FRD -> world NED
          have_q_ = true;
        });

    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
        [this](const px4_msgs::msg::VehicleLocalPosition& msg) { publish(msg); });
  }

private:
  void publish(const px4_msgs::msg::VehicleLocalPosition& msg) {
    if (!have_q_) return;

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = base_frame_;

    const auto enu = nedToEnu(msg.x, msg.y, msg.z);
    tf.transform.translation.x = enu.x + pad_x_;
    tf.transform.translation.y = enu.y + pad_y_;
    tf.transform.translation.z = enu.z;

    // q maps FRD -> NED. Wanted: FLU -> ENU. Compose the two fixed frame
    // changes around it: ENU<-NED is a quarter turn about z then a half
    // turn about x (as a quaternion: (0, s, s, 0) with s = sqrt(1/2)),
    // and FRD<-FLU is a half turn about x ((0, 1, 0, 0)).
    //   q_out = q_enu_ned * q * q_frd_flu
    const double s = std::sqrt(0.5);
    const double a[4] = {0.0, s, s, 0.0};        // ENU <- NED
    const double b[4] = {q_[0], q_[1], q_[2], q_[3]};
    const double c[4] = {0.0, 1.0, 0.0, 0.0};    // FRD <- FLU
    double ab[4], out[4];
    mul(a, b, ab);
    mul(ab, c, out);
    tf.transform.rotation.w = out[0];
    tf.transform.rotation.x = out[1];
    tf.transform.rotation.y = out[2];
    tf.transform.rotation.z = out[3];
    broadcaster_->sendTransform(tf);
  }

  // Hamilton product, [w, x, y, z] like PX4 stores it.
  static void mul(const double p[4], const double q[4], double out[4]) {
    out[0] = p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3];
    out[1] = p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2];
    out[2] = p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1];
    out[3] = p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0];
  }

  double pad_x_ = 0.0, pad_y_ = 0.0;
  std::string map_frame_, base_frame_;
  std::array<float, 4> q_{};
  bool have_q_ = false;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
};

}  // namespace spar_air

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<spar_air::TfFromPx4>());
  rclcpp::shutdown();
  return 0;
}
