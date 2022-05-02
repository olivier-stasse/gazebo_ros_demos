// Copyright 2021 CNRS, LAAS
// Copyright 2013 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <ignition/math/Rand.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_tutorials/solar_sensor.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>

#ifdef NO_ERROR
// NO_ERROR is a macro defined in Windows that's used as an enum in tf2
#undef NO_ERROR
#endif

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif

#include <string>
#include <memory>

namespace gazebo_plugins
{

class SolarSensorPrivate
{
public:
  
  /// Callback to be called at every simulation iteration
  /// \param[in] info Updated simulation info
  void OnUpdate(const gazebo::common::UpdateInfo & info);

  /// The link being traked.
  gazebo::physics::LightPtr light_{nullptr};

  /// The body of the frame to display pose, twist
  gazebo::physics::LinkPtr reference_link_{nullptr};

  /// Pointer to ros node
  gazebo_ros::Node::SharedPtr ros_node_{nullptr};

  /// Odometry publisher
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_{nullptr};

  /// Odom topic name
  std::string topic_name_{"sunloin_imu"};

  /// Frame transform name, should match name of reference link, or be world.
  std::string frame_name_{"base_solar_sensor"};

  /// Turtlebot3 waffle
  std::string model_name_{"turtlebot3_waffle"};

  /// Constant xyz and rpy offsets
  ignition::math::Pose3d offset_;

  /// Keep track of the last update time.
  gazebo::common::Time last_time_;

  /// Publish rate in Hz.
  double update_rate_{0.0};

  /// Gaussian noise
  double gaussian_noise_;

  /// Pointer to the update event connection
  gazebo::event::ConnectionPtr update_connection_{nullptr};
};

SolarSensor::SolarSensor()
: impl_(std::make_unique<SolarSensorPrivate>())
{
}

SolarSensor::~SolarSensor()
{
}
  
// Load the controller
void SolarSensor::Load(gazebo::physics::ModelPtr model,
		       sdf::ElementPtr sdf)
{
  // Configure the plugin from the SDF file
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  if (!sdf->HasElement("update_rate")) {
    RCLCPP_DEBUG(
      impl_->ros_node_->get_logger(), "solar_sensor plugin missing <update_rate>, defaults to 0.0"
      " (as fast as possible)");
  } else {
    impl_->update_rate_ = sdf->GetElement("update_rate")->Get<double>();
  }

  std::string light_name;
  if (!sdf->HasElement("light_name")) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Missing <light_name>, cannot proceed");
    return;
  } else {
    light_name = sdf->GetElement("light_name")->Get<std::string>();
  }

  gazebo::physics::WorldPtr world = model->GetWorld();
  impl_->light_ = world->LightByName(light_name);
  if (!impl_->light_) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(), "light_name: %s does not exist\n",
      light_name.c_str());
    return;
  }

  impl_->pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::Imu>(
    impl_->topic_name_, qos.get_publisher_qos(
      impl_->topic_name_, rclcpp::SensorDataQoS().reliable()));
  impl_->topic_name_ = impl_->pub_->get_topic_name();
  RCLCPP_DEBUG(
    impl_->ros_node_->get_logger(), "Publishing on topic [%s]", impl_->topic_name_.c_str());

  if (sdf->HasElement("xyz_offsets")) {
    RCLCPP_WARN(
      impl_->ros_node_->get_logger(), "<xyz_offsets> is deprecated, use <xyz_offset> instead.");
    impl_->offset_.Pos() = sdf->GetElement("xyz_offsets")->Get<ignition::math::Vector3d>();
  }
  if (!sdf->HasElement("xyz_offset")) {
    if (!sdf->HasElement("xyz_offsets")) {
      RCLCPP_DEBUG(impl_->ros_node_->get_logger(), "Missing <xyz_offset>, defaults to 0s");
    }
  } else {
    impl_->offset_.Pos() = sdf->GetElement("xyz_offset")->Get<ignition::math::Vector3d>();
  }

  if (sdf->HasElement("rpy_offsets")) {
    RCLCPP_WARN(
      impl_->ros_node_->get_logger(), "<rpy_offsets> is deprecated, use <rpy_offset> instead.");
    impl_->offset_.Rot() = ignition::math::Quaterniond(
      sdf->GetElement("rpy_offsets")->Get<ignition::math::Vector3d>());
  }
  if (!sdf->HasElement("rpy_offset")) {
    if (!sdf->HasElement("rpy_offsets")) {
      RCLCPP_DEBUG(impl_->ros_node_->get_logger(), "Missing <rpy_offset>, defaults to 0s");
    }
  } else {
    impl_->offset_.Rot() = ignition::math::Quaterniond(
      sdf->GetElement("rpy_offset")->Get<ignition::math::Vector3d>());
  }

  if (!sdf->HasElement("gaussian_noise")) {
    RCLCPP_DEBUG(impl_->ros_node_->get_logger(), "Missing <gassian_noise>, defaults to 0.0");
    impl_->gaussian_noise_ = 0;
  } else {
    impl_->gaussian_noise_ = sdf->GetElement("gaussian_noise")->Get<double>();
  }

  impl_->last_time_ = world->SimTime();

  if (!sdf->HasElement("frame_name")) {
    RCLCPP_DEBUG(
      impl_->ros_node_->get_logger(), "Missing <frame_name>, defaults to base_solar_sensor");
  } else {
    impl_->frame_name_ = sdf->GetElement("frame_name")->Get<std::string>();
  }

  if (model!=nullptr)
  {
    impl_->reference_link_ = model->GetLink(impl_->frame_name_);
    if (!impl_->reference_link_) {
      RCLCPP_WARN(
        impl_->ros_node_->get_logger(), "<frame_name> [%s] does not exist.",
        impl_->frame_name_.c_str());
    }
  }

  // Listen to the update event. This event is broadcast every simulation iteration
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&SolarSensorPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

// Update the controller
void SolarSensorPrivate::OnUpdate(const gazebo::common::UpdateInfo & info)
{
  if (!light_) {
    return;
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("SolarSensorPrivate::OnUpdate");
#endif
  gazebo::common::Time current_time = info.simTime;

  if (current_time < last_time_) {
    RCLCPP_WARN(ros_node_->get_logger(), "Negative update time difference detected.");
    last_time_ = current_time;
  }

  // Rate control
  if (update_rate_ > 0 &&
    (current_time - last_time_).Double() < (1.0 / update_rate_))
  {
    return;
  }

  // If we don't have any subscribers, don't bother composing and sending the message
  if (ros_node_->count_subscribers(topic_name_) == 0) {
    return;
  }

  // Differentiate to get accelerations
  double tmp_dt = current_time.Double() - last_time_.Double();
  if (tmp_dt == 0) {
    return;
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_BEGIN("fill ROS message");
#endif
  sensor_msgs::msg::Imu imu_msg;

  // Copy data into pose message
  imu_msg.header.frame_id = frame_name_;
  imu_msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);

  // Get inertial rates
  ignition::math::Vector3d vpos = light_->WorldLinearVel();
  ignition::math::Vector3d veul = light_->WorldAngularVel();

  // Get pose/orientation
  auto pose = light_->WorldPose();

  // Apply reference frame
  if (reference_link_) {
    // Convert to relative pose, rates
    auto frame_pose = reference_link_->WorldPose();
    auto frame_vpos = reference_link_->WorldLinearVel();
    auto frame_veul = reference_link_->WorldAngularVel();

    pose.Pos() = pose.Pos() - frame_pose.Pos();
    pose.Pos().Normalize();
    pose.Pos() = frame_pose.Rot().RotateVectorReverse(pose.Pos());
    pose.Rot() *= frame_pose.Rot().Inverse();

    vpos = frame_pose.Rot().RotateVector(vpos - frame_vpos);
    veul = frame_pose.Rot().RotateVector(veul - frame_veul);
  }

  // Apply constant offsets

  // Apply XYZ offsets and get position and rotation components
  pose.Pos() = pose.Pos() + offset_.Pos();
  // Apply RPY offsets
  pose.Rot() = offset_.Rot() * pose.Rot();
  pose.Rot().Normalize();

  double pitch = std::asin(-pose.Pos().Z());
  double yaw= std::atan2(pose.Pos().Y(), pose.Pos().X());
  
  ignition::math::Quaterniond compass_to_sunloin(0.0, pitch, yaw);
  // Fill out messages
  imu_msg.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(compass_to_sunloin);

  // Fill in covariance matrix
  /// @TODO: let user set separate linear and angular covariance values
  double gn2 = gaussian_noise_ * gaussian_noise_;
  imu_msg.orientation_covariance[0] = gn2;
  imu_msg.orientation_covariance[4] = gn2;  
  imu_msg.orientation_covariance[8] = gn2;
  
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("publish");
#endif
  // Publish to ROS
  pub_->publish(imu_msg);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
  // Save last time stamp
  last_time_ = current_time;
}

GZ_REGISTER_MODEL_PLUGIN(SolarSensor)

}  // namespace gazebo_plugins
