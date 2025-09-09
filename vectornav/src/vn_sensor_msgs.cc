/** VectorNav ROS2 Interface
 *
 * Copyright 2021 Dereck Wonnacott <dereck@gmail.com>
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT.
 */

#include "vn_sensor_msgs.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace vectornav
{
/// Convert from DEG to RAD
static double deg2rad(double in) { return in * M_PI / 180.0; }

VnSensorMsgs::VnSensorMsgs(const rclcpp::NodeOptions & options) : Node("vn_sensor_msgs", options)
{
  // Parameters
  declare_parameter<bool>("use_enu", true);
  declare_parameter<std::vector<double>>("orientation_covariance", orientation_covariance_);
  declare_parameter<std::vector<double>>(
    "angular_velocity_covariance", angular_velocity_covariance_);
  declare_parameter<std::vector<double>>(
    "linear_acceleration_covariance", linear_acceleration_covariance_);
  declare_parameter<std::vector<double>>("magnetic_covariance", magnetic_field_covariance_);
  declare_parameter<std::string>("model", "vn100");
  declare_parameter<bool>("publish_time_startup", false);
  declare_parameter<bool>("publish_time_sync_in", false);


  //test
  declare_parameter<double>("roll", 0.0);
  declare_parameter<double>("pitch", 0.0);
  declare_parameter<double>("yaw", 0.0);

  std::string model = get_parameter("model").as_string();
  if(model == "vn100"){
    series_ = 1;
  }
  else if(model == "vn200"){
    series_ = 2;
  }
  else if(model == "vn300"){
    series_ = 3;
  }
  else{
    RCLCPP_INFO(this->get_logger(), "Invalid model ID. Assuming vn100.");
    series_ = 1;
  }
  //
  // Publishers
  //
  // TODO(Dereck): Only publish if data is available from the sensor?
  
  time_startup_en_ = get_parameter("publish_time_startup").as_bool();
  time_syncin_en_ = get_parameter("publish_time_sync_in").as_bool();

  if(time_startup_en_){
    pub_time_startup_ = this->create_publisher<sensor_msgs::msg::TimeReference>("vectornav/time_startup", 10);
  }

  if(time_syncin_en_){
    pub_time_syncin_ =
    this->create_publisher<sensor_msgs::msg::TimeReference>("vectornav/time_syncin", 10);
  }

  
  if(series_ > 1){
    pub_time_gps_ = this->create_publisher<sensor_msgs::msg::TimeReference>("vectornav/time_gps", 10);
    pub_gnss_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("vectornav/gnss", 10);
    pub_time_pps_ = this->create_publisher<sensor_msgs::msg::TimeReference>("vectornav/time_pps", 10);
    pub_pose_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("vectornav/pose", 10);

  }
  
  pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("vectornav/imu", 10);
  pub_imu_uncompensated_ =
    this->create_publisher<sensor_msgs::msg::Imu>("vectornav/imu_uncompensated", 10);
  pub_magnetic_ = this->create_publisher<sensor_msgs::msg::MagneticField>("vectornav/magnetic", 10);
  pub_temperature_ =
    this->create_publisher<sensor_msgs::msg::Temperature>("vectornav/temperature", 10);
  pub_pressure_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("vectornav/pressure", 10);
  pub_velocity_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "vectornav/velocity_body", 10);
  

  //
  // Subscribers
  //


  if(series_ > 1){
    auto sub_vn_gps_cb = std::bind(&VnSensorMsgs::sub_vn_gps, this, std::placeholders::_1);
      sub_vn_gps_ = this->create_subscription<vectornav_msgs::msg::GpsGroup>(
      "vectornav/raw/gps", 10, sub_vn_gps_cb);

      auto sub_vn_ins_cb = std::bind(&VnSensorMsgs::sub_vn_ins, this, std::placeholders::_1);
      sub_vn_ins_ = this->create_subscription<vectornav_msgs::msg::InsGroup>(
      "vectornav/raw/ins", 10, sub_vn_ins_cb);
  }

  if(series_ == 3){
    auto sub_vn_gps2_cb = std::bind(&VnSensorMsgs::sub_vn_gps2, this, std::placeholders::_1);
      sub_vn_gps2_ = this->create_subscription<vectornav_msgs::msg::GpsGroup>(
      "vectornav/raw/gps2", 10, sub_vn_gps2_cb);
  }

  auto sub_vn_common_cb = std::bind(&VnSensorMsgs::sub_vn_common, this, std::placeholders::_1);
  sub_vn_common_ = this->create_subscription<vectornav_msgs::msg::CommonGroup>(
    "vectornav/raw/common", 10, sub_vn_common_cb);

  auto sub_vn_time_cb = std::bind(&VnSensorMsgs::sub_vn_time, this, std::placeholders::_1);
  sub_vn_time_ = this->create_subscription<vectornav_msgs::msg::TimeGroup>(
    "vectornav/raw/time", 10, sub_vn_time_cb);

  auto sub_vn_imu_cb = std::bind(&VnSensorMsgs::sub_vn_imu, this, std::placeholders::_1);
  sub_vn_imu_ = this->create_subscription<vectornav_msgs::msg::ImuGroup>(
    "vectornav/raw/imu", 10, sub_vn_imu_cb);

  auto sub_vn_attitude_cb = std::bind(&VnSensorMsgs::sub_vn_attitude, this, std::placeholders::_1);
  sub_vn_attitude_ = this->create_subscription<vectornav_msgs::msg::AttitudeGroup>(
    "vectornav/raw/attitude", 10, sub_vn_attitude_cb);

  //enu frame option
  use_enu = get_parameter("use_enu").as_bool();
  std::cout << "ENU value: " << use_enu <<std::endl;
}


static void convert_vec_frd_to_rfu(const geometry_msgs::msg::Vector3 & vec_frd, geometry_msgs::msg::Vector3 & vec_rfu)
{
  // swap x and y and negate z
  vec_rfu.x = vec_frd.y;
  vec_rfu.y = vec_frd.x;
  vec_rfu.z = -vec_frd.z;
}

static void convert_to_enu(const geometry_msgs::msg::Quaternion & q_msg_frd2ned, geometry_msgs::msg::Quaternion & q_msg_rfu2enu, double r, double p, double y)
{
  // convert from FRD_TO_NED to RFU_TO_ENU attitude
  static const tf2::Quaternion q_ned2enu(tf2::Vector3(1, 1, 0).normalized(), M_PI);
  static const tf2::Quaternion q_rfu2frd(tf2::Vector3(1, 1, 0).normalized(), M_PI);
  //static const tf2::Quaternion q_ned2enu(0.707, 0.707, 0.0, 0.0);
  //static const tf2::Quaternion q_rfu2frd(0.0, 0.0, 0.707, -0.707);

  //tf2::Quaternion q_rot;
  
  //double r=3.14159, p=0.0, y=1.5709;
  //q_rot.setRPY(r,p,y);
  //std::cout << "q_rot: " << q_rot.getX() << ", " << q_rot.getY() << ", " << q_rot.getZ() << ", " << q_rot.getW() << std::endl;
//
  bool transformRoll = false;
  bool transformYaw = false;
  tf2::Quaternion q_rot_r, q_rot_y;
  if(r > 0)
  {
    q_rot_r.setRPY(r, 0, 0); //FIXME: If this works, take the if statement off!
    transformRoll = true;
  }
///  else if(p > 0)
///  {
///    q_rot.setRPY(0, p, 0);
///    .transform = true;
///  }
///  
  if(y > 0)
  {
    q_rot_y.setRPY(0, 0, y);
    transformYaw = true;
  }


  tf2::Quaternion q_frd2ned;
  tf2::fromMsg(q_msg_frd2ned, q_frd2ned);
  //tf2::Quaternion q_rfu2enu = q_rot * q_frd2ned;
  //q_rfu2enu.normalize();
  tf2::Quaternion q_rfu2enu = q_ned2enu * q_frd2ned * q_rfu2frd;
  if(transformRoll)
  {
    q_rfu2enu = q_rot_r * q_rfu2enu;
  }
  if(transformYaw)
  {
    q_rfu2enu = q_rot_y * q_rfu2enu;
  }
  q_msg_rfu2enu = tf2::toMsg(q_rfu2enu);
}

/** Convert VN common group data to ROS2 standard message types
   *
   */
void VnSensorMsgs::sub_vn_common(const vectornav_msgs::msg::CommonGroup::SharedPtr msg_in) const
{
  // RCLCPP_INFO(get_logger(), "Frame ID: '%s'", msg_in->header.frame_id.c_str());

  // Time Reference (Startup)
  if(time_startup_en_){
    sensor_msgs::msg::TimeReference msg;
    msg.header = msg_in->header;

    msg.time_ref.sec = std::floor(msg_in->timestartup / 1e9);
    msg.time_ref.nanosec = msg_in->timestartup - (msg.time_ref.sec * 1e9);

    msg.source = "startup";

    pub_time_startup_->publish(msg);
  }

  // Time Reference (GPS)
  if(series_ > 1){
    sensor_msgs::msg::TimeReference msg;
    msg.header = msg_in->header;

    msg.time_ref.sec = std::floor(msg_in->timegps / 1e9);
    msg.time_ref.nanosec = msg_in->timegps - (msg.time_ref.sec * 1e9);

    msg.source = "gps";

    pub_time_gps_->publish(msg);
  }

  // Time Reference (SyncIn)
  if(time_syncin_en_){
    sensor_msgs::msg::TimeReference msg;
    msg.header = msg_in->header;

    msg.time_ref.sec = std::floor(msg_in->timesyncin / 1e9);
    msg.time_ref.nanosec = msg_in->timesyncin - (msg.time_ref.sec * 1e9);

    msg.source = "syncin";

    pub_time_syncin_->publish(msg);
  }

  // Time Reference (PPS)
  if(series_ > 1){
    sensor_msgs::msg::TimeReference msg;
    msg.header = msg_in->header;

    msg.time_ref.sec = std::floor(msg_in->timegpspps / 1e9);
    msg.time_ref.nanosec = msg_in->timegpspps - (msg.time_ref.sec * 1e9);

    msg.source = "pps";

    pub_time_pps_->publish(msg);
  }

  // IMU
  {
    sensor_msgs::msg::Imu msg;
    msg.header = msg_in->header;

    if (use_enu) {
      convert_vec_frd_to_rfu(msg_in->angularrate, msg.angular_velocity);
      convert_vec_frd_to_rfu(msg_in->accel, msg.linear_acceleration);
      //msg.angular_velocity = msg_in->angularrate;
      //msg.linear_acceleration = msg_in->accel;
      convert_to_enu(msg_in->quaternion, msg.orientation, get_parameter("roll").as_double(), get_parameter("pitch").as_double(), get_parameter("yaw").as_double());
    } else {
      msg.angular_velocity = msg_in->angularrate;
      msg.linear_acceleration = msg_in->accel;
      msg.orientation = msg_in->quaternion;
    }

//    std::cout << "NED: " << msg_in->quaternion.z << " ENU: " << msg.orientation.z << std::endl;
//    std::cout << "NED: " << msg_in->quaternion.w << " ENU: " << msg.orientation.w << std::endl;

    fill_covariance_from_param("orientation_covariance", msg.orientation_covariance);
    fill_covariance_from_param("angular_velocity_covariance", msg.angular_velocity_covariance);
    fill_covariance_from_param(
      "linear_acceleration_covariance", msg.linear_acceleration_covariance);

    pub_imu_->publish(msg);
  }

  // IMU (Uncompensated)
  {
    sensor_msgs::msg::Imu msg;
    msg.header = msg_in->header;

    if (use_enu) {
      convert_vec_frd_to_rfu(msg_in->imu_rate, msg.angular_velocity);
      convert_vec_frd_to_rfu(msg_in->imu_accel, msg.linear_acceleration);
      convert_to_enu(msg_in->quaternion, msg.orientation, get_parameter("roll").as_double(), get_parameter("pitch").as_double(), get_parameter("yaw").as_double());
    } else {
      msg.angular_velocity = msg_in->imu_rate;
      msg.linear_acceleration = msg_in->imu_accel;
    }

    fill_covariance_from_param("angular_velocity_covariance", msg.angular_velocity_covariance);
    fill_covariance_from_param(
      "linear_acceleration_covariance", msg.linear_acceleration_covariance);

    pub_imu_uncompensated_->publish(msg);
  }

  // Magnetic Field
  {
    sensor_msgs::msg::MagneticField msg;
    msg.header = msg_in->header;
    if (use_enu) {
      convert_vec_frd_to_rfu(msg_in->magpres_mag, msg.magnetic_field);
    } else {
      msg.magnetic_field = msg_in->magpres_mag;
    }

    fill_covariance_from_param("magnetic_covariance", msg.magnetic_field_covariance);

    pub_magnetic_->publish(msg);
  }

  // Temperature
  {
    sensor_msgs::msg::Temperature msg;
    msg.header = msg_in->header;
    msg.temperature = msg_in->magpres_temp;

    pub_temperature_->publish(msg);
  }

  // Pressure
  {
    sensor_msgs::msg::FluidPressure msg;
    msg.header = msg_in->header;

    // Convert kPa to Pa
    msg.fluid_pressure = msg_in->magpres_pres * 1e3;

    pub_pressure_->publish(msg);
  }

  // GNSS
  if(series_ > 1){
    sensor_msgs::msg::NavSatFix msg;
    msg.header = msg_in->header;

    // Status
    if (gps_fix_ == vectornav_msgs::msg::GpsGroup::GPSFIX_NOFIX) {
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    } else {
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    }

    // Position
    msg.latitude = msg_in->position.x;
    msg.longitude = msg_in->position.y;
    msg.altitude = msg_in->position.z;

    // Covariance (Convert NED to ENU)
    /// TODO(Dereck): Use DOP for better estimate?
    const std::vector<double> orientation_covariance_ = {
      gps_posu_.y, 0.0000, 0.0000, 0.0000, gps_posu_.x, 0.0000, 0.0000, 0.0000, gps_posu_.z};

    msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    pub_gnss_->publish(msg);
  }

  // Velocity
  {
    geometry_msgs::msg::TwistWithCovarianceStamped msg;
    msg.header = msg_in->header;
    if (use_enu) {
      convert_vec_frd_to_rfu(ins_velbody_, msg.twist.twist.linear);
      convert_vec_frd_to_rfu(msg_in->angularrate, msg.twist.twist.angular);
    } else {
      msg.twist.twist.linear = ins_velbody_;
      msg.twist.twist.angular = msg_in->angularrate;
    }

    /// TODO(Dereck): Velocity Covariance

    pub_velocity_->publish(msg);
  }

  // Pose
  if(series_ > 1){
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header = msg_in->header;
    msg.header.frame_id = "earth";
    msg.pose.pose.position = ins_posecef_;

    // Converts Quaternion to ECEF
    tf2::Quaternion q_frd2ned, q_ned2ecef;
    tf2::fromMsg(msg_in->quaternion, q_frd2ned);
    auto latitude = deg2rad(msg_in->position.x);
    auto longitude = deg2rad(msg_in->position.y);
    q_ned2ecef = tf2::Quaternion(tf2::Vector3(0, 0, 1), longitude)
                  * tf2::Quaternion(tf2::Vector3(0, 1, 0), -M_PI/2 - latitude);

    if (use_enu) {
      static const tf2::Quaternion q_rfu2frd(tf2::Vector3(1, 1, 0).normalized(), M_PI);
      tf2::Quaternion q_rfu2ecef =q_ned2ecef * q_frd2ned * q_rfu2frd;
      msg.pose.pose.orientation = tf2::toMsg(q_rfu2ecef);
    } else {
      tf2::Quaternion q_frd2ecef =q_ned2ecef * q_frd2ned;
      msg.pose.pose.orientation = tf2::toMsg(q_frd2ecef);
    }

    /// TODO(Dereck): Pose Covariance

    pub_pose_->publish(msg);
  }
}

/** Convert VN time group data to ROS2 standard message types
   *
   */
void VnSensorMsgs::sub_vn_time(const vectornav_msgs::msg::TimeGroup::SharedPtr msg_in) const {}

/** Convert VN imu group data to ROS2 standard message types
   *
   */
void VnSensorMsgs::sub_vn_imu(const vectornav_msgs::msg::ImuGroup::SharedPtr msg_in) const {}

/** Convert VN gps group data to ROS2 standard message types
   *
   * TODO(Dereck): Consider alternate sync methods
   */
void VnSensorMsgs::sub_vn_gps(const vectornav_msgs::msg::GpsGroup::SharedPtr msg_in)
{
  gps_fix_ = msg_in->fix;
  gps_posu_ = msg_in->posu;
}

/** Convert VN attitude group data to ROS2 standard message types
   *
   */
void VnSensorMsgs::sub_vn_attitude(const vectornav_msgs::msg::AttitudeGroup::SharedPtr msg_in) const
{
}

/** Convert VN ins group data to ROS2 standard message types
   *
   */
void VnSensorMsgs::sub_vn_ins(const vectornav_msgs::msg::InsGroup::SharedPtr msg_in)
{
  ins_velbody_ = msg_in->velbody;
  ins_posecef_ = msg_in->posecef;
}

/** Convert VN gps2 group data to ROS2 standard message types
   *
   */
void VnSensorMsgs::sub_vn_gps2(const vectornav_msgs::msg::GpsGroup::SharedPtr msg_in) const {}

/** Copy a covariance matrix array from a parameter into a msg array
   *
   * If a single value is provided, this will set the diagonal values
   * If three values are provided, this will set the diagonal values
   * If nine values are provided, this will fill the matrix
   *
   * \param param_name Name of the parameter to read
   * \param array Array to fill
   */
void VnSensorMsgs::fill_covariance_from_param(
  std::string param_name, std::array<double, 9> & array) const
{
  auto covariance = get_parameter(param_name).as_double_array();

  auto length = covariance.size();
  switch (length) {
    case 1:
      array[0] = covariance[0];
      array[3] = covariance[0];
      array[8] = covariance[0];
      break;

    case 3:
      array[0] = covariance[0];
      array[3] = covariance[1];
      array[8] = covariance[3];
      break;

    case 9:
      std::copy(covariance.begin(), covariance.end(), array.begin());
      break;

    default:
      RCLCPP_ERROR(
        get_logger(), "Parameter '%s' length is %zu; expected 1, 3, or 9", param_name.c_str(),
        length);
  }
}
}


RCLCPP_COMPONENTS_REGISTER_NODE(vectornav::VnSensorMsgs)
