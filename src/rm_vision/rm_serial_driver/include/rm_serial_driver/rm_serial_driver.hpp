// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>

// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <fstream>
#include <iomanip>

#include "auto_aim_interfaces/msg/chassis.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/send.hpp"
#include "auto_aim_interfaces/msg/velocity.hpp"
#include "referee_interfaces/msg/rfid.hpp"
#include "referee_interfaces/msg/buff.hpp"
#include "referee_interfaces/msg/basic_hp.hpp"
#include "referee_interfaces/msg/ally_bot.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "referee_interfaces/msg/game_status.hpp"

namespace rm_serial_driver
{
class move_vec
{
  public:
    float vx=0;
    float vy=0;
    float wz=0;
}move_;
class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);

  ~RMSerialDriver() override;
  float pitch_trans(float originAngle);
  float pitch_re_trans(float originAngle);
  float yaw_trans(float originAngle);
  float yaw_re_trans(float originAngle);

private:
  // 在 RMSerialDriver 类的头文件中添加成员变量
  std::ofstream csv_file_;

  void getParams();

  void receiveData();

  void sendData(const auto_aim_interfaces::msg::Send::SharedPtr msg); // Old signature
  // void sendData(); // New signature for timer callback
  void get_classic(const geometry_msgs::msg::Twist twi);

  void reopenPort();

  void setParam(const rclcpp::Parameter & param);

  void resetTracker();

  // chassis tf
  void publishTransforms(double chassis_yaw_offset,double livox_yaw);

  // Serial port
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  // Param client to set detect_colr
  using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
  bool initial_set_param_ = false;
  uint8_t previous_receive_color_ = 0;
  rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
  ResultFuturePtr set_param_future_;

  // Service client to reset tracker
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;

  // Aimimg point receiving from serial port for visualization
  visualization_msgs::msg::Marker aiming_point_;
  //auto_aim_interfaces::msg::Referee refer_info;
  referee_interfaces::msg::Rfid rfid_info;
  referee_interfaces::msg::Buff buff_info;
  referee_interfaces::msg::BasicHp hp_info;
  referee_interfaces::msg::AllyBot allybot_info;
  auto_aim_interfaces::msg::Chassis Chassis_info;
  referee_interfaces::msg::GameStatus game_status_info;

  //geometry_msgs::msg::Twist classic_;

  double timestamp_offset_ = 0;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Velocity>::SharedPtr velocity_pub_;
  //rclcpp::Publisher<auto_aim_interfaces::msg::Referee>::SharedPtr referee_pub;
  rclcpp::Publisher<referee_interfaces::msg::Rfid>::SharedPtr rfid_pub;
  rclcpp::Publisher<referee_interfaces::msg::Buff>::SharedPtr buff_pub;
  rclcpp::Publisher<referee_interfaces::msg::BasicHp>::SharedPtr hp_pub;
  rclcpp::Publisher<referee_interfaces::msg::AllyBot>::SharedPtr allybot_pub;
  rclcpp::Publisher<auto_aim_interfaces::msg::Chassis>::SharedPtr Chassis_pub;
  rclcpp::Publisher<referee_interfaces::msg::GameStatus>::SharedPtr game_status_pub;

  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Send>::SharedPtr send_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr move_vec_sub;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr gimbal_control_send_timer_;
  bool gimbal_tracking_ = false;
  float gimbal_pitch_ = 0.0;
  float gimbal_yaw_ = 0.0;
  rclcpp::Time gimbal_data_stamp_;

  std::thread receive_thread_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
