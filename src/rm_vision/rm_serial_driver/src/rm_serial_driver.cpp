// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include "rm_serial_driver/rm_serial_driver.hpp"

// ROS
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
// TF2
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <math.h>
#include <iomanip>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{

  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

    getParams();

  // Create Publisher
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::QoS(rclcpp::KeepLast(1)));
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);
  velocity_pub_ = this->create_publisher<auto_aim_interfaces::msg::Velocity>("/current_velocity", 10);
  Chassis_pub = this->create_publisher<auto_aim_interfaces::msg::Chassis>("/Chassis", 10);

    // TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 裁判系统
    // referee_pub=this->create_publisher<auto_aim_interfaces::msg::Referee>("/referee_info", 10);
    buff_pub = this->create_publisher<referee_interfaces::msg::Buff>("/buff", 10);
    rfid_pub = this->create_publisher<referee_interfaces::msg::Rfid>("/rfid", 10);
    hp_pub = this->create_publisher<referee_interfaces::msg::BasicHp>("/basichp", 10);
    allybot_pub = this->create_publisher<referee_interfaces::msg::AllyBot>("/allybot", 10);
    // Detect parameter client
    detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");

    // Tracker reset service client
    reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

    try
    {
      serial_driver_->init_port(device_name_, *device_config_);
      if (!serial_driver_->port()->is_open())
      {
        serial_driver_->port()->open();
        receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
      }
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(
          get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
      throw ex;
    }

    aiming_point_.header.frame_id = "odom";
    aiming_point_.ns = "aiming_point";
    aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
    aiming_point_.action = visualization_msgs::msg::Marker::ADD;
    aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
    aiming_point_.color.r = 1.0;
    aiming_point_.color.g = 1.0;
    aiming_point_.color.b = 1.0;
    aiming_point_.color.a = 1.0;
    aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);
    // Create Subscription
    move_vec_sub = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::SensorDataQoS(), std::bind(&RMSerialDriver::get_classic, this, std::placeholders::_1));

    send_sub_ = this->create_subscription<auto_aim_interfaces::msg::Send>(
        "/gimbalcontrol", rclcpp::SensorDataQoS(),
        std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));

    // target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
    //   "/tracker/target", rclcpp::SensorDataQoS(),
    //   std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));
  }

  RMSerialDriver::~RMSerialDriver()
  {
  }

  void RMSerialDriver::get_classic(const geometry_msgs::msg::Twist twi)
  {
    move_.vx = twi.linear.x;
    move_.vy = twi.linear.y;
    move_.wz = twi.angular.z;
  }

  void RMSerialDriver::receiveData()
  {
    std::vector<uint8_t> header(1);
    std::vector<uint8_t> data;
    data.reserve(sizeof(ReceivePacket));
    // std::cout<<"**************"<<std::endl;
    while (rclcpp::ok())
    {
      try
      {
        serial_driver_->port()->receive(header);

        if (header[0] == 0x5A)
        {
          data.resize(sizeof(ReceivePacket) - 1);
          serial_driver_->port()->receive(data);
          int detect_color = 0;
          data.insert(data.begin(), header[0]);
          //  std::cout<<(int)data.size()<<std::endl<<std::endl;
          //   for(int i=0;i<(int)data.size();i++)
          //     std::cout<<std:uint16_t time:hex<<static_cast<int>(data[i])<<' ';
          //   std::cout<<std::endl;
          ReceivePacket packet = fromVector(data);
          // std::cout<<std::endl;
            std::cout<<"checksum"<<packet.checksum<<std::endl;
          //  std::cout<<"send pitch:"<<packet.eulr.yaw<<std::endl;
          //  std::cout<<sizeof(packet)<<std::endl;
          //  std::cout<<"---------------------"<<std::endl;
          //  std::cout<<packet.eulr.pit<<std::endl;
          //  std::cout<<packet.eulr.yaw<<std::endl;
          //  std::cout<<packet.eulr.rol<<std::endl;
          //  std::cout<<packet.notice<<endl;
          //  std::cout<<packet.current_v<<std::endl;

          // std::cout<<packet.pitch<<std::endl;
          // std::cout<<packet.yaw<<std::endl;
          // std::cout<<packet.roll<<std::endl;

          bool crc_ok = crc16::CRC16_Verify(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
          if (crc_ok)
          {

            packet.pitch = RMSerialDriver::pitch_re_trans(packet.eulr.pit);
            packet.yaw = RMSerialDriver::pitch_re_trans(packet.eulr.yaw);

            int temp = packet.rfid;
            int count = 0;
            rfid_info.base_gain_point = (temp >> (count++)) & 1;
            rfid_info.central_highland_gain_point = (temp >> (count++)) & 1;
            rfid_info.enemy_central_highland_gain_point = (temp >> (count++)) & 1;
            rfid_info.friendly_trapezoidal_highland_gain_point = (temp >> (count++)) & 1;
            rfid_info.enemy_trapezoidal_highland_gain_point = (temp >> (count++)) & 1;
            rfid_info.friendly_fly_ramp_front_gain_point = (temp >> (count++)) & 1;
            rfid_info.friendly_fly_ramp_back_gain_point = (temp >> (count++)) & 1;
            rfid_info.enemy_fly_ramp_front_gain_point = (temp >> (count++)) & 1;
            rfid_info.enemy_fly_ramp_back_gain_point = (temp >> (count++)) & 1;
            rfid_info.friendly_central_highland_lower_gain_point = (temp >> (count++)) & 1;
            rfid_info.friendly_central_highland_upper_gain_point = (temp >> (count++)) & 1;
            rfid_info.enemy_central_highland_lower_gain_point = (temp >> (count++)) & 1;
            rfid_info.enemy_central_highland_upper_gain_point = (temp >> (count++)) & 1;
            rfid_info.friendly_highway_lower_gain_point = (temp >> (count++)) & 1;
            rfid_info.friendly_highway_upper_gain_point = (temp >> (count++)) & 1;
            rfid_info.enemy_highway_lower_gain_point = (temp >> (count++)) & 1;
            rfid_info.friendly_highway_upper_gain_point = (temp >> (count++)) & 1;
            rfid_info.enemy_highway_lower_gain_point = (temp >> (count++)) & 1;
            rfid_info.enemy_highway_upper_gain_point = (temp >> (count++)) & 1;
            rfid_info.friendly_fortress_gain_point = (temp >> (count++)) & 1;
            rfid_info.friendly_outpost_gain_point = (temp >> (count++)) & 1;
            rfid_info.friendly_supply_zone_non_exchange = (temp >> (count++)) & 1;
            rfid_info.friendly_supply_zone_exchange = (temp >> (count++)) & 1;
            rfid_info.friendly_big_resource_island = (temp >> (count++)) & 1;
            rfid_info.enemy_big_resource_island = (temp >> (count++)) & 1;
            rfid_info.center_gain_point = (temp >> (count++)) & 1;
            rfid_pub->publish(rfid_info);

            buff_info.recovery_buff = packet.recovery_buff;
            buff_info.cooling_buff = packet.cooling_buff;
            buff_info.defence_buff = packet.defence_buff;
            buff_info.vulnerability_buff = packet.vulnerability_buff;
            buff_info.attack_buff = packet.attack_buff;
            buff_info.remaining_energy = packet.remaining_energy;
            buff_pub->publish(buff_info);

            hp_info.sentry_hp = packet.sentry_hp;
            hp_info.base_hp = packet.base_hp;
            hp_info.outpost_hp = packet.outpost_hp;
            hp_info.projectile_allowance_17mm = packet.projectile_allowance_17mm;
            hp_pub->publish(hp_info);

            allybot_info.hero_hp = packet.hero_hp;
            allybot_info.hero_position.x = packet.hero_position_x;
            allybot_info.hero_position.y = packet.hero_position_y;
            allybot_info.standard_3_hp = packet.standard_3_hp;
            allybot_info.standard_3_position.x = packet.standard_3_position_x;
            allybot_info.standard_3_position.y = packet.standard_3_position_y;
            allybot_info.standard_4_hp = packet.standard_4_hp;
            allybot_info.standard_4_position.x = packet.standard_4_position_x;
            allybot_info.standard_4_position.y = packet.standard_4_position_y;
            allybot_info.engineer_hp = packet.engineer_hp;
            allybot_info.engineer_position.x = packet.engineer_position_x;
            allybot_info.engineer_position.y = packet.engineer_position_y;
            allybot_info.hero_position.z = 0;
            allybot_info.standard_3_position.z = 0;
            allybot_info.standard_4_position.z = 0;
            allybot_info.engineer_position.z = 0;
            allybot_pub->publish(allybot_info);

            Chassis_info.chassis_yaw_offset = packet.chassis_yaw_offset_;
            Chassis_info.damaged_armor_id = packet.damaged_armor_id;
            // 发布 Chassis 信息
            Chassis_pub->publish(Chassis_info);

            /*-------------UL---------------------------------*/
            // refer_info.event_data=packet.event_data;
            // // refer_info.team=packet.team;
            // refer_info.time=packet.time;
            // // refer_info.race=packet.race;
            // refer_info.rfid=packet.rfid;
            // refer_info.base_hp=packet.base_hp;
            // refer_info.sentry_hp=packet.sentry_hp;
            // // refer_info.ballet_remain=packet.ballet_remain;
            // // refer_info.arm=packet.arm;
            // refer_info.outpost_hp=packet.outpost_hp;
            // refer_info.projectile_allowance_17mm=packet.projectile_allowance_17mm;
            // rfid_info.base_gain_point=1;
            // referee_pub->publish(refer_info);
            // std::cout<<packet.pitch<<std::endl;
            if (!initial_set_param_ || detect_color != previous_receive_color_)
            {
              setParam(rclcpp::Parameter("detect_color", detect_color));
              previous_receive_color_ = detect_color;
            }

            if (false)
            {
              resetTracker();
            }

            // 打印 data 结构体中的 xyz 和 yaw 值
            // std::cout << "xyz: (" << packet.aim_x << ", " << packet.aim_y << ", " << packet.aim_z << ")" << std::endl;
            // std::cout << "pitch: " << packet.pitch << "yaw: " << packet.yaw << std::endl;
            // RCLCPP_INFO(get_logger(), "CRC OK!");

            // //LOG [Receive] aim_xyz

            // RCLCPP_INFO(get_logger(), "[Receive] aim_x %f!", packet.aim_x);
            // RCLCPP_INFO(get_logger(), "[Receive] aim_y %f!", packet.aim_y);
            // RCLCPP_INFO(get_logger(), "[Receive] aim_z %f!", packet.aim_z);

            // // //LOG [Receive] [Receive] rpy
            // RCLCPP_INFO(get_logger(), "[Receive] roll %f!", packet.roll);
            // RCLCPP_INFO(get_logger(), "[Receive] pitch %f!", packet.pitch);
            // RCLCPP_INFO(get_logger(), "[Receive] yaw %f!", packet.yaw);

            sensor_msgs::msg::JointState joint_state;
            timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
            joint_state.header.stamp =
                this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
            joint_state.name.push_back("pitch_joint");
            joint_state.name.push_back("yaw_joint");

            // float temp_pitch = pitch_re_trans(packet.pitch);
            joint_state.position.push_back(packet.pitch);

            // float temp_yaw = yaw_re_trans(packet.yaw);
            joint_state.position.push_back(packet.yaw);
            joint_state_pub_->publish(joint_state);

            auto_aim_interfaces::msg::Velocity current_velocity;
            timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
            current_velocity.header.stamp =
                this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
            current_velocity.velocity = packet.current_v;
            velocity_pub_->publish(current_velocity);

            // 发布 TF 变换
            publishTransforms(Chassis_info.chassis_yaw_offset, joint_state.position[1]);

            // if (1) {
            //   // aiming_point_.header.stamp = this->now();
            //   // aiming_point_.pose.position.x = packet.aim_x;
            //   // aiming_point_.pose.position.y = packet.aim_y;
            //   // aiming_point_.pose.position.z = packet.aim_z;
            //   // marker_pub_->publish(aiming_point_);
            // }
          }
          else
          {
            RCLCPP_ERROR(get_logger(), "CRC error!");
          }
        }
        else
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
        }
      }
      catch (const std::exception &ex)
      {
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
        reopenPort();
      }
    }
  }

// 底盘 tf 变换
void RMSerialDriver::publishTransforms(double chassis_yaw_offset, double livox_yaw)
{
  rclcpp::Time now = this->now();

  geometry_msgs::msg::TransformStamped base_to_chassis;
  // 添加时间戳
  base_to_chassis.header.stamp = now;
  base_to_chassis.header.frame_id = "base_link";
  base_to_chassis.child_frame_id = "chassis";

  base_to_chassis.transform.translation.x = 0.0;
  base_to_chassis.transform.translation.y = 0.0;
  base_to_chassis.transform.translation.z = 0.0;

  tf2::Quaternion q_base_chassis;
  q_base_chassis.setRPY(0, 0, 0);
  base_to_chassis.transform.rotation.x = q_base_chassis.x();
  base_to_chassis.transform.rotation.y = q_base_chassis.y();
  base_to_chassis.transform.rotation.z = q_base_chassis.z();
  base_to_chassis.transform.rotation.w = q_base_chassis.w();

  // chassis_gimbal
  geometry_msgs::msg::TransformStamped chassis_to_gimbal;
  chassis_to_gimbal.header.stamp = now;
  chassis_to_gimbal.header.frame_id = "chassis";
  chassis_to_gimbal.child_frame_id = "gimbal_odom";

  chassis_to_gimbal.transform.translation.x = 0.0;
  chassis_to_gimbal.transform.translation.y = 0.0;
  chassis_to_gimbal.transform.translation.z = 0.35;

  tf2::Quaternion q_chassis_gimbal;
  q_chassis_gimbal.setRPY(0, 0, -livox_yaw+chassis_yaw_offset);
  chassis_to_gimbal.transform.rotation.x = q_chassis_gimbal.x();
  chassis_to_gimbal.transform.rotation.y = q_chassis_gimbal.y();
  chassis_to_gimbal.transform.rotation.z = q_chassis_gimbal.z();
  chassis_to_gimbal.transform.rotation.w = q_chassis_gimbal.w();


    tf_broadcaster_->sendTransform(base_to_chassis);
    tf_broadcaster_->sendTransform(chassis_to_gimbal);
  }

  void RMSerialDriver::sendData(const auto_aim_interfaces::msg::Send::SharedPtr msg)
  {
    const static std::map<std::string, uint8_t> id_unit8_map{
        {"", 0}, {"outpost", 0}, {"1", 1}, {"1", 1}, {"2", 2}, {"3", 3}, {"4", 4}, {"5", 5}, {"guard", 6}, {"base", 7}};

    try
    {
      SendPacket packet;

      // packet.is_fire = msg->is_fire;
      //  packet.x = msg->position.x;
      //  packet.y = msg->position.y;
      //  packet.z = msg->position.z;
      // packet.v_yaw = msg->v_yaw;

      if (msg->tracking == true)
      {
        packet.notice = (1 << 1);
      }
      // bool t=msg->tracking;
      packet.pitch = RMSerialDriver::pitch_trans(msg->pitch);
      packet.yaw = RMSerialDriver::pitch_trans(msg->yaw);
      // std::cout<<"-----------------------------"<<std::endl;
      //       std::cout<<"pitch:"<<packet.pitch<<std::endl;
      //       std::cout<<"yaw:"<<packet.yaw<<std::endl;
      // std::cout<<"------------------------------"<<std::endl;
      // std::cout<<"send pitch:"<<packet.gimbal.pit<<std::endl;
      packet.vx = -move_.vy;
      packet.vy = move_.vx;
      packet.wz /*= move_.wz */ = 0;
      packet.checksum = crc16::CRC16_Calc(reinterpret_cast<uint8_t *>(&packet), sizeof(packet) - sizeof(uint16_t), UINT16_MAX);

      // 打印 data 结构体中的 xyz 和 yaw 值
      // std::cout << "xyz: (" << packet.x << ", " << packet.y << ", " << packet.z << ")" << std::endl;
      // std::cout << "pitch: " << packet.pitch << "yaw: " << packet.yaw << std::endl;
      // RCLCPP_INFO(get_logger(), "[Send] aim_x %f!", packet.x);
      // RCLCPP_INFO(get_logger(), "[Send] aim_y %f!", packet.y);
      // RCLCPP_INFO(get_logger(), "[Send] aim_z %f!", packet.z);

      // RCLCPP_INFO(get_logger(), "-------------------------------------------------------------");
      // RCLCPP_INFO(get_logger(), "[Send] pitch %f!", packet.pitch);
      // RCLCPP_INFO(get_logger(), "[Send] yaw %f!", packet.yaw);
      // RCLCPP_INFO(get_logger(), "-------------------------------------------------------------");

      std::vector<uint8_t> data = toVector(packet);
      // for(int i=0;i<(int)data.size();i++)
      //   std::cout<<static_cast<int>(data[i])<<" ";
      // std::cout<<std::endl;
      serial_driver_->port()->send(data);

      std_msgs::msg::Float64 latency;
      latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
      RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
      latency_pub_->publish(latency);
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
      reopenPort();
    }
  }

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

    try
    {
      baud_rate = declare_parameter<int>("baud_rate", 0);
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
      throw ex;
    }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

      if (fc_string == "none")
      {
        fc = FlowControl::NONE;
      }
      else if (fc_string == "hardware")
      {
        fc = FlowControl::HARDWARE;
      }
      else if (fc_string == "software")
      {
        fc = FlowControl::SOFTWARE;
      }
      else
      {
        throw std::invalid_argument{
            "The flow_control parameter must be one of: none, software, or hardware."};
      }
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
      throw ex;
    }

    try
    {
      const auto pt_string = declare_parameter<std::string>("parity", "");

      if (pt_string == "none")
      {
        pt = Parity::NONE;
      }
      else if (pt_string == "odd")
      {
        pt = Parity::ODD;
      }
      else if (pt_string == "even")
      {
        pt = Parity::EVEN;
      }
      else
      {
        throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
      }
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
      throw ex;
    }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

void RMSerialDriver::setParam(const rclcpp::Parameter & param)
{
  if (!detector_param_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
    return;
  }

  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
    set_param_future_ = detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
        initial_set_param_ = true;
      });
  }
}

void RMSerialDriver::resetTracker()
{
  if (!reset_tracker_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_tracker_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Reset tracker!");
}

float RMSerialDriver::pitch_trans(float originAngle){

    if (originAngle < 0) {
      originAngle = originAngle +2* M_PI;
    }
    else 
      originAngle = originAngle;
    return originAngle;
    // return originAngle;
}


float RMSerialDriver::pitch_re_trans(float originAngle){

  if (originAngle <= M_PI) {
    originAngle = originAngle;
  }
  else
    originAngle = originAngle -2*M_PI;


      return originAngle;
  // return originAngle-M_PI;
}
float RMSerialDriver::yaw_trans(float originAngle){

    if (originAngle <=0) {
      originAngle = abs(originAngle);
    }
    else 
      originAngle = 2 * M_PI - originAngle;
    return originAngle;


}
float RMSerialDriver::yaw_re_trans(float originAngle){

    if (originAngle <= M_PI) {
      originAngle = -originAngle;
    }
    else 
      originAngle = 2*M_PI  -originAngle;
    return originAngle;


}


}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
