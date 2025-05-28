// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>


namespace rm_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 0x5A;
  struct __attribute__((packed))  {
    float yaw;
    float pit;
    float rol;
  } eulr;
  float chassis_yaw_offset_;
  uint8_t notice;
  float current_v; // m/s
  float yaw;
  float pitch;
  float roll;

  //uint32_t  event_data; // 重要事件数据        
  uint8_t time;//
  

  uint32_t rfid;        //增益 
  uint16_t base_hp;      //基地血量
  uint16_t sentry_hp;     //哨兵血量       
  uint16_t  outpost_hp; //前哨战
  uint16_t  projectile_allowance_17mm;    //允许发弹量
  uint8_t recovery_buff;           //# 机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）
  uint8_t cooling_buff;            //# 机器人射击热量冷却倍率（直接值，值为 5 表示 5 倍冷却）
  uint8_t defence_buff;            //# 机器人防御增益（百分比，值为 50 表示 50% 防御增益）
  uint8_t vulnerability_buff;      //# 机器人负防御增益（百分比，值为 30 表示 -30% 防御增益）
  uint16_t attack_buff;           //# 机器人攻击增益（百分比，值为 50 表示 50% 攻击增益）
  uint8_t remaining_energy;       // # 机器人剩余能量值反馈，以 16 进制标识机器人剩余能量值比例，仅在机器人剩余能量小于 50% 时反馈，其余默认反馈 0x32。
  
  uint16_t hero_hp;
  uint16_t standard_3_hp;
  uint16_t standard_4_hp;
  uint16_t engineer_hp;



  float hero_position_x;
  float hero_position_y;
  float standard_3_position_x;
  float standard_3_position_y;
  float standard_4_position_x;
  float standard_4_position_y;
  float engineer_position_x;
  float engineer_position_y;
  uint8_t damaged_armor_id; /* 受击装甲ID */

  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket
{
  //uint8_t header = 0xA5;
   /* 控制命令 */
  float yaw; 
  float pitch;
  float roll=0;   
  uint8_t notice=(1<<1);  
  float vx; 
  float vy; 
  
  float wz = 0; 
  uint16_t checksum = 0;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
