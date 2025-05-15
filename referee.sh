#!/bin/bash

# 初始化变量
time_value=30
game_status=4  # 4 表示比赛开始
sentry_hp=100
hero_hp=100
engineer_hp=100
standard3_hp=100
standard4_hp=100
outpost_hp=300
base_hp=1500

# 友方机器人位置 (初始值)
hero_x=1.5
hero_y=2.0
hero_z=0.0
engineer_x=-1.0
engineer_y=3.5
engineer_z=0.0
standard3_x=2.5
standard3_y=-1.0
standard3_z=0.0
standard4_x=3.0
standard4_y=4.0
standard4_z=0.0

# 无限循环，每秒发布一次消息
while true; do 
  # 随机变化哨兵血量
  sentry_hp=$(( RANDOM % 21 + 90 ))
  
  # 随机变化友方机器人位置
  hero_x=$(echo "scale=2; $hero_x + $(echo "scale=2; $(( RANDOM % 40 - 20 )) / 100" | bc)" | bc)
  hero_y=$(echo "scale=2; $hero_y + $(echo "scale=2; $(( RANDOM % 40 - 20 )) / 100" | bc)" | bc)
  engineer_x=$(echo "scale=2; $engineer_x + $(echo "scale=2; $(( RANDOM % 40 - 20 )) / 100" | bc)" | bc)
  engineer_y=$(echo "scale=2; $engineer_y + $(echo "scale=2; $(( RANDOM % 40 - 20 )) / 100" | bc)" | bc)
  standard3_x=$(echo "scale=2; $standard3_x + $(echo "scale=2; $(( RANDOM % 40 - 20 )) / 100" | bc)" | bc)
  standard3_y=$(echo "scale=2; $standard3_y + $(echo "scale=2; $(( RANDOM % 40 - 20 )) / 100" | bc)" | bc)
  standard4_x=$(echo "scale=2; $standard4_x + $(echo "scale=2; $(( RANDOM % 40 - 20 )) / 100" | bc)" | bc)
  standard4_y=$(echo "scale=2; $standard4_y + $(echo "scale=2; $(( RANDOM % 40 - 20 )) / 100" | bc)" | bc)
  
  # 打印日志
  echo "Publishing: sentry_hp=$sentry_hp, time=$time_value, game_status=$game_status"
  echo "Hero position: ($hero_x, $hero_y, $hero_z)"
  
  # 根据时间情况更新比赛状态
  if [ $time_value -gt 0 ]; then
    time_value=$((time_value - 1))
  else
    # 时间结束，比赛结束
    game_status=5  # 5 表示比赛结束
  fi

  # 发送 BasicHp 消息
  ros2 topic pub --once /referee/basic_hp referee_interfaces/msg/BasicHp "{
    sentry_hp: $sentry_hp,
    outpost_hp: $outpost_hp,
    base_hp: $base_hp
  }" --qos-reliability reliable

  # 发送 GameStatus 消息
  ros2 topic pub --once /referee/game_status referee_interfaces/msg/GameStatus "{
    game_status: $game_status,
    game_type: 1,
    game_progress: 4,
    stage_remain_time: $time_value
  }" --qos-reliability reliable

  # 发送 Rfid 消息
  rfid_status=$((RANDOM % 2))  # 随机 0 或 1
  ros2 topic pub --once /referee/rfid referee_interfaces/msg/Rfid "{
    rfid_status: [$rfid_status, 0, 0, 0, 0, 0, 0, 0]
  }" --qos-reliability reliable

  # 发送 AllyBot 消息
  ros2 topic pub --once /referee/ally_bot referee_interfaces/msg/AllyBot "{
    hero_position: {x: $hero_x, y: $hero_y, z: $hero_z},
    engineer_position: {x: $engineer_x, y: $engineer_y, z: $engineer_z},
    standard_3_position: {x: $standard3_x, y: $standard3_y, z: $standard3_z},
    standard_4_position: {x: $standard4_x, y: $standard4_y, z: $standard4_z}
  }" --qos-reliability reliable

  # 发送 EnemyStatus 消息
  ros2 topic pub --once /referee/enemy_status referee_interfaces/msg/EnemyStatus "{
    hero_hp: $hero_hp,
    engineer_hp: $engineer_hp,
    standard_3_hp: $standard3_hp,
    standard_4_hp: $standard4_hp
  }" --qos-reliability reliable

  # 等待 1 秒
  sleep 1
done
