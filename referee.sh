#!/bin/bash
# filepath: /home/rm/qdu_sentry/referee.sh

# 设置随机数值范围
GAME_PROGRESS_RANGE=( 4 )  # 比赛状态：0-5
STAGE_REMAIN_TIME_RANGE=(0 600)    # 剩余时间：0-600秒
SENTRY_HP_RANGE=(0 600)            # 哨兵血量：0-600
BASE_HP_RANGE=(0 5000)             # 基地血量：0-5000
OUTPOST_HP_RANGE=(0 1500)          # 前哨站血量：0-1500
PROJECTILE_ALLOWANCE_RANGE=(0 500) # 弹药允许量：0-500
HERO_HP_RANGE=(0 200)              # 英雄血量：0-200
HERO_POSITION_X_RANGE=(-5 5)       # 英雄位置X：-5到5
HERO_POSITION_Y_RANGE=(-5 5)       # 英雄位置Y：-5到5

# 生成随机数函数
generate_random() {
    local min=$1
    local max=$2
    echo $((RANDOM % (max - min + 1) + min))
}

# 生成随机浮点数函数
generate_random_float() {
    local min=$1
    local max=$2
    echo "scale=2; $min + ($max - $min) * $RANDOM / 32767" | bc -l
}

# 随机选择数组元素
random_choice() {
    local arr=("$@")
    echo ${arr[$((RANDOM % ${#arr[@]}))]}
}

# 发布游戏状态话题
publish_game_status() {
    while true; do
        local game_progress=$(random_choice "${GAME_PROGRESS_RANGE[@]}")
        local stage_remain_time=$(generate_random ${STAGE_REMAIN_TIME_RANGE[0]} ${STAGE_REMAIN_TIME_RANGE[1]})
        
        ros2 topic pub --once /referee/game_status referee_interfaces/msg/GameStatus \
        "{game_progress: $game_progress, stage_remain_time: $stage_remain_time}"
        
        sleep 0.5
    done
}

# 发布基本血量话题
publish_basic_hp() {
    while true; do
        local sentry_hp=$(generate_random ${SENTRY_HP_RANGE[0]} ${SENTRY_HP_RANGE[1]})
        local base_hp=$(generate_random ${BASE_HP_RANGE[0]} ${BASE_HP_RANGE[1]})
        local outpost_hp=$(generate_random ${OUTPOST_HP_RANGE[0]} ${OUTPOST_HP_RANGE[1]})
        local projectile_allowance=$(generate_random ${PROJECTILE_ALLOWANCE_RANGE[0]} ${PROJECTILE_ALLOWANCE_RANGE[1]})
        
        ros2 topic pub --once /referee/basic_hp referee_interfaces/msg/BasicHp \
        "{sentry_hp: $sentry_hp, base_hp: $base_hp, outpost_hp: $outpost_hp, projectile_allowance_17mm: $projectile_allowance}"
        
        sleep 0.5
    done
}

# 发布友方机器人话题
publish_ally_bot() {
    while true; do
        local hero_hp=$(generate_random ${HERO_HP_RANGE[0]} ${HERO_HP_RANGE[1]})
        local hero_x=$(generate_random_float ${HERO_POSITION_X_RANGE[0]} ${HERO_POSITION_X_RANGE[1]})
        local hero_y=$(generate_random_float ${HERO_POSITION_Y_RANGE[0]} ${HERO_POSITION_Y_RANGE[1]})
        local standard_3_hp=$(generate_random ${HERO_HP_RANGE[0]} ${HERO_HP_RANGE[1]})
        local standard_3_x=$(generate_random_float ${HERO_POSITION_X_RANGE[0]} ${HERO_POSITION_X_RANGE[1]})
        local standard_3_y=$(generate_random_float ${HERO_POSITION_Y_RANGE[0]} ${HERO_POSITION_Y_RANGE[1]})
        local standard_4_hp=$(generate_random ${HERO_HP_RANGE[0]} ${HERO_HP_RANGE[1]})
        local standard_4_x=$(generate_random_float ${HERO_POSITION_X_RANGE[0]} ${HERO_POSITION_X_RANGE[1]})
        local standard_4_y=$(generate_random_float ${HERO_POSITION_Y_RANGE[0]} ${HERO_POSITION_Y_RANGE[1]})
        local engineer_hp=$(generate_random ${HERO_HP_RANGE[0]} ${HERO_HP_RANGE[1]})
        local engineer_x=$(generate_random_float ${HERO_POSITION_X_RANGE[0]} ${HERO_POSITION_X_RANGE[1]})
        local engineer_y=$(generate_random_float ${HERO_POSITION_Y_RANGE[0]} ${HERO_POSITION_Y_RANGE[1]})
        
        ros2 topic pub --once /referee/ally_bot referee_interfaces/msg/AllyBot \
        "{hero_hp: $hero_hp, hero_position: {x: $hero_x, y: $hero_y, z: 0.0}, \
        standard_3_hp: $standard_3_hp, standard_3_position: {x: $standard_3_x, y: $standard_3_y, z: 0.0}, \
        standard_4_hp: $standard_4_hp, standard_4_position: {x: $standard_4_x, y: $standard_4_y, z: 0.0}, \
        engineer_hp: $engineer_hp, engineer_position: {x: $engineer_x, y: $engineer_y, z: 0.0}}"
        
        sleep 0.5
    done
}

# 发布RFID话题
publish_rfid() {
    while true; do
        # 随机生成RFID状态（true/false）
        local base_gain=$([ $((RANDOM % 2)) -eq 0 ] && echo "true" || echo "false")
        local central_highland=$([ $((RANDOM % 2)) -eq 0 ] && echo "true" || echo "false")
        local enemy_central_highland=$([ $((RANDOM % 2)) -eq 0 ] && echo "true" || echo "false")
        local friendly_trapezoidal=$([ $((RANDOM % 2)) -eq 0 ] && echo "true" || echo "false")
        local enemy_trapezoidal=$([ $((RANDOM % 2)) -eq 0 ] && echo "true" || echo "false")
        local friendly_fly_front=$([ $((RANDOM % 2)) -eq 0 ] && echo "true" || echo "false")
        local friendly_fly_back=$([ $((RANDOM % 2)) -eq 0 ] && echo "true" || echo "false")
        local enemy_fly_front=$([ $((RANDOM % 2)) -eq 0 ] && echo "true" || echo "false")
        local enemy_fly_back=$([ $((RANDOM % 2)) -eq 0 ] && echo "true" || echo "false")
        local friendly_central_lower=$([ $((RANDOM % 2)) -eq 0 ] && echo "true" || echo "false")
        local friendly_central_upper=$([ $((RANDOM % 2)) -eq 0 ] && echo "true" || echo "false")
        local enemy_central_lower=$([ $((RANDOM % 2)) -eq 0 ] && echo "true" || echo "false")
        local enemy_central_upper=$([ $((RANDOM % 2)) -eq 0 ] && echo "true" || echo "false")
        local friendly_highway_lower=$([ $((RANDOM % 2)) -eq 0 ] && echo "true" || echo "false")
        local friendly_highway_upper=$([ $((RANDOM % 2)) -eq 0 ] && echo "true" || echo "false")
        local enemy_highway_lower=$([ $((RANDOM % 2)) -eq 0 ] && echo "true" || echo "false")
        local enemy_highway_upper=$([ $((RANDOM % 2)) -eq 0 ] && echo "true" || echo "false")
        local friendly_fortress=$([ $((RANDOM % 2)) -eq 0 ] && echo "true" || echo "false")
        local friendly_outpost=$([ $((RANDOM % 2)) -eq 0 ] && echo "true" || echo "false")
        local friendly_supply_non=$([ $((RANDOM % 2)) -eq 0 ] && echo "true" || echo "false")
        local friendly_supply_exchange=$([ $((RANDOM % 2)) -eq 0 ] && echo "true" || echo "false")
        local friendly_big_resource=$([ $((RANDOM % 2)) -eq 0 ] && echo "true" || echo "false")
        local enemy_big_resource=$([ $((RANDOM % 2)) -eq 0 ] && echo "true" || echo "false")
        local center_gain=$([ $((RANDOM % 2)) -eq 0 ] && echo "true" || echo "false")
        
        ros2 topic pub --once /referee/rfid referee_interfaces/msg/Rfid \
        "{base_gain_point: $base_gain, central_highland_gain_point: $central_highland, \
        enemy_central_highland_gain_point: $enemy_central_highland, \
        friendly_trapezoidal_highland_gain_point: $friendly_trapezoidal, \
        enemy_trapezoidal_highland_gain_point: $enemy_trapezoidal, \
        friendly_fly_ramp_front_gain_point: $friendly_fly_front, \
        friendly_fly_ramp_back_gain_point: $friendly_fly_back, \
        enemy_fly_ramp_front_gain_point: $enemy_fly_front, \
        enemy_fly_ramp_back_gain_point: $enemy_fly_back, \
        friendly_central_highland_lower_gain_point: $friendly_central_lower, \
        friendly_central_highland_upper_gain_point: $friendly_central_upper, \
        enemy_central_highland_lower_gain_point: $enemy_central_lower, \
        enemy_central_highland_upper_gain_point: $enemy_central_upper, \
        friendly_highway_lower_gain_point: $friendly_highway_lower, \
        friendly_highway_upper_gain_point: $friendly_highway_upper, \
        enemy_highway_lower_gain_point: $enemy_highway_lower, \
        enemy_highway_upper_gain_point: $enemy_highway_upper, \
        friendly_fortress_gain_point: $friendly_fortress, \
        friendly_outpost_gain_point: $friendly_outpost, \
        friendly_supply_zone_non_exchange: $friendly_supply_non, \
        friendly_supply_zone_exchange: $friendly_supply_exchange, \
        friendly_big_resource_island: $friendly_big_resource, \
        enemy_big_resource_island: $enemy_big_resource, \
        center_gain_point: $center_gain}"
        
        sleep 0.5
    done
}

# 主函数
main() {
    echo "开始发布裁判系统模拟数据..."
    echo "游戏状态范围: ${GAME_PROGRESS_RANGE[@]}"
    echo "剩余时间范围: ${STAGE_REMAIN_TIME_RANGE[@]}秒"
    echo "哨兵血量范围: ${SENTRY_HP_RANGE[@]}"
    echo "基地血量范围: ${BASE_HP_RANGE[@]}"
    echo "前哨站血量范围: ${OUTPOST_HP_RANGE[@]}"
    echo "弹药允许量范围: ${PROJECTILE_ALLOWANCE_RANGE[@]}"
    echo "机器人血量范围: ${HERO_HP_RANGE[@]}"
    echo "机器人位置X范围: ${HERO_POSITION_X_RANGE[@]}"
    echo "机器人位置Y范围: ${HERO_POSITION_Y_RANGE[@]}"
    echo ""
    echo "每0.5秒更新一次数据"
    echo "按 Ctrl+C 停止"
    
    # 在后台启动各个发布函数
    publish_game_status &
    publish_basic_hp &
    publish_ally_bot &
    publish_rfid &
    
    # 等待所有后台进程
    wait
}

# 捕获Ctrl+C信号，清理后台进程
trap 'echo "正在停止所有进程..."; kill $(jobs -p); exit' INT

# 检查bc命令是否存在（用于浮点数计算）
if ! command -v bc &> /dev/null; then
    echo "错误: 需要安装bc命令来处理浮点数计算"
    echo "请运行: sudo apt-get install bc"
    exit 1
fi

# 运行主函数
main