// Copyright 2025 Jquark
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

#include "lidarscan/lidarscan.hpp"

namespace lidarscan
{
    LidarscanNode::LidarscanNode(const rclcpp::NodeOptions &options)
        : Node("lidarscan_node", options),
          tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
          current_state_(SPIN),
          current_yaw_(0.0),
          current_pitch_(0.0),
          target_yaw_(0.0),
          target_pitch_(0.0),
          tracking_mode_(false),
          last_yaw_cmd_(0.0),
          last_pitch_cmd_(0.0),
          scan_phase_(0.0),
          current_period(0),
          damaged_armor_detected_(false),
          enemy_detected_(false),
          dt_(0.0),
          chassis_yaw_offset_(0.0),
          closest_obstacle_yaw_(0.0),
          closest_obstacle_distance_(M_PI),
          tracking_lose_yaw_(0.0),
          logger_(get_logger())
    {

        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("gimbal_frame", "livox_frame");

        this->declare_parameter<std::string>("terrain_map_topic", "/terrain_map");
        this->declare_parameter<std::string>("gimbal_cmd_topic", "/gimbalcontrol");
        this->declare_parameter<std::string>("tracker_topic", "/tracker/send");
        this->declare_parameter<std::string>("armors_topic", "/detector/armors");
        this->declare_parameter<std::string>("chassis_topic", "/Chassis");

        this->declare_parameter("min_bounding_box", 0.3);
        this->declare_parameter("max_bounding_box", 1.5);
        this->declare_parameter("max_scan_period", 5);  // 最大扫描周期数
        this->declare_parameter("min_scan_points", 20); // 最小扫描点数

        // 云台控制参数
        this->declare_parameter("spin_yaw_speed", 0.5);
        this->declare_parameter("spin_pitch_speed", 0.5);
        this->declare_parameter("scan_yaw_angle", 0.2);
        this->declare_parameter("max_pitch", 0.46);
        this->declare_parameter("min_pitch", -0.39);
        this->declare_parameter("max_yaw_speed", 2.0);
        this->declare_parameter("pitch_more_speed", 2.0);
        this->declare_parameter("pitch_less_speed", 0.4);
        this->declare_parameter("scan_theta", 1.57);

        // 平滑处理参数
        this->declare_parameter("yaw_smooth_factor", 0.3);
        this->declare_parameter("pitch_smooth_factor", 0.3);
        this->declare_parameter("smooth_window_size", 5);
        this->declare_parameter("state_switch_threshold", 0.5);

        // 状态切换控制参数
        this->declare_parameter("max_switch_count", 5);        // 最大切换次数
        this->declare_parameter("hysteresis_threshold", 0.15); // 迟滞阈值
        this->declare_parameter("transition_speed", 0.05);     // 过渡速度

        // 自瞄和装甲板参数
        this->declare_parameter<std::vector<int64_t>>("armor_id", std::vector<int64_t>{1, 2, 3, 4, 5, 6, 7, 8, 9});
        this->declare_parameter<double>("max_distance", 8.0);
        this->declare_parameter<double>("transition_delay", 3.0);

        this->get_parameter("base_frame", base_frame_);
        this->get_parameter("gimbal_frame", gimbal_frame_);

        // 获取话题参数
        this->get_parameter("terrain_map_topic", terrain_map_topic_);
        this->get_parameter("gimbal_cmd_topic", gimbal_cmd_topic_);
        this->get_parameter("tracker_topic", tracker_topic_);
        this->get_parameter("armors_topic", armors_topic_);
        this->get_parameter("chassis_topic", chassis_topic_);

        // 获取障碍物参数
        this->get_parameter("min_bounding_box", min_bounding_box);
        this->get_parameter("max_bounding_box", max_bounding_box);
        this->get_parameter("max_scan_period", max_scan_period_);
        this->get_parameter("min_scan_points", min_scan_points_);
        this->get_parameter("scan_theta", scan_theta_);

        // 获取控制参数
        this->get_parameter("spin_yaw_speed", spin_yaw_speed_);
        this->get_parameter("spin_pitch_speed", spin_pitch_speed_);
        this->get_parameter("scan_yaw_angle", scan_yaw_angle_);
        this->get_parameter("max_pitch", max_pitch_);
        this->get_parameter("min_pitch", min_pitch_);
        this->get_parameter("max_yaw_speed", max_yaw_speed_);
        this->get_parameter("pitch_more_speed", pitch_more_speed_);
        this->get_parameter("pitch_less_speed", pitch_less_speed_);

        // 获取平滑参数
        this->get_parameter("yaw_smooth_factor", yaw_smooth_factor_);
        this->get_parameter("pitch_smooth_factor", pitch_smooth_factor_);
        this->get_parameter("smooth_window_size", smooth_window_size_);
        this->get_parameter("state_switch_threshold", state_switch_threshold_);

        // 获取状态切换参数
        this->get_parameter("max_switch_count", max_switch_count_);
        this->get_parameter("hysteresis_threshold", hysteresis_threshold_);
        this->get_parameter("transition_speed", transition_speed_);

        // 获取自瞄和装甲板参数
        expected_armor_ids_ = this->get_parameter("armor_id").as_integer_array();
        max_distance_ = this->get_parameter("max_distance").as_double();
        scan_transition_delay_ = this->get_parameter("transition_delay").as_double();

        // 使用配置的话题名称创建订阅和发布
        terrainmap_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            terrain_map_topic_, rclcpp::SensorDataQoS(),
            std::bind(&LidarscanNode::terrainmapCallback, this, std::placeholders::_1));

        gimbal_cmd_pub_ = this->create_publisher<auto_aim_interfaces::msg::Send>(
            gimbal_cmd_topic_, rclcpp::SensorDataQoS());

        // 自瞄订阅
        tracker_sub_ = this->create_subscription<auto_aim_interfaces::msg::Send>(
            tracker_topic_, rclcpp::SensorDataQoS(),
            std::bind(&LidarscanNode::trackerCallback, this, std::placeholders::_1));

        // 装甲板订阅
        armors_sub_ = this->create_subscription<auto_aim_interfaces::msg::Armors>(
            armors_topic_, rclcpp::SensorDataQoS(),
            std::bind(&LidarscanNode::armorsCallback, this, std::placeholders::_1));

        // 提高计时器频率到100Hz (每10ms执行一次)
        scan_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&LidarscanNode::scanTimerCallback, this));

        // Chassis订阅
        chassis_sub_ = this->create_subscription<auto_aim_interfaces::msg::Chassis>(
            chassis_topic_, rclcpp::SensorDataQoS(),
            std::bind(&LidarscanNode::chassisCallback, this, std::placeholders::_1));

        tf_buffer_->setUsingDedicatedThread(true);

        for (int i = 0; i < smooth_window_size_; ++i)
        {
            yaw_cmd_queue_.push(0.0);
            pitch_cmd_queue_.push(0.0);
        }

        // 初始化时间
        last_time_ = this->now();
        last_tracker_time_ = this->now();
        last_armor_time_ = this->now();
        mode_switch_time_ = this->now();

        RCLCPP_INFO(logger_, "点云订阅: %s, 自瞄: %s, 装甲板: %s, 底盘: %s",
                    terrain_map_topic_.c_str(), tracker_topic_.c_str(),
                    armors_topic_.c_str(), chassis_topic_.c_str());
        RCLCPP_INFO(logger_, "云台控制发布至: %s", gimbal_cmd_topic_.c_str());
    };

    // 受击反馈装甲板计算
    double LidarscanNode::calculateArmorYaw(uint8_t armor_id)
    {
        // 装甲板相对于底盘的角度
        double armor_relative_yaw = 0.0;
        switch (armor_id)
        {
        case 1:
            armor_relative_yaw = 0.0; // 正前方
            break;
        case 2:
            armor_relative_yaw = M_PI_2; // 左侧 (逆时针90°)
            break;
        case 3:
            armor_relative_yaw = M_PI; // 后方 (逆时针180°)
            break;
        case 4:
            armor_relative_yaw = -M_PI_2; // 右侧 (逆时针270°, 即顺时针90°)
            break;
        default:
            armor_relative_yaw = 0.0; // 默认值，不应该发生
            break;
        }

        double gimbal_offset = -current_yaw_ + chassis_yaw_offset_;

        // 计算云台坐标系下的目标角度
        double target_yaw = armor_relative_yaw + gimbal_offset;

        // 归一化到 [-π, π] 范围
        while (target_yaw > M_PI)
            target_yaw -= 2 * M_PI;
        while (target_yaw < -M_PI)
            target_yaw += 2 * M_PI;

        return target_yaw;
    }

    // Chassis回调函数
    void LidarscanNode::chassisCallback(const auto_aim_interfaces::msg::Chassis::SharedPtr msg)
    {
        chassis_yaw_offset_ = msg->chassis_yaw_offset;

        if (msg->damaged_armor_id > 0 && msg->damaged_armor_id <= 4)
        {
            damaged_armor_detected_ = true;
            current_damaged_armor_id_ = msg->damaged_armor_id;
            target_yaw_ = calculateArmorYaw(current_damaged_armor_id_);

            if (current_state_ != DAMAGED_SCAN)
            {
                current_state_ = DAMAGED_SCAN;
                scan_phase_ = 0.0;
                current_period = max_scan_period_;
                RCLCPP_INFO(logger_, "检测到受损装甲板ID: %d, 目标yaw: %.2f, 切换到DAMAGED_SCAN状态",
                            current_damaged_armor_id_, target_yaw_);
            }
        }
        else if (damaged_armor_detected_)
        {
            damaged_armor_detected_ = false;
            if (current_state_ == DAMAGED_SCAN)
            {
                current_state_ = SPIN;
                RCLCPP_INFO(logger_, "切换回SPIN状态");
            }
        }
    }

    void LidarscanNode::terrainmapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.05f, 0.05f, 0.05f);
        vg.filter(*cloud_filtered);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.2);
        ec.setMinClusterSize(min_scan_points_);
        ec.setMaxClusterSize(10000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_filtered);
        ec.extract(cluster_indices);

        std::vector<double> new_obstacle_yaws;

        for (const auto &indices : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto &index : indices.indices)
            {
                obstacle_cloud->push_back((*cloud_filtered)[index]);
            }

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*obstacle_cloud, centroid);

            double yaw = std::atan2(centroid[1], centroid[0]);

            Eigen::Vector4f min_pt, max_pt;
            pcl::getMinMax3D(*obstacle_cloud, min_pt, max_pt);
            float obstacle_size = (max_pt - min_pt).norm();

            // 合并距离过小的障碍物
            if (obstacle_size >= min_bounding_box && obstacle_size <= max_bounding_box)
            {
                bool merged = false;
                double merge_threshold = scan_theta_ / 2.0;
                for (auto &obs_yaw : new_obstacle_yaws)
                {
                    double yaw_diff = std::abs(obs_yaw - yaw);
                    // 处理角度跨越 PI/-PI 边界的情况
                    if (yaw_diff > M_PI)
                        yaw_diff = 2 * M_PI - yaw_diff;

                    if (yaw_diff < merge_threshold)
                    {
                        obs_yaw = (obs_yaw + yaw) / 2.0;
                        // 确保合并后的角度在 [-PI, PI] 范围内
                        if (obs_yaw > M_PI)
                            obs_yaw -= 2 * M_PI;
                        else if (obs_yaw < -M_PI)
                            obs_yaw += 2 * M_PI;

                        merged = true;
                        break;
                    }
                }
                if (!merged)
                {
                    new_obstacle_yaws.push_back(yaw);
                }
            }
        }

        std::sort(new_obstacle_yaws.begin(), new_obstacle_yaws.end());

        if (new_obstacle_yaws.size() != obstacle_yaws_.size())
        {
            RCLCPP_INFO(logger_, "分割出障碍物总数: %ld", new_obstacle_yaws.size());
        }

        obstacle_yaws_ = new_obstacle_yaws;
    }

    void LidarscanNode::trackerCallback(const auto_aim_interfaces::msg::Send::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_tracker_msg_ = *msg;
        last_tracker_time_ = this->now();

        // RCLCPP_INFO(logger_, "自瞄 yaw: %.2f, pitch: %.2f, tracking: %d",
        //              msg->yaw, msg->pitch, msg->tracking);
    }

    void LidarscanNode::armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr msg)
    {
        bool detected = false;

        if (!msg->armors.empty())
        {
            for (const auto &armor : msg->armors)
            {
                float distance_to_enemy = std::hypot(armor.pose.position.x, armor.pose.position.y);

                if (armor.number.empty())
                {
                    continue;
                }

                try
                {
                    int armor_id = std::stoi(armor.number);
                    const bool is_armor_id_match =
                        std::find(expected_armor_ids_.begin(), expected_armor_ids_.end(), armor_id) !=
                        expected_armor_ids_.end();

                    const bool is_within_distance = (distance_to_enemy <= max_distance_);

                    if (is_armor_id_match && is_within_distance)
                    {
                        detected = true;
                        break;
                    }
                }
                catch (const std::exception &e)
                {
                    RCLCPP_WARN(logger_, "装甲板错误: %s", armor.number.c_str());
                }
            }
        }

        std::lock_guard<std::mutex> lock(data_mutex_);
        enemy_detected_ = detected;
        last_armor_time_ = this->now();
    }

    void LidarscanNode::scanTimerCallback()
    {
        auto current_time = this->now();
        dt_ = (current_time - last_time_).seconds();
        last_time_ = current_time;

        if (dt_ > 0.1)
        {
            dt_ = 0.01;
        }

        std::lock_guard<std::mutex> lock(data_mutex_);

        checkStateTransition();

        updateGimbalCommand();

        gimbalsend();
    }

    void LidarscanNode::checkStateTransition()
    {
        // tracker有效期
        bool is_tracker_valid = last_tracker_msg_.has_value() &&
                                (this->now() - last_tracker_time_).seconds() < scan_transition_delay_;

        // 用于跟踪上一帧是否检测到目标
        static bool prev_tracking_state = false;
        scan_state new_state = current_state_;
        static rclcpp::Time last_state_change_time = this->now();

        // 检测到敌人并且不在跟踪状态，切换到跟踪状态
        if (enemy_detected_ && is_tracker_valid && last_tracker_msg_->tracking &&
            current_state_ != TRACKING)
        {
            RCLCPP_INFO(logger_, " -> TRACKING (检测到敌人)");
            mode_switch_time_ = this->now();
            state_switch_counter_ = 0;
            tracking_exit_time_ = this->now();
            current_state_ = TRACKING;
            last_state_change_time = this->now();
            prev_tracking_state = true;
            return;
        }

        if (current_state_ == TRACKING)
        {
            // 确保在开始跟踪后等待一定时间，防止频繁状态切换
            if ((this->now() - mode_switch_time_).seconds() < 3.0)
            {
                return;
            }

            bool current_tracking = enemy_detected_ && is_tracker_valid && last_tracker_msg_->tracking;

            // 当前帧丢失目标，但上一帧有目标，重置退出计时器
            if (!current_tracking && prev_tracking_state)
            {
                tracking_exit_time_ = this->now();
                // 在失去目标时记录当前的yaw值
                if (current_yaw_ != 0.0) // 添加判断，确保不赋值为0
                {
                    tracking_lose_yaw_ = current_yaw_;
                    RCLCPP_INFO(logger_, "失去目标, TRACKING 计时, 记录yaw: %.2f", tracking_lose_yaw_);
                }
            }

            // 更新跟踪状态
            prev_tracking_state = current_tracking;

            // 如果当前无法跟踪目标
            if (!current_tracking)
            {
                double time_since_exit = (this->now() - tracking_exit_time_).seconds();

                // 如果等待时间已到，切换状态
                if (time_since_exit >= scan_transition_delay_)
                {
                    // 根据当前状态决定下一个状态
                    if (damaged_armor_detected_)
                    {
                        new_state = DAMAGED_SCAN;
                        RCLCPP_INFO(logger_, "TRACKING -> DAMAGED_SCAN (等待时间结束: %.1f秒)", time_since_exit);
                    }
                    else if (!obstacle_yaws_.empty() && isYawNearObstacle())
                    {
                        new_state = SCAN;
                        RCLCPP_INFO(logger_, "TRACKING -> SCAN (%.1fs)", time_since_exit);
                    }
                    else
                    {
                        new_state = SPIN;
                        RCLCPP_INFO(logger_, "TRACKING -> SPIN (%.1fs), 当前yaw: %.2f", time_since_exit, current_yaw_);
                    }
                }
                else
                {
                    // 在等待期间，如果重新发现敌人
                    if (enemy_detected_ && is_tracker_valid && last_tracker_msg_->tracking)
                    {
                        RCLCPP_INFO(logger_, "TRACKING  (重置等待时间)");
                        prev_tracking_state = true;
                        return; // 继续保持 TRACKING
                    }

                    // 等待期间且没有重新发现敌人，输出等待进度
                    RCLCPP_INFO(logger_, "TRACKING 等待: %.1f/%.1f秒 yaw:%.2f",
                                time_since_exit, scan_transition_delay_, current_yaw_);
                }
            }
        }
        // 判断其他状态转换条件
        else if (damaged_armor_detected_)
        {
            if (current_state_ != DAMAGED_SCAN)
            {
                RCLCPP_INFO(logger_, " -> DAMAGED_SCAN");
                current_period = 0;
                state_switch_counter_ = 0;
            }
            new_state = DAMAGED_SCAN;
        }
        else if (!obstacle_yaws_.empty())
        {
            bool is_near_obstacle = isYawNearObstacle();
            if (current_state_ == SPIN && is_near_obstacle)
            {
                if (closest_obstacle_distance_ < (state_switch_threshold_ - hysteresis_threshold_))
                {
                    if (current_state_ != SCAN)
                    {
                        RCLCPP_INFO(logger_, " -> SCAN (距离障碍物: %.3f)", closest_obstacle_distance_);
                        current_period = 0;
                        state_switch_counter_ = 0;
                        in_transition_to_scan_ = true;
                        transition_progress_ = 0.0;
                    }
                    new_state = SCAN;
                }
            }
            else if (current_state_ == SCAN && (!is_near_obstacle || current_period >= max_scan_period_))
            {
                if (!is_near_obstacle && closest_obstacle_distance_ > (state_switch_threshold_ + hysteresis_threshold_))
                {
                    RCLCPP_INFO(logger_, "SCAN -> SPIN (障碍物距离增加: %.3f), 当前yaw: %.2f",
                                closest_obstacle_distance_, current_yaw_);
                    new_state = SPIN;
                    // 保留当前yaw，使状态切换更平滑
                }
                else if (current_period >= max_scan_period_)
                {
                    RCLCPP_INFO(logger_, "SCAN -> SPIN (扫描周期结束，共 %d 个周期), 当前yaw: %.2f", max_scan_period_, current_yaw_);
                    new_state = SPIN;
                    // 保留当前yaw，使状态切换更平滑
                }
            }
        }
        else
        {
            if (current_state_ != SPIN)
            {
                RCLCPP_INFO(logger_, " -> SPIN (无障碍物), 当前yaw: %.2f", current_yaw_);
            }
            new_state = SPIN;
        }

        // 实际执行状态切换
        if (new_state != current_state_)
        {
            // 检查频繁切换
            if ((new_state == SPIN && current_state_ == SCAN) ||
                (new_state == SCAN && current_state_ == SPIN))
            {
                state_switch_counter_++;
                if (state_switch_counter_ > max_switch_count_)
                {
                    RCLCPP_WARN(logger_, "状态切换过于频繁 (%d 次)，保持当前状态: %d",
                                state_switch_counter_, current_state_);
                    scan_transition_delay_ *= 1.5; // 增加冷却时间
                    state_switch_counter_ = 0;
                    return;
                }
            }

            // 在 TRACKING 状态超时后切换到 SPIN 状态时，使用 tracking_lose_yaw_ 避免跳变到 0.0
            if (new_state == SPIN && current_state_ == TRACKING)
            {
                // 确保 tracking_lose_yaw_ 非 0 时才覆盖 current_yaw_
                if (tracking_lose_yaw_ != 0.0)
                {
                    current_yaw_ = tracking_lose_yaw_;
                    RCLCPP_INFO(logger_, "TRACKING -> SPIN (%.1fs), 使用记录的yaw: %.2f", time_since_exit, current_yaw_);
                }
                else
                {
                    RCLCPP_WARN(logger_, "TRACKING -> SPIN (%.1fs), tracking_lose_yaw_ 为 0，保持当前yaw: %.2f", time_since_exit, current_yaw_);
                }
            }

            current_state_ = new_state;
            last_state_change_time = this->now();
        }
    }

    bool LidarscanNode::isYawNearObstacle()
    {
        bool near_obstacle = false;
        closest_obstacle_distance_ = M_PI; // 初始化为最大可能值

        for (const auto &obstacle_yaw : obstacle_yaws_)
        {
            // 计算角度差，并标准化到[-π, π]
            double yaw_diff = obstacle_yaw - current_yaw_;
            while (yaw_diff > M_PI)
                yaw_diff -= 2 * M_PI;
            while (yaw_diff < -M_PI)
                yaw_diff += 2 * M_PI;

            double abs_diff = std::abs(yaw_diff);
            if (abs_diff < closest_obstacle_distance_)
            {
                closest_obstacle_distance_ = abs_diff;
                closest_obstacle_yaw_ = obstacle_yaw;
            }

            if (abs_diff < state_switch_threshold_)
            {
                near_obstacle = true;
            }
        }

        return near_obstacle;
    }

    double LidarscanNode::updateSmoothedValue(std::queue<double> &cmd_queue, double new_value)
    {

        cmd_queue.push(new_value);
        cmd_queue.pop();

        double sum = 0.0;
        std::queue<double> temp_queue = cmd_queue;

        while (!temp_queue.empty())
        {
            sum += temp_queue.front();
            temp_queue.pop();
        }

        return sum / smooth_window_size_;
    }

    // 统一的云台发送函数
    void LidarscanNode::gimbalsend()
    {
        if (current_state_ == TRACKING && last_tracker_msg_.has_value())
        {
            gimbal_cmd_pub_->publish(last_tracker_msg_.value());

            last_yaw_cmd_ = last_tracker_msg_->yaw;
            last_pitch_cmd_ = last_tracker_msg_->pitch;

            updateSmoothedValue(yaw_cmd_queue_, last_tracker_msg_->yaw);
            updateSmoothedValue(pitch_cmd_queue_, last_tracker_msg_->pitch);

            // RCLCPP_INFO(logger_, "自瞄");
            return;
        }

        double smoothed_yaw = updateSmoothedValue(yaw_cmd_queue_, target_yaw_);
        double smoothed_pitch = updateSmoothedValue(pitch_cmd_queue_, target_pitch_);

        double final_yaw = last_yaw_cmd_ + yaw_smooth_factor_ * (smoothed_yaw - last_yaw_cmd_);
        double final_pitch = last_pitch_cmd_ + pitch_smooth_factor_ * (smoothed_pitch - last_pitch_cmd_);

        last_yaw_cmd_ = final_yaw;
        last_pitch_cmd_ = final_pitch;

        auto_aim_interfaces::msg::Send cmd_msg;
        cmd_msg.yaw = final_yaw;
        cmd_msg.pitch = final_pitch;
        cmd_msg.tracking = (current_state_ == SCAN || current_state_ == DAMAGED_SCAN);
        gimbal_cmd_pub_->publish(cmd_msg);
    }

    void LidarscanNode::updateGimbalCommand()
    {
        switch (current_state_)
        {
        case SPIN:
            spinState();
            break;
        case SCAN:
            scanState();
            break;
        case DAMAGED_SCAN:
            damagedScanState();
            break;
        case TRACKING:
            trackingState();
            break;
        }
    }

    void LidarscanNode::trackingState()
    {
        if (last_tracker_msg_.has_value() && !(last_tracker_msg_->yaw))
        {
            current_yaw_ = last_tracker_msg_->yaw;
            current_pitch_ = last_tracker_msg_->pitch;
        }
    }

    void LidarscanNode::spinState()
    {
        static double pitch_direction = 1.0;
        // 检查是否刚从TRACKING状态切换过来，如果是则使用记录的yaw值
        static scan_state last_state = SPIN; 
        last_state = current_state_;

        // 云台Yaw方向旋转相关的处理
        current_pitch_ += pitch_direction * spin_pitch_speed_ * dt_;

        if (current_pitch_ >= max_pitch_)
        {
            current_pitch_ = max_pitch_;
            pitch_direction = -1.0;
        }
        else if (current_pitch_ <= min_pitch_)
        {
            current_pitch_ = min_pitch_;
            pitch_direction = 1.0;
        }

        double yaw_speed_factor = current_pitch_ < 0 ? pitch_less_speed_ : pitch_more_speed_;
        // 确保无论俯仰角如何，yaw都以正确的方向和速度旋转
        current_yaw_ += spin_yaw_speed_ * dt_ * yaw_speed_factor;
        if (current_yaw_ > M_PI)
        {
            current_yaw_ -= 2 * M_PI;
        }
        else if (current_yaw_ < -M_PI)
        {
            current_yaw_ += 2 * M_PI;
        }

        target_yaw_ = current_yaw_;
        target_pitch_ = current_pitch_;
    }

    void LidarscanNode::scanState()
    {
        static double pitch_direction = 1.0;
        static double yaw_direction = 1.0;
        static bool at_boundary = false;

        // 处理上下俯仰
        current_pitch_ += pitch_direction * spin_pitch_speed_ * dt_;
        if (current_pitch_ >= max_pitch_)
        {
            current_pitch_ = max_pitch_;
            pitch_direction = -1.0;
        }
        else if (current_pitch_ <= min_pitch_)
        {
            current_pitch_ = min_pitch_;
            pitch_direction = 1.0;
        }

        // 定义扫描的角度范围
        double yaw_limit_max = closest_obstacle_yaw_ + scan_theta_ / 2;
        double yaw_limit_min = closest_obstacle_yaw_ - scan_theta_ / 2;

        if (in_transition_to_scan_)
        {

            transition_progress_ += transition_speed_ * dt_;

            if (transition_progress_ >= 1.0)
            {
                transition_progress_ = 1.0;
                in_transition_to_scan_ = false;
                RCLCPP_INFO(logger_, "SPIN到SCAN过渡完成");
            }


            double spin_weight = 1.0 - transition_progress_;
            double scan_weight = transition_progress_;

            double spin_yaw_delta = spin_yaw_speed_ * dt_ * 1.5;

            double target_scan_yaw;

            if (current_yaw_ < closest_obstacle_yaw_)
            {
                target_scan_yaw = yaw_limit_min;
                yaw_direction = -1.0;
            }
            else
            {
                target_scan_yaw = yaw_limit_max;
                yaw_direction = 1.0;
            }

            double yaw_to_target = target_scan_yaw - current_yaw_;
            while (yaw_to_target > M_PI)
                yaw_to_target -= 2 * M_PI;
            while (yaw_to_target < -M_PI)
                yaw_to_target += 2 * M_PI;
            double scan_yaw_delta = yaw_to_target * dt_ * 2.0;
            double yaw_delta = spin_yaw_delta * spin_weight + scan_yaw_delta * scan_weight;
            current_yaw_ += yaw_delta;
        }
        else
        {
            double yaw_speed = spin_yaw_speed_ * dt_;
            double yaw_speed_factor = current_pitch_ < 0 ? pitch_less_speed_ : pitch_more_speed_;
            double new_yaw = current_yaw_ + yaw_direction * yaw_speed * yaw_speed_factor;
            if (new_yaw > yaw_limit_max)
            {
                if (!at_boundary)
                {
                    RCLCPP_INFO(logger_, "扫描障碍物 角度: %.2f, 到达右边界, 进度: %d/%d",
                                closest_obstacle_yaw_, current_period, max_scan_period_);
                    current_period++;
                    at_boundary = true;
                }
                new_yaw = yaw_limit_max;
                yaw_direction = -1.0;
            }
            else if (new_yaw < yaw_limit_min)
            {
                if (!at_boundary)
                {
                    RCLCPP_INFO(logger_, "扫描障碍物 角度: %.2f, 到达左边界, 进度: %d/%d",
                                closest_obstacle_yaw_, current_period, max_scan_period_);
                    current_period++;
                    at_boundary = true;
                }
                new_yaw = yaw_limit_min;
                yaw_direction = 1.0;
            }
            else
            {
                at_boundary = false;
            }
            current_yaw_ = new_yaw;
        }

        // 检查是否完成指定数量的扫描周期
        if (current_period >= max_scan_period_)
        {
            current_period = 0;
            scan_phase_ = 0.0;
            current_state_ = SPIN;
            RCLCPP_INFO(logger_, "SCAN -> SPIN (扫描周期结束，共 %d 个周期), 当前yaw: %.2f", max_scan_period_, current_yaw_);
        }

        if (current_yaw_ > M_PI)
            current_yaw_ -= 2 * M_PI;
        else if (current_yaw_ < -M_PI)
            current_yaw_ += 2 * M_PI;

        target_yaw_ = current_yaw_;
        target_pitch_ = current_pitch_;
    }

    void LidarscanNode::damagedScanState()
    {
        static double pitch_direction = 1.0;
        static double yaw_direction = 1.0;
        static bool at_boundary = false;

        // 与普通扫描类似，但围绕装甲板位置进行
        current_pitch_ += pitch_direction * spin_pitch_speed_ * dt_;

        if (current_pitch_ >= max_pitch_)
        {
            current_pitch_ = max_pitch_;
            pitch_direction = -1.0;
        }
        else if (current_pitch_ <= min_pitch_)
        {
            current_pitch_ = min_pitch_;
            pitch_direction = 1.0;
        }

        // 围绕被打击的装甲板位置扫描
        double yaw_limit_max = target_yaw_ + scan_theta_ / 2;
        double yaw_limit_min = target_yaw_ - scan_theta_ / 2;

        // 计算新的yaw位置，考虑方向和速度因子
        double yaw_speed_factor = current_pitch_ < 0 ? pitch_less_speed_ : pitch_more_speed_;
        double yaw_speed = spin_yaw_speed_ * dt_ * yaw_speed_factor;
        double new_yaw = current_yaw_ + yaw_direction * yaw_speed;

        // 检查是否超出范围并调整
        if (new_yaw > yaw_limit_max)
        {
            if (!at_boundary)
            {
                RCLCPP_INFO(logger_, "扫描受损装甲板 ID: %d, 到达右边界, 角度: %.2f, 进度: %d/%d",
                            current_damaged_armor_id_, target_yaw_, current_period, max_scan_period_);
                current_period++;
                at_boundary = true;
            }
            new_yaw = yaw_limit_max;
            yaw_direction = -1.0;
        }
        else if (new_yaw < yaw_limit_min)
        {
            if (!at_boundary)
            {
                RCLCPP_INFO(logger_, "扫描受损装甲板 ID: %d, 到达左边界, 角度: %.2f, 进度: %d/%d",
                            current_damaged_armor_id_, target_yaw_, current_period, max_scan_period_);
                current_period++;
                at_boundary = true;
            }
            new_yaw = yaw_limit_min;
            yaw_direction = 1.0;
        }
        else
        {
            at_boundary = false;
        }

        // 应用计算后的角度
        current_yaw_ = new_yaw;

        if (current_period >= max_scan_period_)
        {
            current_period = 0;
            scan_phase_ = 0.0;
            damaged_armor_detected_ = false;
            current_state_ = SPIN;
            RCLCPP_INFO(logger_, "DAMAGED_SCAN -> SPIN (装甲板扫描完成), 当前yaw: %.2f", current_yaw_);
        }

        // 标准化角度到[-π, π]
        if (current_yaw_ > M_PI)
            current_yaw_ -= 2 * M_PI;
        else if (current_yaw_ < -M_PI)
            current_yaw_ += 2 * M_PI;

        target_yaw_ = current_yaw_;
        target_pitch_ = current_pitch_;
    }

    LidarscanNode::~LidarscanNode() {}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lidarscan::LidarscanNode)
