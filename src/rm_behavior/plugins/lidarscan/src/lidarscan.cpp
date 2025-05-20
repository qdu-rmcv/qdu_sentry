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

        // 平滑处理参数
        this->declare_parameter("yaw_smooth_factor", 0.3);
        this->declare_parameter("pitch_smooth_factor", 0.3);
        this->declare_parameter("smooth_window_size", 5);
        this->declare_parameter("state_switch_threshold", 0.5);

        // 自瞄和装甲板参数
        this->declare_parameter<std::vector<int64_t>>("armor_id", std::vector<int64_t>{1, 2, 3, 4, 5, 6, 7, 8, 9});
        this->declare_parameter<double>("max_distance", 8.0);
        this->declare_parameter<double>("tracking_timeout", 1.5);
        this->declare_parameter<double>("transition_delay", 1.0);
        this->declare_parameter("enemy_distance", 3.0);

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

        // 获取自瞄和装甲板参数
        expected_armor_ids_ = this->get_parameter("armor_id").as_integer_array();
        max_distance_ = this->get_parameter("max_distance").as_double();
        tracking_timeout_ = this->get_parameter("tracking_timeout").as_double();
        scan_transition_delay_ = this->get_parameter("transition_delay").as_double();
        this->get_parameter("enemy_distance", enemy_distance_);

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

        // 初始化平滑队列
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

        RCLCPP_INFO(logger_, "订阅点云: %s, 自瞄: %s, 装甲板: %s, 底盘: %s",
                    terrain_map_topic_.c_str(), tracker_topic_.c_str(),
                    armors_topic_.c_str(), chassis_topic_.c_str());
        RCLCPP_INFO(logger_, "云台控制发布至: %s", gimbal_cmd_topic_.c_str());
    };

    // 计算装甲板对应的yaw角度
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

            if (obstacle_size >= min_bounding_box && obstacle_size <= max_bounding_box)
            {
                bool merged = false;
                for (auto &obs_yaw : new_obstacle_yaws)
                {
                    if (std::abs(obs_yaw - yaw) < state_switch_threshold_)
                    {
                        obs_yaw = (obs_yaw + yaw) / 2.0;
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

        obstacle_yaws_ = new_obstacle_yaws;

    }

    void LidarscanNode::trackerCallback(const auto_aim_interfaces::msg::Send::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_tracker_msg_ = *msg;
        last_tracker_time_ = this->now();

        RCLCPP_INFO(logger_, "自瞄 yaw: %.2f, pitch: %.2f, tracking: %d",
                     msg->yaw, msg->pitch, msg->tracking);
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
                    RCLCPP_WARN(logger_, "装甲板ID错误: %s", armor.number.c_str());
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
            RCLCPP_WARN(logger_, "异常时间间隔: %.3f秒", dt_);
        }

        std::lock_guard<std::mutex> lock(data_mutex_);

        checkStateTransition();

        updateGimbalCommand();

        gimbalsend();
    }

    void LidarscanNode::checkStateTransition()
    {

        bool is_enemy_valid = (this->now() - last_armor_time_).seconds() < tracking_timeout_;
        bool is_tracker_valid = last_tracker_msg_.has_value() &&
                                (this->now() - last_tracker_time_).seconds() < tracking_timeout_;

        if (!is_enemy_valid)
        {
            enemy_detected_ = false;
        }

        scan_state new_state = current_state_;

        if (enemy_detected_ && is_tracker_valid && last_tracker_msg_->tracking)
        {
            if (current_state_ != TRACKING)
            {
                RCLCPP_INFO(logger_, " -> TRACKING");
                mode_switch_time_ = this->now();
            }
            new_state = TRACKING;
        }

        else if (current_state_ == TRACKING)
        {
            if ((this->now() - mode_switch_time_).seconds() < scan_transition_delay_)
            {
                RCLCPP_DEBUG(logger_, "TRACKING: %.1f/%.1fs",
                             (this->now() - mode_switch_time_).seconds(), scan_transition_delay_);
            }
            else
            {
                if (damaged_armor_detected_)
                {
                    new_state = DAMAGED_SCAN;
                    RCLCPP_INFO(logger_, "TRACKING -> DAMAGED_SCAN");
                }
                else if (!obstacle_yaws_.empty() && isYawNearObstacle())
                {
                    new_state = SCAN;
                    RCLCPP_INFO(logger_, "TRACKING -> SCAN");
                }
                else
                {
                    new_state = SPIN;
                    RCLCPP_INFO(logger_, "TRACKING -> SPIN");
                }
            }
        }
        else if (damaged_armor_detected_)
        {
            if (current_state_ != DAMAGED_SCAN)
            {
                RCLCPP_INFO(logger_, " -> DAMAGED_SCAN");
            }
            new_state = DAMAGED_SCAN;
        }

        else if (!obstacle_yaws_.empty() && isYawNearObstacle())
        {
            if (current_state_ != SCAN)
            {
                RCLCPP_INFO(logger_, " -> SCAN");
            }
            new_state = SCAN;
        }
        else
        {
            if (current_state_ != SPIN)
            {
                RCLCPP_INFO(logger_, " -> SPIN");
            }
            new_state = SPIN;
        }

        current_state_ = new_state;
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

        // RCLCPP_INFO(logger_, "扫描 yaw=%.2f, pitch=%.2f",
        //             final_yaw, final_pitch);
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
        if (last_tracker_msg_.has_value())
        {
            current_yaw_ = last_tracker_msg_->yaw;
            current_pitch_ = last_tracker_msg_->pitch;
        }
    }

    void LidarscanNode::spinState()
    {
        current_yaw_ += spin_yaw_speed_ * dt_;
        if (current_yaw_ > M_PI)
        {
            current_yaw_ -= 2 * M_PI;
        }
        else if (current_yaw_ < -M_PI)
        {
            current_yaw_ += 2 * M_PI;
        }

        static double pitch_direction = 1.0;

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
        current_yaw_ += spin_yaw_speed_ * dt_ * (yaw_speed_factor - 1.0);

        target_yaw_ = current_yaw_;
        target_pitch_ = current_pitch_;
    }

    void LidarscanNode::scanState()
    {
        current_yaw_ += spin_yaw_speed_ * dt_;
        if (current_yaw_ > M_PI)
        {
            current_yaw_ -= 2 * M_PI;
        }
        else if (current_yaw_ < -M_PI)
        {
            current_yaw_ += 2 * M_PI;
        }

        static double pitch_direction = 1.0;

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

        double theta = asin(1 / enemy_distance_);
        double yaw_limit_max = closest_obstacle_yaw_ + theta / 2;
        double yaw_limit_min = closest_obstacle_yaw_ - theta / 2;



        static double yaw_direction = 1.0;
        if (current_yaw_ > yaw_limit_max)
        {
            current_yaw_ = yaw_limit_max;
            yaw_direction = -1.0;
            current_period++;
        }
        else if (current_yaw_ < yaw_limit_min)
        {
            current_yaw_ = yaw_limit_min;
            yaw_direction = 1.0;
            current_period++;
        }

        RCLCPP_INFO(logger_, "扫描障碍物: 中心角度=%.2f, 扫描范围=[%.2f, %.2f], 当前角度=%.2f, 周期=%d/%d",
                    closest_obstacle_yaw_, yaw_limit_min, yaw_limit_max, current_yaw_, current_period, max_scan_period_);

        if (current_period >= max_scan_period_)
        {
            current_period = 0;
            scan_phase_ = 0.0;
            current_state_ = SPIN;
            RCLCPP_INFO(logger_, "SCAN -> SPIN");
        }

        double yaw_speed_factor = current_pitch_ < 0 ? pitch_less_speed_ : pitch_more_speed_;
        current_yaw_ += spin_yaw_speed_ * dt_ * (yaw_speed_factor - 1.0);

        target_yaw_ = current_yaw_;
        target_pitch_ = current_pitch_;
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

    void LidarscanNode::damagedScanState()
    {

    }

    LidarscanNode::~LidarscanNode() {}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lidarscan::LidarscanNode)
