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

#ifndef LIDARSCAN__PC_PROCESS_HPP_
#define LIDARSCAN__PC_PROCESS_HPP_

#include <memory>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/common/centroid.h"
#include "pcl/features/moment_of_inertia_estimation.h"

namespace lidarscan {

// 障碍物类
struct ObstaclePoint {
    pcl::PointXYZ point;  // 障碍物中心点坐标
    double yaw;           // 障碍物的偏航角
    double pitch;         // 障碍物的俯仰角
    
    ObstaclePoint(const pcl::PointXYZ& p, double y, double p_angle)
        : point(p), yaw(y), pitch(p_angle) {}
};

class PointCloudProcessor {
public:
    explicit PointCloudProcessor(rclcpp::Logger logger);
    
    // 配置处理器参数
    void configure(double voxel_leaf_size, 
                  double min_obstacle_dimension, 
                  double max_obstacle_dimension,
                  double min_obstacle_size);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
    // 检测和分类障碍物
    std::vector<ObstaclePoint> detectObstacles(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
    // 设置前一帧点云
    void setPreviousCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
private:
    // 判断是否为新障碍物
    bool isNewObstacle(const pcl::PointXYZ& point) const;
    
    // 日志
    rclcpp::Logger logger_;
    
    // 点云处理参数
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud_;
    double min_obstacle_dimension_{0.3};
    double max_obstacle_dimension_{0.7};
    double min_obstacle_size_{0.1};
};

}  // namespace lidarscan

#endif  // LIDARSCAN__PC_PROCESS_HPP_
