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

#include "lidarscan/pc_process.hpp"
#include <cmath>

namespace lidarscan {

PointCloudProcessor::PointCloudProcessor(rclcpp::Logger logger)
: logger_(logger),
  previous_cloud_(new pcl::PointCloud<pcl::PointXYZ>())
{
}

void PointCloudProcessor::configure(double voxel_leaf_size, 
                                  double min_obstacle_dimension, 
                                  double max_obstacle_dimension,
                                  double min_obstacle_size)
{
    min_obstacle_dimension_ = min_obstacle_dimension;
    max_obstacle_dimension_ = max_obstacle_dimension;
    min_obstacle_size_ = min_obstacle_size;
    
    voxel_filter_.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::filterCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    voxel_filter_.setInputCloud(cloud);
    voxel_filter_.filter(*filtered_cloud);
    return filtered_cloud;
}

std::vector<ObstaclePoint> PointCloudProcessor::detectObstacles(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    std::vector<ObstaclePoint> detected_obstacles;
    
    // 聚类提取障碍物
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.2);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(10000);
    ec.setInputCloud(cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);
    
    for (const auto& cluster : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud, cluster.indices, *cluster_cloud);
        if (cluster_cloud->empty()) continue;
        
        // 计算质心
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cluster_cloud, centroid);
        
        // 检查高度
        bool height_ok = (centroid[2] > 0.1 && centroid[2] < 2.0);
        
        // 提取AABB特征
        pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
        feature_extractor.setInputCloud(cluster_cloud);
        feature_extractor.compute();
        pcl::PointXYZ min_point_AABB, max_point_AABB;
        feature_extractor.getAABB(min_point_AABB, max_point_AABB);
        
        // 计算尺寸
        float dim_x = max_point_AABB.x - min_point_AABB.x;
        float dim_y = max_point_AABB.y - min_point_AABB.y;
        float dim_z = max_point_AABB.z - min_point_AABB.z;
        float min_dim = std::min({dim_x, dim_y, dim_z});
        float max_dim = std::max({dim_x, dim_y, dim_z});
        
        // 检查尺寸
        bool size_ok = (min_dim >= static_cast<float>(min_obstacle_dimension_) &&
                       max_dim <= static_cast<float>(max_obstacle_dimension_));
        
        if (height_ok && size_ok) {
            pcl::PointXYZ obstacle_point;
            obstacle_point.x = centroid[0];
            obstacle_point.y = centroid[1];
            obstacle_point.z = centroid[2];
            
            if (isNewObstacle(obstacle_point)) {
                // 计算障碍物的yaw和pitch角度
                double yaw = std::atan2(obstacle_point.y, obstacle_point.x);
                double pitch = 0.0; // 通常pitch设为0，云台会在scan时进行pitch扫描
                
                // 添加到检测列表
                detected_obstacles.emplace_back(obstacle_point, yaw, pitch);
                
                RCLCPP_INFO(logger_, "Detected obstacle at (%.2f, %.2f, %.2f), yaw=%.2f",
                           obstacle_point.x, obstacle_point.y, obstacle_point.z, yaw);
            }
        } else {
            RCLCPP_DEBUG(logger_, "Rejected cluster: height_ok=%d, size_ok=%d, dims=%.2fx%.2fx%.2f",
                       height_ok, size_ok, dim_x, dim_y, dim_z);
        }
    }
    
    RCLCPP_INFO(logger_, "Detected %zu obstacles", detected_obstacles.size());
    return detected_obstacles;
}

void PointCloudProcessor::setPreviousCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::copyPointCloud(*cloud, *previous_cloud_);
}

bool PointCloudProcessor::isNewObstacle(const pcl::PointXYZ& point) const
{
    if (previous_cloud_->empty()) {
        return true;
    }
    
    for (const auto& prev_point : previous_cloud_->points) {
        double dist = std::sqrt(
            std::pow(point.x - prev_point.x, 2) +
            std::pow(point.y - prev_point.y, 2) +
            std::pow(point.z - prev_point.z, 2));
        if (dist < min_obstacle_size_) {
            return false;
        }
    }
    return true;
}

}  // namespace lidarscan
