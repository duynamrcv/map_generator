#pragma once

#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <queue>
#include <random>
#include <tuple>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "grid_map/RayCaster.hpp"

#define logit(x) (log((x) / (1 - (x))))

using namespace std::chrono_literals;
using std::placeholders::_1;

struct MappingParameters
{
    Eigen::Vector3d mapOrigin_, mapSize_;
    Eigen::Vector3d mapMinBoundary_, mapMaxBoundary_;  // map range in pos
    Eigen::Vector3i mapVoxelNum_;                      // map range in index
    Eigen::Vector3d localUpdateRange_;
    double resolution_;
    double obstaclesInflation_;
    int localMapMargin_;
    double visualizationTruncateHeight_;

    /* raycasting */
    double pHit_, pMiss_, pMin_, pMax_, pOcc_;  // occupancy probability
};

struct MappingData
{
    Eigen::Vector3d odomPos_;
    Eigen::Vector3i localBoundMin_, localBoundMax_;
    std::vector<char> occupancyBuffer_;
};

class GridMap : public rclcpp::Node
{
public:
    GridMap();
    void setParameters();
    void convertToOccupancy();

    int getInflateOccupancy(Eigen::Vector3d pos);
    bool isInMap(const Eigen::Vector3d& idx);
    bool isInMap(const Eigen::Vector3i& idx);

private:
    void timerCallback();
    void cloudMapCallback(const sensor_msgs::msg::PointCloud2& msg);
    void odomCallback(const nav_msgs::msg::Odometry& msg);

    // occupancy map management
    void resetBuffer();
    void resetBuffer(Eigen::Vector3d minPos, Eigen::Vector3d maxPos);

    inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
    inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
    inline int toAddress(const Eigen::Vector3i& id);
    inline int toAddress(int& x, int& y, int& z);
    inline void boundIndex(Eigen::Vector3i& id);
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupancyMapPublisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloudMapSubscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscription_;

    MappingParameters mp_;
    MappingData md_;

    sensor_msgs::msg::PointCloud2 occupancyMap_;
};