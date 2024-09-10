#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <math.h>
#include <Eigen/Eigen>
#include <chrono>
#include <iostream>
#include <memory>
#include <random>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class RandomMapSensing : public rclcpp::Node
{
public:
    RandomMapSensing();
    void setParameters();
    void generateWall(double minX, double maxX, double minY, double maxY, double minZ, double maxZ,
                      pcl::PointCloud<pcl::PointXYZ>& cloudMap);
    void generateCircle(double x, double y, double z, double radius, double theta,
                        pcl::PointCloud<pcl::PointXYZ>& cloudMap);
    bool generateRandomMap(pcl::PointCloud<pcl::PointXYZ>& cloudMap);

private:
    void timerCallback();
    void odometryCallback(const nav_msgs::msg::Odometry& msg);

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalMapPublisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscription_;

    sensor_msgs::msg::PointCloud2 globalMap_;

    bool hasOdom_ = false;

    std::vector<double> state_;

    int obsNum_, circleNum_;
    double sizeX_, sizeY_, sizeZ_;
    double xl_, xh_, yl_, yh_, wl_, wh_, hl_, hh_;
    double radiusL_, radiusH_, zl_, zh_, theta_;
    double limitZ_, sensingRange_, resolution_, initX_, initY_;
};