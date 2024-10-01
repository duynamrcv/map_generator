#pragma once

#include <Eigen/Eigen>
#include <queue>
#include <unordered_map>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "grid_map/GridMap.hpp"

#define inf            (1 << 30);
#define IN_OPEN_LIST_  'a';
#define IN_CLOSE_LIST_ 'b';
#define NOT_EXPANDED_  'c';

using namespace std::chrono_literals;
using std::placeholders::_1;

class KinoAStarNode
{
public:
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d input;
    Eigen::Vector3i index;  // index of the node in the occupancy grid, used for pruning
    double gCost;
    double fCost;
    double duration;
    KinoAStarNode* parent;
    char nodeState;

    KinoAStarNode()
    {
        gCost     = inf;
        fCost     = inf;
        parent    = NULL;
        duration  = inf;
        input     = Eigen::Vector3d::Zero();
        nodeState = NOT_EXPANDED_;
    }

    ~KinoAStarNode(){};
};
typedef KinoAStarNode* KinoAStarNodePtr;

class KinoAStarNodeComparator
{
public:
    bool operator()(KinoAStarNodePtr node1, KinoAStarNodePtr node2)
    {
        return node1->fCost > node2->fCost;
    }
};

template <typename T>
struct vector3i_hash : public std::unary_function<T, size_t>
{
    size_t operator()(const T& vector) const
    {
        size_t seed = 0;
        for (size_t i = 0; i < vector.size(); ++i)
        {
            auto elem = *(vector.data() + i);
            seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }

        return seed;
    }
};

class KinoAStarNodeHashTable
{
private:
    std::unordered_map<Eigen::Vector3i, KinoAStarNodePtr, vector3i_hash<Eigen::Vector3i>>
        dataTable_;

public:
    void insert(Eigen::Vector3i index, KinoAStarNodePtr node)
    {
        dataTable_.insert(std::make_pair(index, node));
    }

    KinoAStarNodePtr find(Eigen::Vector3i index)
    {
        auto iter = dataTable_.find(index);
        if (iter != dataTable_.end())
        {
            return iter->second;
        }
        else
        {
            return NULL;
        }
    }

    void erase(Eigen::Vector3i index) { dataTable_.erase(index); }

    void clear() { dataTable_.clear(); }

    KinoAStarNodeHashTable(){};
    ~KinoAStarNodeHashTable(){};
};

class KinoAStar : public rclcpp::Node
{
public:
    KinoAStar();
    // ~KinoAStar();
    void setParameters();

private:
    void timerCallback();
    void localCloudCallback(const sensor_msgs::msg::PointCloud2& msg);
    void odomCallback(const nav_msgs::msg::Odometry& msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped& msg);

    // bool isCollisionFree(Eigen::Vector3d pt, Eigen::Vector3d acc);
    pcl::PointCloud<pcl::PointXYZ> toPCL(const std::vector<Eigen::Vector3d>& obs);
    bool search(Eigen::Vector3d startPt, Eigen::Vector3d startVel, Eigen::Vector3d endPt,
                Eigen::Vector3d endVel, std::vector<Eigen::Vector3d>& path);
    void visPathNodes(std::vector<Eigen::Vector3d>& pathNodesList);
    void visEllipsoid(std::vector<Eigen::Vector3d>& pathNodesList,
                      std::vector<Eigen::Matrix3d>& rotList);

    // main helper functions
    Eigen::Vector3i posToIndex(Eigen::Vector3d pos);
    double getHeuristicCost(Eigen::Vector3d x1, Eigen::Vector3d v1, Eigen::Vector3d x2,
                            Eigen::Vector3d v2, double& optimalTime);
    std::vector<double> cubic(double a, double b, double c, double d);
    std::vector<double> quartic(double a, double b, double c, double d, double e);
    bool computeShotTraj(Eigen::Vector3d x1, Eigen::Vector3d v1, Eigen::Vector3d x2,
                         Eigen::Vector3d v2, double optimalTime);
    std::vector<KinoAStarNodePtr> retrievePath(KinoAStarNodePtr endNode,
                                               std::vector<Eigen::Vector3d>& pathNodesList);
    void samplePath(std::vector<KinoAStarNodePtr> pathPool, std::vector<Eigen::Vector3d>& path);
    void sampleEllipsoid(std::vector<KinoAStarNodePtr> pathPool, std::vector<Eigen::Vector3d>& path,
                         std::vector<Eigen::Matrix3d>& rotList);
    void StateTransit(Eigen::Matrix<double, 6, 1>& x0, Eigen::Matrix<double, 6, 1>& xt,
                      Eigen::Vector3d ut, double dt);
    bool isCollisionFree(Eigen::Vector3d pt, Eigen::Vector3d acc);

    // main data structure
    std::priority_queue<KinoAStarNodePtr, std::vector<KinoAStarNodePtr>, KinoAStarNodeComparator>
        openList_;
    KinoAStarNodeHashTable closeList_;
    KinoAStarNodeHashTable expandedList_;
    std::vector<KinoAStarNodePtr> pathNodePool_;
    std::vector<Eigen::Matrix3d> rotList;

    // main search parameters
    int allocatedNodeNum_;
    int useNodeNum_;
    int collisionCheckType_;
    double accRes_;
    double rou_;
    double lambdaHeu_;
    double tieBreaker_;
    double goalTolerance_;
    double stepSize_;
    double maxVel_, maxAccel_;
    double sampleTau_;

    // main robot parameters
    double rRobot_;
    double hRobot_;

    Eigen::Matrix<double, 3, 4> shotCoef_;
    Eigen::Matrix<double, 3, 4> velCoef_;
    Eigen::Matrix<double, 3, 4> accCoef_;

    // main visualization parameters
    visualization_msgs::msg::Marker pathNodeMarker_;

    /* main map parameters: grid map */
    Eigen::Vector3d origin_;
    Eigen::Vector3d mapSize_;
    double resolution_;
    std::shared_ptr<GridMap> gridMap_;

    nav_msgs::msg::Odometry odom_;

    // main map parameters: cloud map
    std::vector<Eigen::Vector3d> obs_;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr localCloudSubscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalSubscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pathNodePublisher_,
        elliposidPublisher_, pathPublisher_;
};