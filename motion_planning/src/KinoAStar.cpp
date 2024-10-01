#include "motion_planning/KinoAStar.hpp"

KinoAStar::KinoAStar() : Node("kino_astar")
{
    setParameters();
    pathNodePublisher_  = this->create_publisher<visualization_msgs::msg::Marker>("path_nodes", 1);
    elliposidPublisher_ = this->create_publisher<visualization_msgs::msg::Marker>("elliposid", 30);
    pathPublisher_      = this->create_publisher<visualization_msgs::msg::Marker>("path", 10);

    localCloudSubscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "local_cloud", 10, std::bind(&KinoAStar::localCloudCallback, this, _1));
    odomSubscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&KinoAStar::odomCallback, this, _1));
    goalSubscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal", 10, std::bind(&KinoAStar::goalCallback, this, _1));

    gridMap_ = std::make_shared<GridMap>();
    setParameters();

    timer_ = this->create_wall_timer(500ms, std::bind(&KinoAStar::timerCallback, this));
}

void KinoAStar::setParameters()
{
    allocatedNodeNum_   = this->declare_parameter("kino_astar/allocated_node_num", 1000);
    collisionCheckType_ = this->declare_parameter("kino_astar/collision_check_type", 2);
    rou_                = this->declare_parameter("kino_astar/rou_time", 50.0);
    lambdaHeu_          = this->declare_parameter("kino_astar/lambda_heu", 3.0);
    goalTolerance_      = this->declare_parameter("kino_astar/goal_tolerance", 2.0);
    stepSize_           = this->declare_parameter("kino_astar/step_size", 0.1);
    maxVel_             = this->declare_parameter("kino_astar/max_velocity", 7.0);
    maxAccel_           = this->declare_parameter("kino_astar/max_accelration", 10.0);
    accRes_             = this->declare_parameter("kino_astar/acc_resolution", 4.0);
    sampleTau_          = this->declare_parameter("kino_astar/sample_tau", 0.3);
    rRobot_             = this->declare_parameter("kino_astar/robot_r", 0.4);
    hRobot_             = this->declare_parameter("kino_astar/robot_h", 0.1);

    pathNodeMarker_.header.frame_id    = "map";
    pathNodeMarker_.type               = visualization_msgs::msg::Marker::SPHERE_LIST;
    pathNodeMarker_.action             = visualization_msgs::msg::Marker::ADD;
    pathNodeMarker_.pose.orientation.w = 1.0;
    pathNodeMarker_.scale.x            = 0.3;
    pathNodeMarker_.scale.y            = 0.3;
    pathNodeMarker_.scale.z            = 0.3;
    pathNodeMarker_.color.r            = 1.0;
    pathNodeMarker_.color.a            = 1.0;
    pathNodeMarker_.color.g            = 0.0;
    pathNodeMarker_.color.b            = 0.0;
}

void KinoAStar::timerCallback() {}

void KinoAStar::localCloudCallback(const sensor_msgs::msg::PointCloud2& msg)
{
    obs_.clear();
    pcl::PointCloud<pcl::PointXYZ> latestCloud;
    pcl::fromROSMsg(msg, latestCloud);

    for (size_t i = 0; i < latestCloud.points.size(); ++i)
    {
        Eigen::Vector3d pt;
        pt(0) = latestCloud.points[i].x;
        pt(1) = latestCloud.points[i].y;
        pt(2) = latestCloud.points[i].z;
        obs_.push_back(pt);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr =
        std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(toPCL(obs_));
    kdtree_.setInputCloud(cloudPtr);
}

void KinoAStar::odomCallback(const nav_msgs::msg::Odometry& msg)
{
    odom_ = msg;
}
void KinoAStar::goalCallback(const geometry_msgs::msg::PoseStamped& msg)
{
    Eigen::Vector3d endPt(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    Eigen::Vector3d endVel(0, 0, 0);
    Eigen::Vector3d startPt(0., 0., 0.);
    Eigen::Vector3d startVel(0., 0., 0.);

    std::vector<Eigen::Vector3d> path;
    int success = search(startPt, startVel, endPt, endVel, path);
    // Eigen::Vector3d startPt(odom_.pose.pose.position.x, odom_.pose.pose.position.y,
    //                         odom_.pose.pose.position.z);
    // Eigen::Vector3d startVel(odom_.twist.twist.linear.x, odom_.twist.twist.linear.y,
    //                          odom_.twist.twist.linear.z);

    if (success == 1)
    {
        visualization_msgs::msg::Marker pathMarker;
        pathMarker.header.frame_id = "map";
        pathMarker.header.stamp    = this->get_clock()->now();

        pathMarker.ns = "path";
        pathMarker.id = 0;

        pathMarker.type = visualization_msgs::msg::Marker::LINE_STRIP;

        pathMarker.action = visualization_msgs::msg::Marker::ADD;

        pathMarker.pose.orientation.w = 1.0;
        pathMarker.scale.x            = 0.08;
        pathMarker.scale.y            = 0.08;
        pathMarker.scale.z            = 0.08;
        pathMarker.color.a            = 1.0;
        pathMarker.color.r            = 1.0;
        pathMarker.color.g            = 0.0;
        pathMarker.color.b            = 0.0;

        for (int i = 0; i < path.size(); i++)
        {
            geometry_msgs::msg::Point pt;
            pt.x = path[i][0];
            pt.y = path[i][1];
            pt.z = path[i][2];
            pathMarker.points.push_back(pt);
        }

        pathPublisher_->publish(pathMarker);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Path not found!");
    }
    path.clear();
}

bool KinoAStar::search(Eigen::Vector3d startPt, Eigen::Vector3d startVel, Eigen::Vector3d endPt,
                       Eigen::Vector3d endVel, std::vector<Eigen::Vector3d>& path)
{
    double invAccRes           = 1.0 / accRes_;
    double optimalTime         = inf;
    KinoAStarNodePtr startNode = pathNodePool_[useNodeNum_];
    startNode->position        = startPt;
    startNode->velocity        = startVel;
    startNode->index           = posToIndex(startPt);
    startNode->gCost           = 0.0;
    startNode->fCost = lambdaHeu_ * getHeuristicCost(startPt, startVel, endPt, endVel, optimalTime);
    useNodeNum_++;

    openList_.push(startNode);
    expandedList_.insert(startNode->index, startNode);
    startNode->nodeState = IN_OPEN_LIST_;

    std::vector<Eigen::Vector3d> pathNodesList;

    while (!openList_.empty())
    {
        KinoAStarNodePtr currentNode = openList_.top();
        openList_.pop();
        closeList_.insert(currentNode->index, currentNode);
        currentNode->nodeState = IN_CLOSE_LIST_;
        currentNode->duration  = sampleTau_;

        // Check if near goal
        if ((currentNode->position - endPt).norm() < goalTolerance_)
        {
            double tmpCost     = lambdaHeu_ * (currentNode->position, currentNode->velocity, endPt,
                                           endVel, optimalTime);
            bool shotPathFound = computeShotTraj(currentNode->position, currentNode->velocity,
                                                 endPt, endVel, optimalTime);

            if (shotPathFound)
            {
                RCLCPP_INFO(this->get_logger(), "Use node number: %d", useNodeNum_);
                RCLCPP_INFO(this->get_logger(), "Total cost J: %f", currentNode->gCost + tmpCost);
                currentNode->duration                  = optimalTime;
                std::vector<KinoAStarNodePtr> pathPool = retrievePath(currentNode, pathNodesList);
                pathNodesList.push_back(endPt);
                visPathNodes(pathNodesList);
                switch (collisionCheckType_)
                {
                    case 1:
                        samplePath(pathPool, path);
                        break;
                    case 2:
                        sampleEllipsoid(pathPool, path, rotList);
                        visEllipsoid(path, rotList);
                        break;
                }
                return true;
            }
            else if (currentNode->parent != NULL)
            {
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "No shot path found. Stop");
                return false;
            }
        }

        // Expand current node
        for (double ax = -maxAccel_; ax <= maxAccel_ + 1e-3; ax += invAccRes * maxAccel_)
        {
            for (double ay = -maxAccel_; ay <= maxAccel_ + 1e-3; ay += invAccRes * maxAccel_)
            {
                for (double az = -maxAccel_; az <= maxAccel_ + 1e-3; az += invAccRes * maxAccel_)
                {
                    Eigen::Vector3d ut;
                    ut << ax, ay, az;
                    Eigen::Matrix<double, 6, 1> x0;
                    x0.head(3) = currentNode->position;
                    x0.tail(3) = currentNode->velocity;
                    Eigen::Matrix<double, 6, 1> xt;

                    int segmentNum     = std::floor(sampleTau_ / stepSize_);
                    bool flag          = false;
                    bool collisionFlag = false;
                    for (int i = 0; i <= segmentNum; i++)
                    {
                        double t = i * stepSize_;
                        StateTransit(x0, xt, ut, t);
                        Eigen::Vector3d tmpPos = xt.head(3);

                        // Check out of map
                        if (!gridMap_->isInMap(tmpPos))
                        {
                            flag = true;
                            break;
                        }

                        // Check collision
                        switch (collisionCheckType_)
                        {
                            case 1:
                                if (gridMap_->getInflateOccupancy(tmpPos) == 1)
                                {
                                    collisionFlag = true;
                                    break;
                                }
                            case 2:
                                if (!isCollisionFree(tmpPos, ut))
                                {
                                    collisionFlag = true;
                                    break;
                                }
                                break;
                        }
                        if (collisionFlag)
                        {
                            flag = true;
                            break;
                        }

                        // Check velocity limit
                        if (xt.tail(3)(0) < -maxVel_ || xt.tail(3)(0) > maxVel_ ||
                            xt.tail(3)(1) < -maxVel_ || xt.tail(3)(1) > maxVel_ ||
                            xt.tail(3)(2) < -maxVel_ || xt.tail(3)(2) > maxVel_)
                        {
                            flag = true;
                            break;
                        }
                    }
                    if (flag) continue;
                    StateTransit(x0, xt, ut, sampleTau_);

                    if (closeList_.find(posToIndex(xt.head(3))) != NULL)
                        continue;
                    else if (expandedList_.find(posToIndex(xt.head(3))) == NULL)
                    {
                        KinoAStarNodePtr proNode = pathNodePool_[useNodeNum_];
                        proNode->position        = xt.head(3);
                        proNode->velocity        = xt.tail(3);
                        proNode->index           = posToIndex(xt.head(3));
                        proNode->gCost = currentNode->gCost + (ut.dot(ut) + rou_) * sampleTau_;
                        proNode->fCost =
                            proNode->gCost + lambdaHeu_ * getHeuristicCost(proNode->position,
                                                                           proNode->velocity, endPt,
                                                                           endVel, optimalTime);
                        proNode->parent    = currentNode;
                        proNode->input     = ut;
                        proNode->duration  = sampleTau_;
                        proNode->nodeState = IN_OPEN_LIST_;
                        useNodeNum_++;
                        openList_.push(proNode);
                        expandedList_.insert(proNode->index, proNode);

                        // Check useNodeNum_ reach allocatedNodeNum_
                        if (useNodeNum_ >= allocatedNodeNum_)
                        {
                            RCLCPP_INFO(this->get_logger(), "Reach max node number. Stop");
                            return false;
                        }
                    }
                    else
                    {
                        double tmpGCost = currentNode->gCost + (ut.dot(ut) + rou_) * sampleTau_;
                        KinoAStarNodePtr oldNode = expandedList_.find(posToIndex(xt.head(3)));
                        if (tmpGCost < oldNode->gCost)
                        {
                            oldNode->position = xt.head(3);
                            oldNode->velocity = xt.tail(3);
                            oldNode->gCost    = tmpGCost;
                            oldNode->fCost =
                                oldNode->gCost +
                                lambdaHeu_ * getHeuristicCost(oldNode->position, oldNode->velocity,
                                                              endPt, endVel, optimalTime);
                            oldNode->parent = currentNode;
                            oldNode->input  = ut;
                        }
                    }
                }
            }
        }
    }
    RCLCPP_INFO(this->get_logger(), "open list is empty. No path found. Stop.");
    return false;
}

pcl::PointCloud<pcl::PointXYZ> KinoAStar::toPCL(const std::vector<Eigen::Vector3d>& obs)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width  = obs.size();
    cloud.height = 1;
    cloud.points.resize(cloud.width * cloud.height);
    for (unsigned int i = 0; i < obs.size(); i++)
    {
        cloud.points[i].x = obs[i](0);
        cloud.points[i].y = obs[i](1);
        cloud.points[i].z = obs[i](2);
    }
    return cloud;
}

Eigen::Vector3i KinoAStar::posToIndex(Eigen::Vector3d pos)
{
    Eigen::Vector3i index;
    for (int i = 0; i < 3; i++)
    {
        index(i) = std::floor((pos(i) - origin_(i)) / resolution_);
    }
    return index;
}

double KinoAStar::getHeuristicCost(Eigen::Vector3d x1, Eigen::Vector3d v1, Eigen::Vector3d x2,
                                   Eigen::Vector3d v2, double& optimalTime)
{
    Eigen::Vector3d dp = x2 - x1;
    double optimalCost = inf;

    double a = -36 * dp.dot(dp);
    double b = 24 * dp.dot(v1 + v2);
    double c = -4 * (v1.dot(v1) + v1.dot(v2) + v2.dot(v2));
    double d = 0;
    double e = rou_;

    std::vector<double> dts = quartic(e, d, c, b, a);
    double TBar             = ((x1 - x2).lpNorm<Eigen::Infinity>() / maxVel_);
    for (int i = 0; i < dts.size(); i++)
    {
        double t = dts[i];
        double tmpCost =
            a / (-3 * std::pow(t, 3)) + b / (-2 * std::pow(t, 2)) + c / (-1 * t) + e * t;
        if (tmpCost < optimalCost && t > TBar && tmpCost > 0)
        {
            optimalCost = tmpCost;
            optimalTime = t;
        }
    }
    return tieBreaker_ * optimalCost;
}

std::vector<double> KinoAStar::cubic(double a, double b, double c, double d)
{
    std::vector<double> dts;

    double a2 = b / a;
    double a1 = c / a;
    double a0 = d / a;

    double Q = (3 * a1 - a2 * a2) / 9;
    double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
    double D = Q * Q * Q + R * R;
    if (D > 0)
    {
        double S = std::cbrt(R + sqrt(D));
        double T = std::cbrt(R - sqrt(D));
        dts.push_back(-a2 / 3 + (S + T));
        return dts;
    }
    else if (D == 0)
    {
        double S = std::cbrt(R);
        dts.push_back(-a2 / 3 + S + S);
        dts.push_back(-a2 / 3 - S);
        return dts;
    }
    else
    {
        double theta = acos(R / sqrt(-Q * Q * Q));
        dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
        dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
        dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
        return dts;
    }
}

std::vector<double> KinoAStar::quartic(double a, double b, double c, double d, double e)
{
    std::vector<double> dts;

    double a3 = b / a;
    double a2 = c / a;
    double a1 = d / a;
    double a0 = e / a;

    std::vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
    double y1              = ys.front();
    double r               = a3 * a3 / 4 - a2 + y1;
    if (r < 0) return dts;

    double R = sqrt(r);
    double D, E;
    if (R != 0)
    {
        D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 +
                 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
        E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 -
                 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    }
    else
    {
        D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
        E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
    }

    if (!std::isnan(D))
    {
        dts.push_back(-a3 / 4 + R / 2 + D / 2);
        dts.push_back(-a3 / 4 + R / 2 - D / 2);
    }
    if (!std::isnan(E))
    {
        dts.push_back(-a3 / 4 - R / 2 + E / 2);
        dts.push_back(-a3 / 4 - R / 2 - E / 2);
    }

    return dts;
}

bool KinoAStar::computeShotTraj(Eigen::Vector3d x1, Eigen::Vector3d v1, Eigen::Vector3d x2,
                                Eigen::Vector3d v2, double optimalTime)
{
    double td          = optimalTime;
    Eigen::Vector3d dp = x2 - x1;
    Eigen::Vector3d dv = v2 - v1;
    shotCoef_.col(0)   = x1;
    shotCoef_.col(1)   = v1;
    shotCoef_.col(2)   = 0.5 * (6 / (td * td) * (dp - v1 * td) - 2 * dv / td);
    shotCoef_.col(3)   = 1.0 / 6.0 * (-12 / (td * td * td) * (dp - v1 * td) + 6 * dv / (td * td));

    Eigen::Matrix<double, 4, 4> TransitV;
    TransitV << 0, 0, 0, 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0;
    Eigen::Matrix<double, 4, 4> TransitA;
    TransitA << 0, 0, 0, 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0;
    velCoef_ = shotCoef_ * TransitV;
    accCoef_ = velCoef_ * TransitA;

    Eigen::Matrix<double, 4, 1> tVector;

    int segmentNum = std::floor(td / stepSize_);
    double currT   = 0.0;
    for (int j = 0; j <= segmentNum; j++)
    {
        currT = j * stepSize_;
        for (int i = 0; i < 4; i++)
        {
            tVector(i) = pow(currT, i);
        }

        Eigen::Vector3d shotPos = shotCoef_ * tVector;
        Eigen::Vector3d shotVel = velCoef_ * tVector;
        Eigen::Vector3d shotAcc = accCoef_ * tVector;

        // check collision
        if (gridMap_->getInflateOccupancy(shotPos) != 0)
        {
            return false;
        }
    }
    return true;
}

std::vector<KinoAStarNodePtr> KinoAStar::retrievePath(KinoAStarNodePtr endNode,
                                                      std::vector<Eigen::Vector3d>& pathNodesList)
{
    KinoAStarNodePtr currentNode = endNode;
    std::vector<KinoAStarNodePtr> pathNodes;

    while (currentNode->parent != NULL)
    {
        pathNodes.push_back(currentNode);
        pathNodesList.push_back(currentNode->position);
        currentNode = currentNode->parent;
    }
    pathNodes.push_back(currentNode);
    pathNodesList.push_back(currentNode->position);
    std::reverse(pathNodes.begin(), pathNodes.end());
    std::reverse(pathNodesList.begin(), pathNodesList.end());
    return pathNodes;
}

void KinoAStar::samplePath(std::vector<KinoAStarNodePtr> pathPool,
                           std::vector<Eigen::Vector3d>& path)
{
    if (pathPool.size() != 1)
    {
        for (int i = 0; i < pathPool.size() - 1; i++)
        {
            KinoAStarNodePtr currNode = pathPool[i];
            KinoAStarNodePtr nextNode = pathPool[i + 1];
            double tCurr              = 0;
            Eigen::Matrix<double, 6, 1> x0, xt;
            x0 << currNode->position, currNode->velocity;

            int segmentNum = std::floor(currNode->duration / stepSize_);
            for (int j = 0; j < segmentNum; j++)
            {
                tCurr = j * stepSize_;
                StateTransit(x0, xt, nextNode->input, tCurr);
                path.push_back(xt.head(3));
            }
            StateTransit(x0, xt, nextNode->input, currNode->duration);
            if ((xt.head(3) - nextNode->position).norm() > 1e-2)
            {
                RCLCPP_ERROR(this->get_logger(), "Error in sample");
            }
        }
        KinoAStarNodePtr lastNode = pathPool.back();
        double td                 = lastNode->duration;

        Eigen::Matrix<double, 4, 1> tVector;
        int segmentNum = std::floor(td / stepSize_);
        double tCurr   = 0;
        for (int j = 0; j < segmentNum; j++)
        {
            tCurr = j * stepSize_;
            for (int i = 0; i < 4; i++)
            {
                tVector(i) = pow(tCurr, i);
            }
            Eigen::Vector3d shotPos = shotCoef_ * tVector;
            path.push_back(shotPos);
        }
    }
    else
    {
        KinoAStarNodePtr lastNode = pathPool.back();
        double td                 = lastNode->duration;

        Eigen::Matrix<double, 4, 1> tVector;
        int segmentNum = std::floor(td / stepSize_);
        double tCurr   = 0;
        for (int j = 0; j < segmentNum; j++)
        {
            tCurr = j * stepSize_;
            for (int i = 0; i < 4; i++)
            {
                tVector(i) = pow(tCurr, i);
            }
            Eigen::Vector3d shotPos = shotCoef_ * tVector;
            path.push_back(shotPos);
        }
    }
}

void KinoAStar::sampleEllipsoid(std::vector<KinoAStarNodePtr> pathPool,
                                std::vector<Eigen::Vector3d>& path,
                                std::vector<Eigen::Matrix3d>& rotList)
{
    if (pathPool.size() != 1)
    {
        for (int i = 0; i < pathPool.size() - 1; i++)
        {
            KinoAStarNodePtr currNode = pathPool[i];
            KinoAStarNodePtr nextNode = pathPool[i + 1];
            double tCurr              = 0.0;
            Eigen::Matrix<double, 6, 1> x0, xt;
            x0 << currNode->position, currNode->velocity;

            int segmentNum = std::floor(currNode->duration / stepSize_);
            for (int j = 0; j < segmentNum; j++)
            {
                tCurr = j * stepSize_;
                StateTransit(x0, xt, nextNode->input, tCurr);
                path.push_back(xt.head(3));

                Eigen::Vector3d b3 =
                    (nextNode->input + 9.81 * Eigen::Vector3d::UnitZ()).normalized();
                Eigen::Vector3d c1(cos(0), sin(0), 0);
                Eigen::Vector3d b2 = b3.cross(c1).normalized();
                Eigen::Vector3d b1 = b2.cross(b3).normalized();

                Eigen::Matrix3d Rot;
                Rot << b1, b2, b3;
                rotList.push_back(Rot);
            }
        }

        KinoAStarNodePtr lastNode = pathPool.back();
        double td                 = lastNode->duration;

        Eigen::Matrix<double, 4, 1> tVector;

        int segmentNum = std::floor(td / stepSize_);
        double tCurr   = 0.0;
        for (int j = 0; j <= segmentNum; j++)
        {
            tCurr = j * stepSize_;
            for (int i = 0; i < 4; i++)
            {
                tVector(i) = pow(tCurr, i);
            }

            Eigen::Vector3d shotPos = shotCoef_ * tVector;
            Eigen::Vector3d shotAcc = accCoef_ * tVector;
            path.push_back(shotPos);

            Eigen::Vector3d b3 = (shotAcc + 9.81 * Eigen::Vector3d::UnitZ()).normalized();
            Eigen::Vector3d c1(cos(0), sin(0), 0);
            Eigen::Vector3d b2 = b3.cross(c1).normalized();
            Eigen::Vector3d b1 = b2.cross(b3).normalized();

            Eigen::Matrix3d Rot;
            Rot << b1, b2, b3;
            rotList.push_back(Rot);
        }
    }
    else
    {
        KinoAStarNodePtr lastNode = pathPool.back();
        double td                 = lastNode->duration;

        Eigen::Matrix<double, 4, 1> tVector;

        int segmentNum = std::floor(td / stepSize_);
        double tCurr   = 0.0;
        for (int j = 0; j <= segmentNum; j++)
        {
            tCurr = j * stepSize_;
            for (int i = 0; i < 4; i++)
            {
                tVector(i) = pow(tCurr, i);
            }

            Eigen::Vector3d shotPos = shotCoef_ * tVector;
            Eigen::Vector3d shotAcc = accCoef_ * tVector;
            path.push_back(shotPos);
            Eigen::Vector3d b3 = (shotAcc + 9.81 * Eigen::Vector3d::UnitZ()).normalized();
            Eigen::Vector3d c1(cos(0), sin(0), 0);
            Eigen::Vector3d b2 = b3.cross(c1).normalized();
            Eigen::Vector3d b1 = b2.cross(b3).normalized();

            Eigen::Matrix3d Rot;
            Rot << b1, b2, b3;
            rotList.push_back(Rot);
        }
    }
}
void KinoAStar::StateTransit(Eigen::Matrix<double, 6, 1>& x0, Eigen::Matrix<double, 6, 1>& xt,
                             Eigen::Vector3d ut, double dt)
{
    Eigen::Matrix<double, 6, 6> eAt = Eigen::Matrix<double, 6, 6>::Identity();
    for (int i = 0; i < 3; i++)
    {
        eAt(i, 3 + i) = dt;
    }
    // bug fixed: Integral should set zero
    Eigen::Matrix<double, 6, 3> Integral = Eigen::Matrix<double, 6, 3>::Zero();
    for (int i = 0; i < 6; i++)
    {
        if (i < 3)
            Integral(i, i) = 0.5 * dt * dt;
        else
            Integral(i, i - 3) = dt;
    }

    xt = eAt * x0 + Integral * ut;
}

bool KinoAStar::isCollisionFree(Eigen::Vector3d pt, Eigen::Vector3d acc)
{
    // check collision with local cloud
    Eigen::Vector3d b3 = (acc + 9.81 * Eigen::Vector3d::UnitZ()).normalized();
    Eigen::Vector3d c1(cos(0), sin(0), 0);
    Eigen::Vector3d b2 = b3.cross(c1).normalized();
    Eigen::Vector3d b1 = b2.cross(b3).normalized();

    Eigen::Matrix3d Rot;
    Rot << b1, b2, b3;

    Eigen::Matrix3d P = Eigen::Matrix3d::Zero();
    P(0, 0)           = rRobot_;
    P(1, 1)           = rRobot_;
    P(2, 2)           = hRobot_;

    Eigen::Matrix3d E = Rot * P * Rot.transpose();

    pcl::PointXYZ searchPoint;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    searchPoint.x = pt(0);
    searchPoint.y = pt(1);
    searchPoint.z = pt(2);

    float radius = rRobot_ + 1e-1;
    if (kdtree_.radiusSearch(searchPoint, radius, pointIdxRadiusSearch,
                             pointRadiusSquaredDistance) > 0)
    {
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
        {
            Eigen::Vector3d tmpPt = E.inverse() * (obs_[pointIdxRadiusSearch[i]] - pt);
            if (tmpPt.norm() <= 1.0) return false;
        }
    }
    return true;
}

void KinoAStar::visPathNodes(std::vector<Eigen::Vector3d>& pathNodesList)
{
    pathNodeMarker_.points.clear();
    for (int i = 0; i < pathNodesList.size(); i++)
    {
        // publishes the path nodes
        geometry_msgs::msg::Point pt;
        pt.x = pathNodesList[i](0);
        pt.y = pathNodesList[i](1);
        pt.z = pathNodesList[i](2);
        pathNodeMarker_.points.push_back(pt);
    }
    pathNodePublisher_->publish(pathNodeMarker_);
}

void KinoAStar::visEllipsoid(std::vector<Eigen::Vector3d>& pathNodesList,
                             std::vector<Eigen::Matrix3d>& rotList)
{
    for (int i = 0; i < pathNodesList.size(); i++)
    {
        // publishes the ellipsoid
        visualization_msgs::msg::Marker elliposidMarker_;
        elliposidMarker_.header.frame_id = "world";
        elliposidMarker_.header.stamp    = this->get_clock()->now();
        elliposidMarker_.ns              = "elliposid";
        elliposidMarker_.id              = i;
        elliposidMarker_.type            = visualization_msgs::msg::Marker::SPHERE;
        elliposidMarker_.action          = visualization_msgs::msg::Marker::ADD;
        elliposidMarker_.color.a         = 0.5;
        elliposidMarker_.color.r         = 1.0;
        elliposidMarker_.color.g         = 0.0;
        elliposidMarker_.color.b         = 1.0;
        elliposidMarker_.scale.x         = 0.4 * 2;
        elliposidMarker_.scale.y         = 0.4 * 2;
        elliposidMarker_.scale.z         = 0.1 * 2;

        elliposidMarker_.pose.position.x = pathNodesList[i](0);
        elliposidMarker_.pose.position.y = pathNodesList[i](1);
        elliposidMarker_.pose.position.z = pathNodesList[i](2);

        Eigen::Quaterniond q(rotList[i]);
        elliposidMarker_.pose.orientation.x = q.x();
        elliposidMarker_.pose.orientation.y = q.y();
        elliposidMarker_.pose.orientation.z = q.z();
        elliposidMarker_.pose.orientation.w = q.w();

        elliposidPublisher_->publish(elliposidMarker_);
    }
}
