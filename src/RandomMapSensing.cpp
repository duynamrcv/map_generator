#include "map_generator/RandomMapSensing.hpp"

RandomMapSensing::RandomMapSensing() : Node("random_map_sensing")
{
    setParameters();
    globalMapPublisher_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_generator/global_cloud", 1);
    odomSubscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry", 50, std::bind(&RandomMapSensing::odometryCallback, this, _1));

    // generate point cloud data
    pcl::PointCloud<pcl::PointXYZ> cloudMap;

    generateRandomMap(cloudMap);
    pcl::toROSMsg(cloudMap, globalMap);
    globalMap.header.frame_id = "map";
    globalMap.header.stamp    = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(),
                "Converted PointCloud2 with frame_id: %s, width: %d, height: %d",
                globalMap.header.frame_id.c_str(), globalMap.width, globalMap.height);

    timer_ = this->create_wall_timer(500ms, std::bind(&RandomMapSensing::timerCallback, this));
}

void RandomMapSensing::setParameters()
{
    initX_ = this->declare_parameter("init_state_x", 0.0);
    initY_ = this->declare_parameter("init_state_y", 0.0);

    sizeX_ = this->declare_parameter("map/x_size", 10);
    sizeY_ = this->declare_parameter("map/y_size", 10);

    obsNum_     = this->declare_parameter("map/column_num", 10);
    circleNum_  = this->declare_parameter("map/cricle_num", 10);
    resolution_ = this->declare_parameter("map/resolution", 0.1);

    wl_ = this->declare_parameter("obstacle_shape/lower_rad", 0.0);
    wh_ = this->declare_parameter("obstacle_shape/upper_rad", 0.0);
    hl_ = this->declare_parameter("obstacle_shape/lower_height", 0.0);
    hh_ = this->declare_parameter("obstacle_shape/upper_height", 0.0);

    radiusL_ = this->declare_parameter("obstacle_shape/radius_l", 0.0);
    radiusH_ = this->declare_parameter("obstacle_shape/radius_h", 0.0);
    zl_      = this->declare_parameter("obstacle_shape/z_l", 2.0);
    zh_      = this->declare_parameter("obstacle_shape/z_h", 2.0);
    theta_   = this->declare_parameter("obstacle_shape/theta", 0.0);

    sensingRange_ = this->declare_parameter("sensing/radius", 10);

    xl_ = -sizeX_ / 2.0;
    xh_ = sizeX_ / 2.0;
    yl_ = -sizeY_ / 2.0;
    yh_ = sizeY_ / 2.0;
}

void RandomMapSensing::timerCallback()
{
    globalMapPublisher_->publish(globalMap);
}

void RandomMapSensing::odometryCallback(const nav_msgs::msg::Odometry& msg)
{
    if (msg.child_frame_id == "X" || msg.child_frame_id == "O") return;
    hasOdom_ = true;

    state_ = {msg.pose.pose.position.x,
              msg.pose.pose.position.y,
              msg.pose.pose.position.z,
              msg.twist.twist.linear.x,
              msg.twist.twist.linear.y,
              msg.twist.twist.linear.z,
              0.0,
              0.0,
              0.0};
}

void RandomMapSensing::generateWall(double minX, double maxX, double minY, double maxY, double minZ,
                                    double maxZ, pcl::PointCloud<pcl::PointXYZ>& cloudMap)
{
    int numX, numY, numZ;
    numX = ceil((maxX - minX) / resolution_);
    numY = ceil((maxY - minY) / resolution_);
    numZ = ceil((maxZ - minZ) / resolution_);
    for (int i = 0; i < numX; i++)
    {
        for (int j = 0; j < numY; j++)
        {
            for (int k = 0; k < numZ; k++)
            {
                pcl::PointXYZ pt;
                pt.x = minX + i * resolution_;
                pt.y = minY + j * resolution_;
                pt.z = minZ + k * resolution_;
                cloudMap.push_back(pt);
            }
        }
    }
}

void RandomMapSensing::generateCircle(double x, double y, double z, double radius, double theta,
                                      pcl::PointCloud<pcl::PointXYZ>& cloudMap)
{
    x = floor(x / resolution_) * resolution_ + resolution_ / 2.0;
    y = floor(y / resolution_) * resolution_ + resolution_ / 2.0;
    z = floor(z / resolution_) * resolution_ + resolution_ / 2.0;

    Eigen::Vector3d translate(x, y, z);
    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0, 1;

    for (double angle = 0.0; angle < 2 * M_PI; angle += resolution_ / 2)
    {
        Eigen::Vector3d cpt;
        cpt(0) = 0.0;
        cpt(1) = radius * cos(angle);
        cpt(2) = radius * sin(angle);

        cpt = rotate * cpt + translate;

        pcl::PointXYZ pt;
        pt.x = cpt(0);
        pt.y = cpt(1);
        pt.z = cpt(2);
        cloudMap.push_back(pt);
    }
}

bool RandomMapSensing::generateRandomMap(pcl::PointCloud<pcl::PointXYZ>& cloudMap)
{
    pcl::PointXYZ ptRandom;
    std::uniform_real_distribution<double> randX, randY, randW, randH;
    std::uniform_real_distribution<double> randRadius1, randRadius2, randTheta, randZ;

    std::random_device rd;
    std::default_random_engine eng(rd());

    randX = std::uniform_real_distribution<double>(xl_, xh_);
    randY = std::uniform_real_distribution<double>(yl_, yh_);
    randW = std::uniform_real_distribution<double>(wl_, wh_);
    randH = std::uniform_real_distribution<double>(hl_, hh_);

    randRadius1 = std::uniform_real_distribution<double>(radiusL_, radiusH_);
    randRadius2 = std::uniform_real_distribution<double>(radiusL_, 1.2);
    randTheta   = std::uniform_real_distribution<double>(-theta_, theta_);
    randZ       = std::uniform_real_distribution<double>(zl_, zh_);

    // generate polar obs
    for (int i = 0; i < obsNum_; i++)
    {
        double x, y, w, h;
        x = randX(eng);
        y = randY(eng);
        w = randW(eng);

        if (sqrt(pow(x - initX_, 2) + pow(y - initY_, 2)) < 2.0)
        {
            i--;
            continue;
        }

        if (sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0)
        {
            i--;
            continue;
        }

        // map to middle of the grid
        x = floor(x / resolution_) * resolution_ + resolution_ / 2.0;
        y = floor(y / resolution_) * resolution_ + resolution_ / 2.0;

        int widNum = ceil(w / resolution_);

        for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
        {
            for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
            {
                h          = randH(eng);
                int heiNum = ceil(h / resolution_);
                for (int t = 0; t < heiNum; t++)
                {
                    ptRandom.x = x + (r + 0.5) * resolution_ + 1e-2;
                    ptRandom.y = y + (s + 0.5) * resolution_ + 1e-2;
                    ptRandom.z = (t + 0.5) * resolution_ + 1e-2;
                    cloudMap.points.push_back(ptRandom);
                }
            }
        }
    }

    // generate circle obs
    for (int i = 0; i < circleNum_; ++i)
    {
        double x, y, z;
        x = randX(eng);
        y = randY(eng);
        z = randZ(eng);

        if (sqrt(pow(x - initX_, 2) + pow(y - initY_, 2)) < 2.0)
        {
            i--;
            continue;
        }

        if (sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0)
        {
            i--;
            continue;
        }

        x = floor(x / resolution_) * resolution_ + resolution_ / 2.0;
        y = floor(y / resolution_) * resolution_ + resolution_ / 2.0;
        z = floor(z / resolution_) * resolution_ + resolution_ / 2.0;

        Eigen::Vector3d translate(x, y, z);

        double theta = randTheta(eng);
        Eigen::Matrix3d rotate;
        rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0, 1;

        double radius1 = randRadius1(eng);
        double radius2 = randRadius2(eng);

        // draw a circle centered at (x,y,z)
        Eigen::Vector3d cpt;
        for (double angle = 0.0; angle < 2 * M_PI; angle += resolution_ / 2)
        {
            cpt(0) = 0.0;
            cpt(1) = radius1 * cos(angle);
            cpt(2) = radius2 * sin(angle);

            // inflate
            Eigen::Vector3d cptIf;
            for (int ifx = -0; ifx <= 0; ++ifx)
            {
                for (int ify = -0; ify <= 0; ++ify)
                {
                    for (int ifz = -0; ifz <= 0; ++ifz)
                    {
                        cptIf = cpt + Eigen::Vector3d(ifx * resolution_, ify * resolution_,
                                                      ifz * resolution_);
                        cptIf = rotate * cptIf + Eigen::Vector3d(x, y, z);

                        ptRandom.x = cptIf(0);
                        ptRandom.y = cptIf(1);
                        ptRandom.z = cptIf(2);
                        cloudMap.push_back(ptRandom);
                    }
                }
            }
        }
    }

    cloudMap.width    = cloudMap.points.size();
    cloudMap.height   = 1;
    cloudMap.is_dense = true;

    RCLCPP_INFO(this->get_logger(), "Finished generate random map with %d columns, %d circles",
                obsNum_, circleNum_);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;
    kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

    return true;
}