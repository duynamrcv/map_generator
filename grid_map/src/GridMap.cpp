#include "grid_map/GridMap.hpp"

GridMap::GridMap() : Node("random_map_sensing")
{
    setParameters();
    occupancyMapPublisher_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/grid_map/occupancy", 1);
    cloudMapSubscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/random_map/global_cloud", 1, std::bind(&GridMap::cloudMapCallback, this, _1));

    timer_ = this->create_wall_timer(1000ms, std::bind(&GridMap::timerCallback, this));
}

void GridMap::setParameters()
{
    double xSize = this->declare_parameter("grid_map/x_size", 40.0);
    double ySize = this->declare_parameter("grid_map/y_size", 20.0);
    double zSize = this->declare_parameter("grid_map/z_size", 5.0);

    mp_.localUpdateRange_(0) = this->declare_parameter("grid_map/local_update_range_x", 40.0);
    mp_.localUpdateRange_(1) = this->declare_parameter("grid_map/local_update_range_y", 20.0);
    mp_.localUpdateRange_(2) = this->declare_parameter("grid_map/local_update_range_z", 5.0);

    mp_.obstaclesInflation_ = this->declare_parameter("grid_map/obstacles_inflation", 0.099);
    mp_.resolution_         = this->declare_parameter("grid_map/resolution", 0.1);
    mp_.localMapMargin_     = this->declare_parameter("grid_map/local_map_margin", 10);

    mp_.visualizationTruncateHeight_ =
        this->declare_parameter("grid_map/visualization_truncate_height", 2.0);

    mp_.mapOrigin_ = Eigen::Vector3d(-xSize / 2.0, -ySize / 2.0, 0.0);
    mp_.mapSize_   = Eigen::Vector3d(xSize, ySize, zSize);

    for (int i = 0; i < 3; ++i)
        mp_.mapVoxelNum_(i) = ceil(mp_.mapSize_(i) / mp_.resolution_);

    mp_.mapMinBoundary_ = mp_.mapOrigin_;
    mp_.mapMaxBoundary_ = mp_.mapOrigin_ + mp_.mapSize_;

    int bufferSize       = mp_.mapVoxelNum_(0) * mp_.mapVoxelNum_(1) * mp_.mapVoxelNum_(2);
    md_.occupancyBuffer_ = std::vector<char>(bufferSize, 0);
}

void GridMap::resetBuffer()
{
    Eigen::Vector3d minPos = mp_.mapMinBoundary_;
    Eigen::Vector3d maxPos = mp_.mapMaxBoundary_;

    resetBuffer(minPos, maxPos);

    md_.localBoundMin_ = Eigen::Vector3i::Zero();
    md_.localBoundMax_ = mp_.mapVoxelNum_ - Eigen::Vector3i::Ones();
}

void GridMap::resetBuffer(Eigen::Vector3d minPos, Eigen::Vector3d maxPos)
{
    Eigen::Vector3i minId, maxId;
    posToIndex(minPos, minId);
    posToIndex(maxPos, maxId);

    boundIndex(minId);
    boundIndex(maxId);

    // reset occ and dist buffer
    for (int x = minId(0); x <= maxId(0); ++x)
    {
        for (int y = minId(1); y <= maxId(1); ++y)
        {
            for (int z = minId(2); z <= maxId(2); ++z)
            {
                md_.occupancyBuffer_[toAddress(x, y, z)] = 0;
            }
        }
    }
}

void GridMap::timerCallback()
{
    convertToOccupancy();
    occupancyMapPublisher_->publish(occupancyMap_);
}

void GridMap::cloudMapCallback(const sensor_msgs::msg::PointCloud2& msg)
{
    pcl::PointCloud<pcl::PointXYZ> latestCloud;
    pcl::fromROSMsg(msg, latestCloud);

    if (latestCloud.points.size() == 0) return;

    this->resetBuffer(-mp_.localUpdateRange_, mp_.localUpdateRange_);

    double minX = mp_.mapMaxBoundary_(0);
    double minY = mp_.mapMaxBoundary_(1);
    double minZ = mp_.mapMaxBoundary_(2);

    double maxX = mp_.mapMinBoundary_(0);
    double maxY = mp_.mapMinBoundary_(1);
    double maxZ = mp_.mapMinBoundary_(2);

    pcl::PointXYZ pt;
    Eigen::Vector3d p3d, p3dInf;

    int infStep  = ceil(mp_.obstaclesInflation_ / mp_.resolution_);
    int infStepZ = 1;

    for (size_t i = 0; i < latestCloud.points.size(); ++i)
    {
        pt     = latestCloud.points[i];
        p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

        // point inside update range
        Eigen::Vector3d devi = p3d;
        Eigen::Vector3i infPt;

        if (fabs(devi(0)) < mp_.localUpdateRange_(0) && fabs(devi(1)) < mp_.localUpdateRange_(1) &&
            fabs(devi(2)) < mp_.localUpdateRange_(2))
        {
            // inflate the point
            for (int x = -infStep; x <= infStep; ++x)
                for (int y = -infStep; y <= infStep; ++y)
                    for (int z = -infStepZ; z <= infStepZ; ++z)
                    {
                        p3dInf(0) = pt.x + x * mp_.resolution_;
                        p3dInf(1) = pt.y + y * mp_.resolution_;
                        p3dInf(2) = pt.z + z * mp_.resolution_;

                        maxX = std::max(maxX, p3dInf(0));
                        maxY = std::max(maxY, p3dInf(1));
                        maxZ = std::max(maxZ, p3dInf(2));

                        minX = std::min(minX, p3dInf(0));
                        minY = std::min(minY, p3dInf(1));
                        minZ = std::min(minZ, p3dInf(2));

                        posToIndex(p3dInf, infPt);

                        if (!isInMap(infPt)) continue;

                        int idxInf = toAddress(infPt);

                        md_.occupancyBuffer_[idxInf] = 1;
                    }
        }
    }

    posToIndex(Eigen::Vector3d(maxX, maxY, maxZ), md_.localBoundMax_);
    posToIndex(Eigen::Vector3d(minX, minY, minZ), md_.localBoundMin_);

    boundIndex(md_.localBoundMin_);
    boundIndex(md_.localBoundMax_);
}

void GridMap::convertToOccupancy()
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    Eigen::Vector3i minCut = md_.localBoundMin_;
    Eigen::Vector3i maxCut = md_.localBoundMax_;

    int lmm = mp_.localMapMargin_;
    minCut -= Eigen::Vector3i(lmm, lmm, lmm);
    maxCut += Eigen::Vector3i(lmm, lmm, lmm);

    boundIndex(minCut);
    boundIndex(maxCut);

    for (int x = minCut(0); x <= maxCut(0); ++x)
    {
        for (int y = minCut(1); y <= maxCut(1); ++y)
        {
            for (int z = minCut(2); z <= maxCut(2); ++z)
            {
                if (md_.occupancyBuffer_[toAddress(x, y, z)] == 0) continue;
                Eigen::Vector3d pos;
                indexToPos(Eigen::Vector3i(x, y, z), pos);
                if (pos(2) > mp_.visualizationTruncateHeight_) continue;
                pcl::PointXYZ pt;
                pt.x = pos(0);
                pt.y = pos(1);
                pt.z = pos(2);
                cloud.push_back(pt);
            }
        }
    }

    cloud.width           = cloud.points.size();
    cloud.height          = 1;
    cloud.is_dense        = true;
    cloud.header.frame_id = "map";

    pcl::toROSMsg(cloud, occupancyMap_);
    occupancyMap_.header.frame_id = "map";
    occupancyMap_.header.stamp    = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(),
                "Occupancy Map PointCloud2 with frame_id: %s, width: %d, height: %d",
                occupancyMap_.header.frame_id.c_str(), occupancyMap_.width, occupancyMap_.height);
}

inline void GridMap::boundIndex(Eigen::Vector3i& id)
{
    Eigen::Vector3i id1;
    id1(0) = std::max(std::min(id(0), mp_.mapVoxelNum_(0) - 1), 0);
    id1(1) = std::max(std::min(id(1), mp_.mapVoxelNum_(1) - 1), 0);
    id1(2) = std::max(std::min(id(2), mp_.mapVoxelNum_(2) - 1), 0);
    id     = id1;
}

inline int GridMap::toAddress(const Eigen::Vector3i& id)
{
    return id(0) * mp_.mapVoxelNum_(1) * mp_.mapVoxelNum_(2) + id(1) * mp_.mapVoxelNum_(2) + id(2);
}

inline int GridMap::toAddress(int& x, int& y, int& z)
{
    return x * mp_.mapVoxelNum_(1) * mp_.mapVoxelNum_(2) + y * mp_.mapVoxelNum_(2) + z;
}

inline bool GridMap::isInMap(const Eigen::Vector3i& idx)
{
    if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0)
    {
        return false;
    }
    if (idx(0) > mp_.mapVoxelNum_(0) - 1 || idx(1) > mp_.mapVoxelNum_(1) - 1 ||
        idx(2) > mp_.mapVoxelNum_(2) - 1)
    {
        return false;
    }
    return true;
}

inline void GridMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id)
{
    for (int i = 0; i < 3; ++i)
    {
        id(i) = floor((pos(i) - mp_.mapOrigin_(i)) / mp_.resolution_);
    }
}

inline void GridMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos)
{
    for (int i = 0; i < 3; ++i)
    {
        pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.mapOrigin_(i);
    }
}