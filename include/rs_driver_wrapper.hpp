#ifndef RS_DRIVER_WRAPPER_HPP
#define RS_DRIVER_WRAPPER_HPP

#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/msg/point_cloud_msg.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <mutex>

using namespace robosense::lidar;

class RSDriverWrapper {
public:
    RSDriverWrapper();
    ~RSDriverWrapper();

    // 初始化驱动
    bool initDriver(int msop_port, int difop_port, LidarType lidar_type);

    // 启动驱动
    void start();

    // 停止驱动
    void stop();

    // 获取最新点云数据
    pcl::PointCloud<pcl::PointXYZI>::Ptr getPointCloud();

private:
    std::shared_ptr<LidarDriver<PointCloudT<pcl::PointXYZI>>> driver_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr latest_cloud_;
    std::mutex cloud_mutex_;
};

#endif // RS_DRIVER_WRAPPER_HPP
