#include "rs_driver_wrapper.hpp"

RSDriverWrapper::RSDriverWrapper() : latest_cloud_(new pcl::PointCloud<pcl::PointXYZI>()) {}

RSDriverWrapper::~RSDriverWrapper() {
    stop();
}

bool RSDriverWrapper::initDriver(int msop_port, int difop_port, LidarType lidar_type) {
    RSDriverParam param;
    param.input_type = InputType::ONLINE_LIDAR;
    param.input_param.msop_port = msop_port;
    param.input_param.difop_port = difop_port;
    param.lidar_type = lidar_type;

    driver_ = std::make_shared<LidarDriver<PointCloudT<pcl::PointXYZI>>>();

    // 注册回调函数
    driver_->regPointCloudCallback(
        // 获取空闲点云的回调函数
        [this]() -> std::shared_ptr<PointCloudT<pcl::PointXYZI>> {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            return std::make_shared<PointCloudT<pcl::PointXYZI>>();
        },
        // 返回点云的回调函数
        [this](std::shared_ptr<PointCloudT<pcl::PointXYZI>> msg) {
            std::lock_guard<std::mutex> lock(cloud_mutex_);

            // 将 msg 的点云数据拷贝到 latest_cloud_
            latest_cloud_->points.assign(msg->points.begin(), msg->points.end());
            latest_cloud_->height = msg->height;
            latest_cloud_->width = msg->width;
            latest_cloud_->is_dense = msg->is_dense;
        });

    return driver_->init(param);
}

void RSDriverWrapper::start() {
    if (driver_) {
        driver_->start();
    }
}

void RSDriverWrapper::stop() {
    if (driver_) {
        driver_->stop();
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr RSDriverWrapper::getPointCloud() {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    return latest_cloud_;
}
