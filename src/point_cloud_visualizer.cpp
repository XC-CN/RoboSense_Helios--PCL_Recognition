#include "point_cloud_visualizer.hpp"


// 构造函数
PointCloudVisualizer::PointCloudVisualizer() {
    viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>("Point Cloud Viewer");
    viewer_->setBackgroundColor(0, 0, 0);
    viewer_->addCoordinateSystem(1.0);    // 添加坐标系

}

// 更新点云
void PointCloudVisualizer::updatePointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    if (!cloud || cloud->empty()) {
        std::cerr << "Input cloud is empty or invalid!" << std::endl;
        return;
    }
    
    const std::lock_guard<std::mutex> lock(mtx_);
    viewer_->removePointCloud("cloud");  // 删除上一帧点云
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_color(cloud, "intensity");

    if (!viewer_->addPointCloud<pcl::PointXYZI>(cloud, intensity_color, "cloud")) {
        std::cerr << "Failed to add point cloud to viewer!" << std::endl;
        return;
    }
    viewer_->spinOnce(100);  // 强制刷新可视化窗口
}

// 标记面板中心
void PointCloudVisualizer::markPanel(const pcl::PointXYZ& panel_center) {
    const std::lock_guard<std::mutex> lock(mtx_);
    viewer_->removeShape("solar_panel");
    viewer_->addSphere(panel_center, 0.1, 1.0, 0.0, 0.0, "solar_panel");  // 用红色球体标记
}

// 启动可视化线程
void PointCloudVisualizer::spin() {
    while (!viewer_->wasStopped()) {
        const std::lock_guard<std::mutex> lock(mtx_);
        viewer_->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// 绘制边界框
void PointCloudVisualizer::drawBoundingBox(const pcl::PointXYZI& min_pt, const pcl::PointXYZI& max_pt) {
    const std::lock_guard<std::mutex> lock(mtx_);

    // 删除之前的边界框（防止多次叠加）
    viewer_->removeShape("bounding_box");

    // 添加边界框
    viewer_->addCube(min_pt.x, max_pt.x, min_pt.y, max_pt.y, min_pt.z, max_pt.z, 1.0, 0.0, 0.0, "bounding_box");
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "bounding_box"); // 设置边界框线条宽度
}

// 单次刷新
void PointCloudVisualizer::spinOnce() {
    const std::lock_guard<std::mutex> lock(mtx_);
    viewer_->spinOnce(100);
}

