#include "solar_panel_detector.hpp"
#include <iostream>

SolarPanelDetector::SolarPanelDetector(float distance_threshold)
    : distance_threshold_(distance_threshold) {}

bool SolarPanelDetector::detect(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointXYZ& panel_center, pcl::PointXYZI& min_pt, pcl::PointXYZI& max_pt) {
    if (cloud->empty()) {
        std::cerr << "Input cloud is empty!" << std::endl;
        return false;
    }

    // 创建 RANSAC 平面分割对象
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distance_threshold_);

    pcl::PointCloud<pcl::PointXYZI>::Ptr remaining_cloud(new pcl::PointCloud<pcl::PointXYZI>(*cloud)); // 剩余点云
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    bool panel_found = false;

    while (!remaining_cloud->empty()) {
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

        // 拟合平面
        seg.setInputCloud(remaining_cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            std::cout << "No more planes detected." << std::endl;
            break;
        }

        // 提取当前平面的点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        extract.setInputCloud(remaining_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false); // 提取平面点云
        extract.filter(*plane_cloud);

        // 移除当前平面点云，更新剩余点云
        extract.setNegative(true);
        extract.filter(*remaining_cloud);

        // 计算边界框
        pcl::getMinMax3D(*plane_cloud, min_pt, max_pt);

        float width = max_pt.x - min_pt.x;
        float height = max_pt.y - min_pt.y;

        std::cout << "Detected plane: width = " << width << "m, height = " << height << "m" << std::endl;

        // 判断是否符合太阳能板尺寸
        if (isPanelSizeValid(min_pt, max_pt)) {
            panel_center.x = (min_pt.x + max_pt.x) / 2.0f;
            panel_center.y = (min_pt.y + max_pt.y) / 2.0f;
            panel_center.z = (min_pt.z + max_pt.z) / 2.0f;
            panel_found = true;
            std::cout << "Solar panel detected!" << std::endl;
            break; // 如果只需要第一个符合条件的平面，则可以退出循环
        }
    }

    return panel_found;
}

bool SolarPanelDetector::isPanelSizeValid(const pcl::PointXYZI& min_pt, const pcl::PointXYZI& max_pt) {
    float width = max_pt.x - min_pt.x;
    float height = max_pt.y - min_pt.y;

    // 判断是否符合太阳能板的宽度和高度
    return (width > 0.68 - 0.1 && width < 0.68 + 0.1 && height > 0.45 - 0.1 && height < 0.45 + 0.1);
}
