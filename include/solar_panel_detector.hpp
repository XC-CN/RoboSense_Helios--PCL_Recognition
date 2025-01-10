#ifndef SOLAR_PANEL_DETECTOR_HPP
#define SOLAR_PANEL_DETECTOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

class SolarPanelDetector {
public:
    SolarPanelDetector(float distance_threshold); // 构造函数：RANSAC 距离阈值

    // 检测太阳能板，返回是否检测到
    bool detect(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointXYZ& panel_center, pcl::PointXYZI& min_pt, pcl::PointXYZI& max_pt);

private:
    float distance_threshold_; // RANSAC 的距离阈值
    bool isPanelSizeValid(const pcl::PointXYZI& min_pt, const pcl::PointXYZI& max_pt); // 判断尺寸是否符合太阳能板
};

#endif // SOLAR_PANEL_DETECTOR_HPP
