#ifndef POINT_CLOUD_VISUALIZER_HPP
#define POINT_CLOUD_VISUALIZER_HPP
#include <thread>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <mutex>

class PointCloudVisualizer {
public:
    PointCloudVisualizer();

    // 更新点云显示
    void updatePointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    // 标注检测出的太阳能板
    void markPanel(const pcl::PointXYZ& panel_center);

    // 开始可视化
    void spin();

    // 框选边界框
    void drawBoundingBox(const pcl::PointXYZI& min_pt, const pcl::PointXYZI& max_pt);

    void spinOnce();

private:
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    std::mutex mtx_;
};

#endif // POINT_CLOUD_VISUALIZER_HPP
