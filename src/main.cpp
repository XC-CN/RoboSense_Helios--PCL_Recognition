#include "rs_driver_wrapper.hpp"
#include "solar_panel_detector.hpp"
#include "point_cloud_visualizer.hpp"
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <csignal>
#include <atomic>
#include <iostream>

// 用于多线程的队列和同步机制
std::queue<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_queue;
std::mutex queue_mutex;
std::condition_variable queue_cv;
std::atomic<bool> processing(true);  // 控制线程退出的标志

// 捕获 SIGINT 信号（Ctrl+C）
void signalHandler(int signal) {
    if (signal == SIGINT) {
        std::cout << "SIGINT received. Stopping program..." << std::endl;
        processing = false;
        queue_cv.notify_all();  // 通知所有等待线程
    }
}

// 数据采集线程
void dataAcquisition(RSDriverWrapper& driver) {
    try {
        while (processing) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = driver.getPointCloud();
            if (cloud && !cloud->empty()) {
                {
                    std::lock_guard<std::mutex> lock(queue_mutex);
                    cloud_queue.push(cloud);  // 将点云数据推入队列
                }
                queue_cv.notify_one();  // 通知处理线程
                std::cout << "Point cloud pushed to queue. Size: " << cloud->size() << std::endl;
            } else {
                std::cerr << "Warning: Empty point cloud received!" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 模拟刷新率
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception in data acquisition thread: " << e.what() << std::endl;
    }
    std::cout << "Data acquisition thread stopped." << std::endl;
}

// 数据处理线程
void dataProcessing(SolarPanelDetector& detector, PointCloudVisualizer& visualizer) {
    try {
        while (processing) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;

            {
                // 等待数据或退出信号
                std::unique_lock<std::mutex> lock(queue_mutex);
                queue_cv.wait(lock, [] { return !cloud_queue.empty() || !processing; });

                if (!processing && cloud_queue.empty()) {
                    break;  // 如果退出标志已设置且队列为空，结束处理线程
                }

                // 从队列中取出点云数据
                cloud = cloud_queue.front();
                cloud_queue.pop();
                std::cout << "Point cloud popped from queue. Size: " << cloud->size() << std::endl;
            }

            try {
                // 检测逻辑
                pcl::PointXYZ panel_center;
                pcl::PointXYZI min_pt, max_pt;
                bool found_panel = detector.detect(cloud, panel_center, min_pt, max_pt);

                if (found_panel) {
                    std::cout << "Solar panel detected at center: ("
                              << panel_center.x << ", "
                              << panel_center.y << ", "
                              << panel_center.z << ")" << std::endl;

                    visualizer.markPanel(panel_center);
                    visualizer.drawBoundingBox(min_pt, max_pt);
                } else {
                    std::cout << "No solar panel detected." << std::endl;
                }

                // 更新点云到 Viewer，即使没有检测到太阳能板，也更新点云
                visualizer.updatePointCloud(cloud);
                visualizer.spinOnce();  // 单次刷新 Viewer
            } catch (const std::exception& e) {
                std::cerr << "Exception during data processing: " << e.what() << std::endl;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception in data processing thread: " << e.what() << std::endl;
    }
    std::cout << "Data processing thread stopped." << std::endl;
}

int main() {
    // 捕获退出信号
    std::signal(SIGINT, signalHandler);

    // 初始化 RSDriverWrapper
    RSDriverWrapper driver;
    if (!driver.initDriver(6699, 7788, LidarType::RSHELIOS)) {
        std::cerr << "Failed to initialize RSDriver!" << std::endl;
        return -1;
    }
    driver.start();
    std::cout << "RSDriver started." << std::endl;

    // 初始化 SolarPanelDetector 和 PointCloudVisualizer
    SolarPanelDetector detector(0.01);  // RANSAC 距离阈值
    PointCloudVisualizer visualizer;

    // 启动采集线程和处理线程
    std::thread acquisition_thread(dataAcquisition, std::ref(driver));
    std::thread processing_thread(dataProcessing, std::ref(detector), std::ref(visualizer));

    try {
        // 等待线程结束
        acquisition_thread.join();
        processing_thread.join();
    } catch (const std::exception& e) {
        std::cerr << "Exception in main thread: " << e.what() << std::endl;
    }

    driver.stop();
    std::cout << "Exiting program..." << std::endl;

    return 0;
}
