cmake_minimum_required(VERSION 3.10)

# 项目名称
project(DetectSolarPanel)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 PCL 库
find_package(PCL 1.12 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# 设置 rs_driver 的路径
set(RS_DRIVER_PATH ${CMAKE_SOURCE_DIR}/rs_driver)

# 包含 PCL 和 rs_driver 的头文件路径
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${RS_DRIVER_PATH}/src)
include_directories(${CMAKE_SOURCE_DIR}/include) # 包含新模块的头文件路径

# 如果需要链接 PCL 库
link_directories(${PCL_LIBRARY_DIRS})

# 搜集 rs_driver 的所有源文件
file(GLOB_RECURSE RS_DRIVER_SOURCES
    ${RS_DRIVER_PATH}/src/*.cpp
    ${RS_DRIVER_PATH}/src/*.c
)

# 设置主项目源文件
set(SOURCES
    src/main.cpp                   # 主程序入口
    src/rs_driver_wrapper.cpp      # RSDriver 封装实现
    src/solar_panel_detector.cpp   # 太阳能板检测实现
    src/point_cloud_visualizer.cpp # 点云可视化实现
)

# 创建可执行文件，并包含 rs_driver 的源文件
add_executable(detect_solar_panel ${SOURCES} ${RS_DRIVER_SOURCES})

# 链接 PCL、pthread 和 pcap 库
target_link_libraries(detect_solar_panel ${PCL_LIBRARIES} pthread pcap)
