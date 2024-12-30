# RoboSense_Helios_Configuration

新建工作空间，在src文件夹中：\
``git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git``

初始化并更新子模块
```
cd rslidar_sdk
git submodule init
git submodule update
```

## 修改内容
config文件夹中的config.yaml，雷达型号改为RSHELIOS，原来是RSM1
```
lidar:
  - driver:
      lidar_type: RSHELIOS             #LiDAR type - RS16, RS32, RSBP, RSHELIOS, RSHELIOS_16P, RS128, RS80, RS48, RSP128, RSP80, RSP48, 
                                   #             RSM1, RSM1_JUMBO, RSM2, RSM3, RSE1, RSMX.
```

编译、source、运行
```
cd ../..
catkin_make
source devel/setup.bash
roslaunch rslidar_sdk start.launch
```

## 部署Fastlio2
Fastlio2 github：
``git clone https://github.com/hku-mars/FAST_LIO.git``
