# RoboSense_Helios_Configuration

SDK下载：https://cdn.robosense.cn/20220607191752_85539.rar

## 修改内容
config文件夹中的config.yaml，雷达型号改为RSHELIOS，原来是RSM1
```
lidar:
  - driver:
      lidar_type: RSHELIOS             #LiDAR type - RS16, RS32, RSBP, RSHELIOS, RSHELIOS_16P, RS128, RS80, RS48, RSP128, RSP80, RSP48, 
                                   #             RSM1, RSM1_JUMBO, RSM2, RSM3, RSE1, RSMX.
```

## 部署Fastlio2
Fastlio2 github：
``git clone https://github.com/hku-mars/FAST_LIO.git``
