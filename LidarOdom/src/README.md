---
title: design
tags: liv-slam
notebook: blogs
---

# 一、运行须知
1. 需在静止状体下启动程序。

# 二、算法设计
## 1. addPointcloud() processing pipeline
1. 添加第一帧点云 -> 保留第一帧相关信息
2. 第二帧点云 -> 对第一帧点云去畸变

# 三、传感器设置
1. 经过充分的验证发现，Velodyne VLP-16 激光雷达的帧时间戳取的是最后一个点的时间，每个点的时间戳为相对最后一个点的相对时间（负数）。
