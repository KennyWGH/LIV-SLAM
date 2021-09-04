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
