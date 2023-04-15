# ikid_ws
![image](https://github.com/BronWang/ikid_ws/blob/ikid_ws_ros_control/img/team_logo.png)
Ikid Robot ros仿真环境＋机器人模型+运动功能包
# ubuntu 20.04 + ros/noetic

# 安装依赖：
- sudo apt install ros-noetic-rviz-imu-plugin
- sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers 
- sudo apt-get install ros-noetic-effort-controllers

# 更新日志：
## 2022/12/25
- 添加了启动脚本 **ikidRobot.sh**
- 添加了仿真环境机器人运动节点
- 添加了键盘控制功能包
- 添加了机器人跌倒爬起的功能
- 使用PID控制来稳定机器人主体的 **roll angle** 和 **pitch angle**

## 2023/1/6
- 添加了特殊步态调用节点
- 优化了机器人起步不稳定的问题
- 对于步态的调用，可以单独对每一个关键帧设置与下一关键帧之间的插帧数
- 添加了右脚踢球步态文件

## 2023/1/7
- 添加了机器人零点的读取功能
- 添加了特殊步态文件上传功能的客户端

## 2023/1/10
- 修正了zmp预观控制“未来值”序列数据计算不准确的问题
- 对起步稳定性进一步优化

## 2023/2/8
- 增加了动态踢球功能，动态踢球使用四段五次多项式插值
- 把正弦轨迹替换为了两段多项式插值轨迹
- 增加了机器人的全身偏转力矩的计算功能

## 2023/2/22
- 添加了Dynamixel MX-64舵机通信协议
- 添加了物理控制板消息类型与话题

## 2023/3/3
- 实现了物理控制板imu信息接收
- 通过了初步的实机测试

## 2023/3/23
- 单独添加了零点步态文件
- 摔倒后再步行，零点异常（已解决）


