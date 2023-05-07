#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <unordered_map>
#include "std_msgs/Float64.h"
#include "std_msgs/Byte.h"
#include "ros_socket/robotModel.h"
#include "ros_socket/robot_joint.h"
#include "ros_socket/robot_head_pos.h"
#include "ros_socket/packetTransformer.h"

serial::Serial sp;
std::unordered_map<uint8_t, std::pair<uint16_t,uint16_t>> joint_map;
std::unordered_map<uint8_t, uint16_t> head_joint_map;
double frame_T = 0.02;
void doControlBoardMsg(const ros_socket::robot_joint::ConstPtr& controlBoardMsg){
    // 20个关节，一个主体，两个手端，两个脚端，一个头部，共26个部件
    // id 0 主体
    // id 1 头部
    // id 2 颈前摆          初始位置2048    上2048-2844下
    // id 3 颈旋转			初始位置2048    顺1024-3096逆
    // id 4 右大臂前摆	    初始位置2048    后1024-3096前
    
    uint16_t  angle_right_arm_front_swing =2048 + (int)(-controlBoardMsg->joint[RIGHT_ARM_FRONT_SWING]*180/M_PI/0.088);
    angle_right_arm_front_swing = angle_right_arm_front_swing>4096?4096:angle_right_arm_front_swing;
    angle_right_arm_front_swing = angle_right_arm_front_swing<0?0:angle_right_arm_front_swing;
    if(!joint_map.count(RIGHT_ARM_FRONT_SWING)){
        joint_map[RIGHT_ARM_FRONT_SWING].first = angle_right_arm_front_swing;
        joint_map[RIGHT_ARM_FRONT_SWING].second = 0;
    }else{
        uint16_t temp = (uint16_t)(abs(angle_right_arm_front_swing - joint_map[RIGHT_ARM_FRONT_SWING].first)*0.088/360/frame_T*60/117.07*1023);
        temp = temp>1023?1023:temp+1;
        joint_map[RIGHT_ARM_FRONT_SWING].second = temp;
        joint_map[RIGHT_ARM_FRONT_SWING].first = angle_right_arm_front_swing;
    }
    // id 5 右大臂侧摆	    初始位置1024    左1024-3096右
    uint16_t angle_right_arm_side_swing = 1024 + (int)(-controlBoardMsg->joint[RIGHT_ARM_SIDE_SWING]*180/M_PI/0.088);
    angle_right_arm_side_swing = angle_right_arm_side_swing>3096?3096:angle_right_arm_side_swing;
    angle_right_arm_side_swing = angle_right_arm_side_swing<1024?1024:angle_right_arm_side_swing;
    if(!joint_map.count(RIGHT_ARM_SIDE_SWING)){
        joint_map[RIGHT_ARM_SIDE_SWING].first = angle_right_arm_side_swing;
        joint_map[RIGHT_ARM_SIDE_SWING].second = 0;
    }else{
        uint16_t temp =(uint16_t)(abs(angle_right_arm_side_swing - joint_map[RIGHT_ARM_SIDE_SWING].first)*0.088/360/frame_T*60/117.07*1023);
        temp = temp>1023?1023:temp+1;
        joint_map[RIGHT_ARM_SIDE_SWING].second = temp;
        joint_map[RIGHT_ARM_SIDE_SWING].first = angle_right_arm_side_swing;
    }
    // id 6 右臂肘前摆	    初始位置2048    后2048-3593前
    uint16_t angle_right_elbow_front_swing = 2048 + (int)(-controlBoardMsg->joint[RIGHT_ARM_ELBOW_FRONT_SWING]*180/M_PI/0.088);
    angle_right_elbow_front_swing = angle_right_elbow_front_swing>4000?4000:angle_right_elbow_front_swing;
    angle_right_elbow_front_swing = angle_right_elbow_front_swing<2048?2048:angle_right_elbow_front_swing;
    if(!joint_map.count(RIGHT_ARM_ELBOW_FRONT_SWING)){
        joint_map[RIGHT_ARM_ELBOW_FRONT_SWING].first = angle_right_elbow_front_swing;
        joint_map[RIGHT_ARM_ELBOW_FRONT_SWING].second = 0;
    }else{
        uint16_t temp =(uint16_t)(abs(angle_right_elbow_front_swing - joint_map[RIGHT_ARM_ELBOW_FRONT_SWING].first)*0.088/360/frame_T*60/117.07*1023);
        temp = temp>1023?1023:temp+1;
        joint_map[RIGHT_ARM_ELBOW_FRONT_SWING].second = temp;
        joint_map[RIGHT_ARM_ELBOW_FRONT_SWING].first = angle_right_elbow_front_swing;
    }
    // id 7 右手端			
    // id 8 左大臂前摆	    初始位置2048    前1024-3096后
    uint16_t angle_left_arm_front_swing = 2048 + (int)(controlBoardMsg->joint[LEFT_ARM_FRONT_SWING]*180/M_PI/0.088);
    angle_left_arm_front_swing = angle_left_arm_front_swing>4096?4096:angle_left_arm_front_swing;
    angle_left_arm_front_swing = angle_left_arm_front_swing<0?0:angle_left_arm_front_swing;
    if(!joint_map.count(LEFT_ARM_FRONT_SWING)){
        joint_map[LEFT_ARM_FRONT_SWING].first = angle_left_arm_front_swing;
        joint_map[LEFT_ARM_FRONT_SWING].second = 0;
    }else{
        uint16_t temp =(uint16_t)(abs(angle_left_arm_front_swing - joint_map[LEFT_ARM_FRONT_SWING].first)*0.088/360/frame_T*60/117.07*1023);
        temp = temp>1023?1023:temp+1;
        joint_map[LEFT_ARM_FRONT_SWING].second =temp;
        joint_map[LEFT_ARM_FRONT_SWING].first = angle_left_arm_front_swing;
    }
    // id 9 左大臂侧摆	    初始位置3096    左1024-3096右
    uint16_t angle_left_arm_side_swing = 3096 + (int)(-controlBoardMsg->joint[LEFT_ARM_SIDE_SWING]*180/M_PI/0.088);
    angle_left_arm_side_swing = angle_left_arm_side_swing>3096?3096:angle_left_arm_side_swing;
    angle_left_arm_side_swing = angle_left_arm_side_swing<1024?1024:angle_left_arm_side_swing;
    if(!joint_map.count(LEFT_ARM_SIDE_SWING)){
        joint_map[LEFT_ARM_SIDE_SWING].first = angle_left_arm_side_swing;
        joint_map[LEFT_ARM_SIDE_SWING].second = 0;
    }else{
        uint16_t temp = (uint16_t)(abs(angle_left_arm_side_swing - joint_map[LEFT_ARM_SIDE_SWING].first)*0.088/360/frame_T*60/117.07*1023);
        temp = temp>1023?1023:temp+1;
        joint_map[LEFT_ARM_SIDE_SWING].second = temp;
        joint_map[LEFT_ARM_SIDE_SWING].first = angle_left_arm_side_swing;
    }
    // id 10 左臂肘前摆	    初始位置2048    前747-2048后
    uint16_t angle_left_elbow_front_swing = 2048 + (int)(controlBoardMsg->joint[LEFT_ARM_ELBOW_FRONT_SWING]*180/M_PI/0.088);
    angle_left_elbow_front_swing = angle_left_elbow_front_swing>2048?2048:angle_left_elbow_front_swing;
    angle_left_elbow_front_swing = angle_left_elbow_front_swing<100?100:angle_left_elbow_front_swing;
    if(!joint_map.count(LEFT_ARM_ELBOW_FRONT_SWING)){
        joint_map[LEFT_ARM_ELBOW_FRONT_SWING].first = angle_left_elbow_front_swing;
        joint_map[LEFT_ARM_ELBOW_FRONT_SWING].second = 0;
    }else{
        uint16_t temp = (uint16_t)(abs(angle_left_elbow_front_swing - joint_map[LEFT_ARM_ELBOW_FRONT_SWING].first)*0.088/360/frame_T*60/117.07*1023);
        temp = temp>1023?1023:temp+1;
        joint_map[LEFT_ARM_ELBOW_FRONT_SWING].second = temp;
        joint_map[LEFT_ARM_ELBOW_FRONT_SWING].first = angle_left_elbow_front_swing;
    }
    // id 11 左手端		    
    // id 12 右髋前摆		初始位置2048    后1536-2560前
    uint16_t angle_right_hip_front_swing = 2048 + (int)(-controlBoardMsg->joint[RIGHT_HIP_FRONT_SWING]*180/M_PI/0.088);
    angle_right_hip_front_swing = angle_right_hip_front_swing>4000?4000:angle_right_hip_front_swing;
    angle_right_hip_front_swing = angle_right_hip_front_swing<1036?1036:angle_right_hip_front_swing;
    if(!joint_map.count(RIGHT_HIP_FRONT_SWING)){
        joint_map[RIGHT_HIP_FRONT_SWING].first = angle_right_hip_front_swing;
        joint_map[RIGHT_HIP_FRONT_SWING].second = 0;
    }else{
        uint16_t temp = (uint16_t)(abs(angle_right_hip_front_swing - joint_map[RIGHT_HIP_FRONT_SWING].first)*0.088/360/frame_T*60/117.07*1023);
        temp = temp>1023?1023:temp+1;
        joint_map[RIGHT_HIP_FRONT_SWING].second = temp;
        joint_map[RIGHT_HIP_FRONT_SWING].first = angle_right_hip_front_swing;
    }
    // id 13 右髋侧摆		初始位置2048    右1712-2165左
    uint16_t angle_right_hip_side_swing = 2048 + (int)(controlBoardMsg->joint[RIGHT_HIP_SIDE_SWING]*180/M_PI/0.088);
    angle_right_hip_side_swing = angle_right_hip_side_swing>2165?2165:angle_right_hip_side_swing;
    angle_right_hip_side_swing = angle_right_hip_side_swing<1200?1200:angle_right_hip_side_swing;
    if(!joint_map.count(RIGHT_HIP_SIDE_SWING)){
        joint_map[RIGHT_HIP_SIDE_SWING].first = angle_right_hip_side_swing;
        joint_map[RIGHT_HIP_SIDE_SWING].second = 0;
    }else{
        uint16_t temp = (uint16_t)(abs(angle_right_hip_side_swing - joint_map[RIGHT_HIP_SIDE_SWING].first)*0.088/360/frame_T*60/117.07*1023);
        temp = temp>1023?1023:temp+1;
        joint_map[RIGHT_HIP_SIDE_SWING].second =temp;
        joint_map[RIGHT_HIP_SIDE_SWING].first = angle_right_hip_side_swing;
    }
    // id 14 右髋旋转		初始位置2048    顺1712-2280逆
    uint16_t angle_right_hip_rotation = 2048 + (int)(controlBoardMsg->joint[RIGHT_HIP_ROTATION]*180/M_PI/0.088);
    angle_right_hip_rotation = angle_right_hip_rotation>2380?2380:angle_right_hip_rotation;
    angle_right_hip_rotation = angle_right_hip_rotation<1612?1612:angle_right_hip_rotation;
    if(!joint_map.count(RIGHT_HIP_ROTATION)){
        joint_map[RIGHT_HIP_ROTATION].first = angle_right_hip_rotation;
        joint_map[RIGHT_HIP_ROTATION].second = 0;
    }else{
        uint16_t temp = (uint16_t)(abs(angle_right_hip_rotation - joint_map[RIGHT_HIP_ROTATION].first)*0.088/360/frame_T*60/117.07*1023);
        temp = temp>1023?1023:temp+1;
        joint_map[RIGHT_HIP_ROTATION].second = temp;
        joint_map[RIGHT_HIP_ROTATION].first = angle_right_hip_rotation;
    }
    // id 15 右膝前摆		初始位置2048    后1024-3096前
    uint16_t angle_right_knee_front_swing = 2048 + (int)(-controlBoardMsg->joint[RIGHT_KNEE_FRONT_SWING]*180/M_PI/0.088);
    angle_right_knee_front_swing = angle_right_knee_front_swing>3900?3900:angle_right_knee_front_swing;
    angle_right_knee_front_swing = angle_right_knee_front_swing<200?200:angle_right_knee_front_swing;
    if(!joint_map.count(RIGHT_KNEE_FRONT_SWING)){
        joint_map[RIGHT_KNEE_FRONT_SWING].first = angle_right_knee_front_swing;
        joint_map[RIGHT_KNEE_FRONT_SWING].second = 0;
    }else{
        uint16_t temp = (uint16_t)(abs(angle_right_knee_front_swing - joint_map[RIGHT_KNEE_FRONT_SWING].first)*0.088/360/frame_T*60/117.07*1023);
        temp = temp>1023?1023:temp+1;
        joint_map[RIGHT_KNEE_FRONT_SWING].second = temp;
        joint_map[RIGHT_KNEE_FRONT_SWING].first = angle_right_knee_front_swing;
    }
    // id 16 右踝前摆		初始位置2048    上1376-2851下
    uint16_t angle_right_ankle_front_swing = 2048 + (int)(controlBoardMsg->joint[RIGHT_ANKLE_FRONT_SWING]*180/M_PI/0.088);
    angle_right_ankle_front_swing = angle_right_ankle_front_swing>3500?3500:angle_right_ankle_front_swing;
    angle_right_ankle_front_swing = angle_right_ankle_front_swing<500?500:angle_right_ankle_front_swing;
    if(!joint_map.count(RIGHT_ANKLE_FRONT_SWING)){
        joint_map[RIGHT_ANKLE_FRONT_SWING].first = angle_right_ankle_front_swing;
        joint_map[RIGHT_ANKLE_FRONT_SWING].second = 0;
    }else{
        uint16_t temp = (uint16_t)(abs(angle_right_ankle_front_swing - joint_map[RIGHT_ANKLE_FRONT_SWING].first)*0.088/360/frame_T*60/117.07*1023);
        temp = temp>1023?1023:temp+1;
        joint_map[RIGHT_ANKLE_FRONT_SWING].second = temp;
        joint_map[RIGHT_ANKLE_FRONT_SWING].first = angle_right_ankle_front_swing;
    }
    // id 17 右踝侧摆		初始位置2048    右1536-2560左
    uint16_t angle_right_ankle_side_swing = 2048 + (int)(controlBoardMsg->joint[RIGHT_ANKLE_SIDE_SWING]*180/M_PI/0.088);
    angle_right_ankle_side_swing = angle_right_ankle_side_swing>2660?2660:angle_right_ankle_side_swing;
    angle_right_ankle_side_swing = angle_right_ankle_side_swing<1436?1436:angle_right_ankle_side_swing;
    if(!joint_map.count(RIGHT_ANKLE_SIDE_SWING)){
        joint_map[RIGHT_ANKLE_SIDE_SWING].first = angle_right_ankle_side_swing;
        joint_map[RIGHT_ANKLE_SIDE_SWING].second = 0;
    }else{
        uint16_t temp = (uint16_t)(abs(angle_right_ankle_side_swing - joint_map[RIGHT_ANKLE_SIDE_SWING].first)*0.088/360/frame_T*60/117.07*1023);
        temp = temp>1023?1023:temp+1;
        joint_map[RIGHT_ANKLE_SIDE_SWING].second = temp;
        joint_map[RIGHT_ANKLE_SIDE_SWING].first = angle_right_ankle_side_swing;
    }
    // id 18 右脚端		    
    // id 19 左髋前摆		初始位置2048    前1536-2560后
    uint16_t angle_left_hip_front_swing = 2048 + (int)(controlBoardMsg->joint[LEFT_HIP_FRONT_SWING]*180/M_PI/0.088);
    angle_left_hip_front_swing = angle_left_hip_front_swing>2960?2960:angle_left_hip_front_swing;
    angle_left_hip_front_swing = angle_left_hip_front_swing<100?100:angle_left_hip_front_swing;
    if(!joint_map.count(LEFT_HIP_FRONT_SWING)){
        joint_map[LEFT_HIP_FRONT_SWING].first = angle_left_hip_front_swing;
        joint_map[LEFT_HIP_FRONT_SWING].second = 0;
    }else{
        uint16_t temp = (uint16_t)(abs(angle_left_hip_front_swing - joint_map[LEFT_HIP_FRONT_SWING].first)*0.088/360/frame_T*60/117.07*1023);
        temp = temp>1023?1023:temp+1;
        joint_map[LEFT_HIP_FRONT_SWING].second = temp;
        joint_map[LEFT_HIP_FRONT_SWING].first = angle_left_hip_front_swing;
    }
    // id 20 左髋侧摆		初始位置2048    右1712-2165左有问题 右1848-2374左
    uint16_t angle_left_hip_side_swing = 2048 + (int)(controlBoardMsg->joint[LEFT_HIP_SIDE_SWING]*180/M_PI/0.088);
    angle_left_hip_side_swing = angle_left_hip_side_swing>3000?3000:angle_left_hip_side_swing;
    angle_left_hip_side_swing = angle_left_hip_side_swing<1848?1848:angle_left_hip_side_swing;
    if(!joint_map.count(LEFT_HIP_SIDE_SWING)){
        joint_map[LEFT_HIP_SIDE_SWING].first = angle_left_hip_side_swing;
        joint_map[LEFT_HIP_SIDE_SWING].second = 0;
    }else{
        uint16_t temp = (uint16_t)(abs(angle_left_hip_side_swing - joint_map[LEFT_HIP_SIDE_SWING].first)*0.088/360/frame_T*60/117.07*1023);
        temp = temp>1023?1023:temp+1;
        joint_map[LEFT_HIP_SIDE_SWING].second = temp;
        joint_map[LEFT_HIP_SIDE_SWING].first = angle_left_hip_side_swing;
    }
    // id 21 左髋旋转		初始位置2048    顺1712-2280逆
    uint16_t angle_left_hip_rotation = 2048 + (int)(controlBoardMsg->joint[LEFT_HIP_ROTATION]*180/M_PI/0.088);
    angle_left_hip_rotation = angle_left_hip_rotation>2380?2380:angle_left_hip_rotation;
    angle_left_hip_rotation = angle_left_hip_rotation<1512?1512:angle_left_hip_rotation;
    if(!joint_map.count(LEFT_HIP_ROTATION)){
        joint_map[LEFT_HIP_ROTATION].first = angle_left_hip_rotation;
        joint_map[LEFT_HIP_ROTATION].second = 0;
    }else{
        uint16_t temp = (uint16_t)(abs(angle_left_hip_rotation - joint_map[LEFT_HIP_ROTATION].first)*0.088/360/frame_T*60/117.07*1023);
        temp = temp>1023?1023:temp+1;
        joint_map[LEFT_HIP_ROTATION].second = temp;
        joint_map[LEFT_HIP_ROTATION].first = angle_left_hip_rotation;
    }
    // id 22 左膝前摆		初始位置2048    前1024-3096后
    uint16_t angle_left_knee_front_swing = 2048 + (int)(controlBoardMsg->joint[LEFT_KNEE_FRONT_SWING]*180/M_PI/0.088);
    angle_left_knee_front_swing = angle_left_knee_front_swing>3900?3900:angle_left_knee_front_swing;
    angle_left_knee_front_swing = angle_left_knee_front_swing<200?200:angle_left_knee_front_swing;
    if(!joint_map.count(LEFT_KNEE_FRONT_SWING)){
        joint_map[LEFT_KNEE_FRONT_SWING].first = angle_left_knee_front_swing;
        joint_map[LEFT_KNEE_FRONT_SWING].second = 0;
    }else{
        uint16_t temp = (uint16_t)(abs(angle_left_knee_front_swing - joint_map[LEFT_KNEE_FRONT_SWING].first)*0.088/360/frame_T*60/117.07*1023);
        temp = temp>1023?1023:temp+1;
        joint_map[LEFT_KNEE_FRONT_SWING].second =temp;
        joint_map[LEFT_KNEE_FRONT_SWING].first = angle_left_knee_front_swing;
    }
    // id 23 左踝前摆		初始位置2048    下1376-2851上
    uint16_t angle_left_ankle_front_swing = 2048 + (int)(-controlBoardMsg->joint[LEFT_ANKLE_FRONT_SWING]*180/M_PI/0.088);
    angle_left_ankle_front_swing = angle_left_ankle_front_swing>3500?3500:angle_left_ankle_front_swing;
    angle_left_ankle_front_swing = angle_left_ankle_front_swing<500?500:angle_left_ankle_front_swing;
    if(!joint_map.count(LEFT_ANKLE_FRONT_SWING)){
        joint_map[LEFT_ANKLE_FRONT_SWING].first = angle_left_ankle_front_swing;
        joint_map[LEFT_ANKLE_FRONT_SWING].second = 0;
    }else{
        uint16_t temp = (uint16_t)(abs(angle_left_ankle_front_swing - joint_map[LEFT_ANKLE_FRONT_SWING].first)*0.088/360/frame_T*60/117.07*1023);
        temp = temp>1023?1023:temp+1;
        joint_map[LEFT_ANKLE_FRONT_SWING].second = temp;
        joint_map[LEFT_ANKLE_FRONT_SWING].first = angle_left_ankle_front_swing;
    }
    // id 24 左踝侧摆		初始位置2048    右1536-2560左
    uint16_t angle_left_ankle_side_swing = 2048 + (int)(controlBoardMsg->joint[LEFT_ANKLE_SIDE_SWING]*180/M_PI/0.088);
    angle_left_ankle_side_swing = angle_left_ankle_side_swing>2660?2660:angle_left_ankle_side_swing;
    angle_left_ankle_side_swing = angle_left_ankle_side_swing<1436?1436:angle_left_ankle_side_swing;
    if(!joint_map.count(LEFT_ANKLE_SIDE_SWING)){
        joint_map[LEFT_ANKLE_SIDE_SWING].first = angle_left_ankle_side_swing;
        joint_map[LEFT_ANKLE_SIDE_SWING].second = 0;
    }else{
        uint16_t temp = (uint16_t)(abs(angle_left_ankle_side_swing - joint_map[LEFT_ANKLE_SIDE_SWING].first)*0.088/360/frame_T*60/117.07*1023);
        temp = temp>1023?1023:temp+1;
        joint_map[LEFT_ANKLE_SIDE_SWING].second = temp;
        joint_map[LEFT_ANKLE_SIDE_SWING].first = angle_left_ankle_side_swing;
    }
    // id 25 左脚端	
    // The range of the value is 0~4095 (0xFFF), and the unit is 0.088 degree.

    // 打包数据 使用SYNC WRITE方式
    int N_dynamxiel = joint_map.size();
    int L_data_dyna = 4;
    PacketTransformer packet;
    packet.dspInst->id = ID_DSP;
    packet.dspInst->length = (L_data_dyna+1)*N_dynamxiel+4;
    packet.dspInst->instruction = INST_SET_MULTIPLE_DXL_ANGLE_SPEED;
    packet.dspInst->parameter[0] = 0x1e;
    packet.dspInst->parameter[1] = L_data_dyna;
    int i = 2;
    for(auto it:joint_map){
        packet.dspInst->parameter[i++] = it.first;
        packet.dspInst->parameter[i++] = it.second.first & 0xff;
        packet.dspInst->parameter[i++] = (it.second.first>>8) & 0xff;
        packet.dspInst->parameter[i++] = it.second.second & 0xff;
        packet.dspInst->parameter[i++] = (it.second.second>>8) & 0xff;
        // printf("%d, %d ", it.second & 0xff,(it.second>>8) & 0xff);
    }
    // printf("\n");
    packet.ConstructPacket();
    ByteArray byteArray;
    byteArray = packet.GetByteArray();
    size_t ret=1;
    // 发送位置数据
    try{
        ret = sp.write(byteArray.getBuffer(),byteArray.getLength());
    }catch(const serial::IOException& e)
    {
        ROS_ERROR_STREAM("unable to send joint msg");
        //return -1;
    }
    ros::Duration(0.015).sleep();
    // uint8_t buffer[100];
    // size_t n = sp.available();
    // if(n != 0 ){
    //     n = sp.read(buffer,n);
    //     std::cout<<"接收到的pos数据:" << n ;
    //     // for(int i = 0; i<n; i++){
    //     //     std::cout<< std::hex<< (buffer[i]&0xff)<<" ";
    //     // }
    //     //std::cout<<"接收到的数据：";
    //     std::cout<<std::endl;
    // }

    // 获取IMU数据
    // packet.Reset();
    // packet.dspInst->id = ID_DSP;
    // packet.dspInst->length = 2;
    // packet.dspInst->instruction = INFO_ICM20948_FEEDBACK;
    // packet.ConstructPacket();
    // byteArray = packet.GetByteArray();
    // ret=1;
    // // 发送imu数据
    // try{
    //     ret = sp.write(byteArray.getBuffer(),byteArray.getLength());
    // }catch(const serial::IOException& e)
    // {
    //     ROS_ERROR_STREAM("unable to send INFO_ICM20948_FEEDBACK msg");
    //     //return -1;
    // }
    // // 延时5ms等待下位机的返回数据发送完毕​
    // ros::Duration(0.02).sleep();
    // size_t n = sp.available();
    // if(n > 58) n = 58;
    // printf("%ld \n", n);
    // if(n != 0 ){
    //     n = sp.read(buffer,n);
    //     std::cout<<"接收到的imu数据";
    //     // for(int i = 0; i<n; i++){
    //     //     std::cout<< std::hex<< (buffer[i]&0xff)<<" ";
    //     // }
    //     std::cout<<std::endl;
    // }
    // icm20948_data_t sensors_data;
    // packet.Reset();
    // packet.SetRawPacket(buffer, n);
    // bool tempb = packet.TryDestructOnePacket();
    // //printf("%ld  %d\n", n,tempb);
    // if(packet.dspInst->instruction == INFO_ICM20948_FEEDBACK && n == 58 && tempb){
    //     sensors_data = *(icm20948_data_t *)packet.dspInst->parameter;
    //     printf("accel: %f, %f, %f\ngyro: %f, %f, %f\ncompass: %f, %f, %f\nori: %f, %f, %f\ntemperature: %f\n",
    //     sensors_data.accel_float[0],sensors_data.accel_float[1],sensors_data.accel_float[2],
    //     sensors_data.gyro_float[0],sensors_data.gyro_float[1],sensors_data.gyro_float[2],
    //     sensors_data.compass_float[0],sensors_data.compass_float[1],sensors_data.compass_float[2],
    //     sensors_data.orientation[0],sensors_data.orientation[1],sensors_data.orientation[2],
    //     sensors_data.temperature);
    // }
    
    
    sp.flushInput();
    sp.flushOutput();

}


void doHeadPosMsg(const ros_socket::robot_head_pos::ConstPtr& headPosMsg){
    // 20个关节，一个主体，两个手端，两个脚端，一个头部，共26个部件
    // id 0 主体
    // id 1 头部
    // id 2 颈前摆          初始位置2048    上2048-2844下
    // id 3 颈旋转			初始位置2048    顺1024-3096逆
    
    uint16_t  angle_neck_rotation =2048 + (int)(headPosMsg->neck_rotation_theta*180/M_PI/0.088);
    angle_neck_rotation = angle_neck_rotation>3072?3072:angle_neck_rotation;
    angle_neck_rotation = angle_neck_rotation<1024?1024:angle_neck_rotation;
    head_joint_map[NECK_ROTATION] = angle_neck_rotation;

    uint16_t angle_neck_front_swing = 2048 + (int)(headPosMsg->neck_front_swing_theta*180/M_PI/0.088);
    angle_neck_front_swing = angle_neck_front_swing>3072?3072:angle_neck_front_swing;
    angle_neck_front_swing = angle_neck_front_swing<2048?2048:angle_neck_front_swing;
    head_joint_map[FRONT_NECK_SWING] = angle_neck_front_swing;
    

    // 打包数据 使用SYNC WRITE方式
    int N_dynamxiel = head_joint_map.size();
    int L_data_dyna = 2;
    PacketTransformer packet;
    packet.dspInst->id = ID_DSP;
    packet.dspInst->length = (L_data_dyna+1)*N_dynamxiel+4;
    packet.dspInst->instruction = INST_SET_MULTIPLE_DXL_ANGLE_SPEED;
    packet.dspInst->parameter[0] = 0x1e;
    packet.dspInst->parameter[1] = L_data_dyna;
    int i = 2;
    for(auto it:head_joint_map){
        packet.dspInst->parameter[i++] = it.first;
        packet.dspInst->parameter[i++] = it.second & 0xff;
        packet.dspInst->parameter[i++] = (it.second>>8) & 0xff;
        // printf("%d, %d ", it.second & 0xff,(it.second>>8) & 0xff);
    }
    // printf("\n");
    packet.ConstructPacket();
    ByteArray byteArray;
    byteArray = packet.GetByteArray();
    size_t ret=1;
    // 发送位置数据
    try{
        ret = sp.write(byteArray.getBuffer(),byteArray.getLength());
    }catch(const serial::IOException& e)
    {
        ROS_ERROR_STREAM("unable to send joint msg");
        //return -1;
    }
    
    
    
    sp.flushInput();
    sp.flushOutput();
}

void doTorqueOnMsg(const std_msgs::Byte::ConstPtr& torqueOnMsg){
    // 打包数据
    PacketTransformer packet;
    packet.dspInst->id = ID_DSP;
    packet.dspInst->length = 2;
    //if(torqueOnMsg->data == 1)
        packet.dspInst->instruction = INST_TORQUE_ON;
    //else
        //packet.dspInst->instruction = INST_TORQUE_OFF;
    packet.ConstructPacket();
    ByteArray byteArray;
    byteArray = packet.GetByteArray();
    size_t ret=1;
    // 发送位置数据
    // uint8_t* temp = byteArray.getBuffer();
    // for(int i = 0; i < byteArray.getLength(); i++){
    //     printf("%x ",temp[i]);
    // }
    // printf("\n");
    try{
        ret = sp.write(byteArray.getBuffer(),byteArray.getLength());
    }catch(const serial::IOException& e)
    {
        ROS_ERROR_STREAM("unable to send torqueOnMsg");
        //return -1;
    }
    ros::Duration(0.01).sleep();
    uint8_t buffer[100];
    size_t n = sp.available();
    if(n > 0)
        std::cout<<"n:" << n << "舵机扭力已开启"<<std::endl;
    else
        std::cout<<"n:" << n << "舵机扭力未开启"<<std::endl;
    if(n != 0 ){
        n = sp.read(buffer,n);
        std::cout<<"接收到的数据:";
        for(int i = 0; i<n; i++){
            std::cout<< std::hex<< (buffer[i]&0xff)<<" ";
        }
        //std::cout<<"接收到的数据：";
        std::cout<<std::endl;
    }
    sp.flush();
}

void initHeadPos(){
    // 20个关节，一个主体，两个手端，两个脚端，一个头部，共26个部件
    // id 0 主体
    // id 1 头部
    // id 2 颈前摆          初始位置2048    上2048-2844下
    // id 3 颈旋转			初始位置2048    顺1024-3096逆
    uint16_t  angle_neck_rotation =2048 + (int)(0*180/M_PI/0.088);
    angle_neck_rotation = angle_neck_rotation>3072?3072:angle_neck_rotation;
    angle_neck_rotation = angle_neck_rotation<1024?1024:angle_neck_rotation;
    head_joint_map[NECK_ROTATION] = angle_neck_rotation;

    uint16_t angle_neck_front_swing = 2048 + (int)(0*180/M_PI/0.088);
    angle_neck_front_swing = angle_neck_front_swing>3072?3072:angle_neck_front_swing;
    angle_neck_front_swing = angle_neck_front_swing<2048?2048:angle_neck_front_swing;
    head_joint_map[FRONT_NECK_SWING] = angle_neck_front_swing;
    

    // 打包数据 使用SYNC WRITE方式
    int N_dynamxiel = head_joint_map.size();
    int L_data_dyna = 2;
    PacketTransformer packet;
    packet.dspInst->id = ID_DSP;
    packet.dspInst->length = (L_data_dyna+1)*N_dynamxiel+4;
    packet.dspInst->instruction = INST_SET_MULTIPLE_DXL_ANGLE_SPEED;
    packet.dspInst->parameter[0] = 0x1e;
    packet.dspInst->parameter[1] = L_data_dyna;
    int i = 2;
    for(auto it:head_joint_map){
        packet.dspInst->parameter[i++] = it.first;
        packet.dspInst->parameter[i++] = it.second & 0xff;
        packet.dspInst->parameter[i++] = (it.second>>8) & 0xff;
        // printf("%d, %d ", it.second & 0xff,(it.second>>8) & 0xff);
    }
    // printf("\n");
    packet.ConstructPacket();
    ByteArray byteArray;
    byteArray = packet.GetByteArray();
    size_t ret=1;
    // 发送位置数据
    try{
        ret = sp.write(byteArray.getBuffer(),byteArray.getLength());
    }catch(const serial::IOException& e)
    {
        ROS_ERROR_STREAM("unable to send joint msg");
        //return -1;
    }
    
    
    
    sp.flushInput();
    sp.flushOutput();
}

int main(int argc, char** argv){
    ros::init(argc,argv, "serial_port");
    ros::NodeHandle n;
    ros::param::get("/pid_amend/walk_frame_T",frame_T);
    serial::Timeout to = serial::Timeout::simpleTimeout(10);

    // 设置串口名称
    sp.setPort("/dev/ttyUSB0");
    sp.setBaudrate(115200);
    sp.setTimeout(to);
    try
    {
        sp.open();
    }
    catch(const serial::IOException& e)
    {
        ROS_ERROR_STREAM("unable to connect ttyUSB0");
        return -1;
    }
    if(sp.isOpen()){
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened");
    }else{
        return -1;
    }

    ros::Subscriber sub = n.subscribe<ros_socket::robot_joint>("/ikid_robot/control_board_joint_msg",5,doControlBoardMsg);
    ros::Subscriber subHead = n.subscribe<ros_socket::robot_head_pos>("/ikid_robot/robot_head_pos_msg",1,doHeadPosMsg);
    ros::Subscriber sub2 = n.subscribe<std_msgs::Byte>("/torque_on",10,doTorqueOnMsg);
    doTorqueOnMsg(nullptr); //保证扭力开启
    initHeadPos(); //保证机器人头部有力
    // ros::Rate loop_rate(1);
    // uint8_t buffer[1024];
    // uint8_t send_data[7] = {0xDD, 1,0xA5,0x03,0x00,0xFF,0xFD};
    // while (ros::ok())
    // {
    //     size_t n = sp.available();
    //     size_t ret=1;
    //     try{
    //         ret = sp.write(send_data,sizeof(send_data));
    //     }catch(const serial::IOException& e)
    //     {
    //         ROS_ERROR_STREAM("unable to send");
    //         //return -1;
    //     }
        
    //     ROS_INFO_STREAM(ret);
    //     if(n != 0 ){
    //         n = sp.read(buffer,n);
    //         for(int i = 0; i<n; i++){
    //             std::cout<<"接收到的数据：";
    //             std::cout<< std::hex<< (buffer[i]&0xff)<<" ";
    //         }
    //         std::cout<<std::endl;
    //     }
    //     loop_rate.sleep();
    // }
    ros::spin();
    sp.close();
    return 0;
    
    
}
