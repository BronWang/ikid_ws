#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <fstream>

std::fstream fout;
void doImuMsg(const sensor_msgs::Imu::ConstPtr& imuMsg_p){
	tf::Quaternion quaternion(
		imuMsg_p->orientation.x,
		imuMsg_p->orientation.y,
		imuMsg_p->orientation.z,
		imuMsg_p->orientation.w
	);
	double roll,pitch,yaw, x_accel, y_accel;
	tf::Matrix3x3(quaternion).getRPY(roll,pitch,yaw);
    roll=roll*180/M_PI;
    pitch=pitch*180/M_PI;
    yaw=yaw*180/M_PI;
    x_accel = imuMsg_p->linear_acceleration.x;
    y_accel = imuMsg_p->linear_acceleration.y;
    // ros::param::set("imu_data_roll",roll);
    // ros::param::set("imu_data_pitch",pitch);
    // ros::param::set("imu_data_yaw",yaw);

    fout.open("/home/wp/ikid_ws/imubuffer.txt", std::ios::out);
    if(fout.fail()){
        fout.open("/home/wp/ikid_ws/imubuffer.txt", std::ios::app);
        fout.close();
        fout.open("/home/wp/ikid_ws/imubuffer.txt", std::ios::out);
    }
    fout << roll << ' ' << pitch << ' ' << yaw;
    fout.close();
}


int main(int argc, char *argv[])
{
    //执行 ros 节点初始化
    ros::init(argc,argv,"pid_amend_node");
    //创建 ros 节点句柄
    ros::NodeHandle n;
    // imu测试话题数据
    ros::Subscriber suber = n.subscribe<sensor_msgs::Imu>("/imu",10,doImuMsg);
    // ros::param::set("imu_data_roll",0.0);
    // ros::param::set("imu_data_pitch",0.0);
    // ros::param::set("imu_data_yaw",0.0);
    
    ros::spin();
    return 0;
}
