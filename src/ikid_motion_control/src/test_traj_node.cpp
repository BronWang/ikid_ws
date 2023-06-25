#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "ikid_motion_control/robotModel.h"


extern robotLink robotModel[26];
extern double theta;
int main(int argc, char *argv[])
{
    
  //执行 ros 节点初始化
  ros::init(argc,argv,"test_traj_node");
  //创建 ros 节点句柄
  ros::NodeHandle n;
	
	ROS_INFO("hello world!");
	robotStart(n);
	ros::Duration(3).sleep();
	double com[3];
	Calc_com(com);
	clearImuDataTxt();
    clearZmpDataTxt();
	printf("重心 :%f %f %f", com[0], com[1], com[2]);
	startTrajPlan();
	for(int i = 0; i < 5; i++){
		trajPlan();
	}
	anglePlan(-10/180.0*M_PI);
	for(int i = 0; i < 2; i++){
		trajPlan();
	}
	anglePlan(10/180.0*M_PI);
	for(int i = 0; i < 5; i++){
		trajPlan();
	}
  return 0;
}
