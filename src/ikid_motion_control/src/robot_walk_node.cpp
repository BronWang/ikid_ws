#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "ikid_motion_control/robotModel.h"
#include "ikid_motion_control/cmd_walk.h"

extern robotLink robotModel[26];
extern double theta;
extern double sx;
extern double sy;

void doWalkMsg(const ikid_motion_control::cmd_walk::ConstPtr& walkMsg){
    ROS_INFO("0000");
    sx = walkMsg->sx;
    sy = walkMsg->sy;
	if(!(walkMsg->stop_walk)){
        bool stop_walk_flag;
        ros::param::get("stop_walk_flag",stop_walk_flag); //如果是停止开始，先执行启动步态
        if(stop_walk_flag){
            startTrajPlan();
        }
        ros::param::set("stop_walk_flag",false);
        if(walkMsg->var_theta > 10e-5){
            ros::param::set("stop_turn_flag",false);
            anglePlan(walkMsg->var_theta);
            ros::param::set("stop_turn_flag",true);
        }else if(walkMsg->var_theta < 0){
            ros::param::set("stop_turn_flag",false);
            anglePlan(walkMsg->var_theta);
            ros::param::set("stop_turn_flag",true);
        }
        if(walkMsg->walk_with_ball){
            ros::param::set("walk_with_ball",true);
        }else{
            ros::param::set("walk_with_ball",false);
        }
        

    }else{
        bool stop_walk_flag;
        ros::param::get("stop_walk_flag",stop_walk_flag); //如果已经停止，不执行操作
        if(!stop_walk_flag){
            sx = 0;
            trajPlan();
            FallUpInitPos();  // 保证停稳
            ros::param::set("stop_walk_flag",true);
        }
        
    }
}


int main(int argc, char *argv[])
{
    //执行 ros 节点初始化
    ros::init(argc,argv,"robot_walk_node");
    //创建 ros 节点句柄
    ros::NodeHandle n;

    ROS_INFO("IkID-ROBOT NB! Let's walk it!");
    robotStart(n);
    ros::Duration(3).sleep();

    ros::Subscriber sub = n.subscribe<ikid_motion_control::cmd_walk>("/cmd_walk",5,doWalkMsg);
    ros::param::set("stop_walk_flag",true);  //设置停止行走标志位于参数服务器中
    ros::param::set("stop_turn_flag",true);  //设置停止转弯标志位于参数服务器中
    ros::param::set("walk_with_ball",false);  //设置动态踢球标志位于参数服务器中

    clearImuDataTxt();
    clearZmpDataTxt();

    while (ros::ok())
    {
        bool stop_walk_flag;
        ros::param::get("stop_walk_flag",stop_walk_flag);
        judgeFall();
        if(!stop_walk_flag){
            trajPlan();
        }
        ros::spinOnce();
    }
    
    
    return 0;
}
