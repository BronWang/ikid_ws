#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <std_msgs/Int16.h>
#include "ikid_motion_control/robotModel.h"
#include "ikid_motion_control/cmd_walk.h"

double step_len = 0.1;
double step_wid = 0.0528 * 2;
ros::Publisher puber_special_gait;

void doSpecialGaitMsg(const std_msgs::Int16::ConstPtr& id_msg){
    ikid_motion_control::cmd_walk walk_msg;
    bool stop_walk_flag = false;
    walk_msg.stop_walk = true;
    walk_msg.sx = step_len;
    walk_msg.sy = step_wid;
    walk_msg.var_theta = 0;
    walk_msg.walk_with_ball = false;
    puber_special_gait.publish(walk_msg);

    ros::param::get("stop_walk_flag", stop_walk_flag);
    while(!stop_walk_flag){
        ros::param::get("stop_walk_flag", stop_walk_flag);
    }
    specialGaitExec(id_msg->data);
    // walk_msg.stop_walk = false;
    // puber_special_gait.publish(walk_msg);
    // ros::param::get("stop_walk_flag", stop_walk_flag);
    // while(stop_walk_flag){
    //     ros::param::get("stop_walk_flag", stop_walk_flag);
    // }
}

int main(int argc, char *argv[])
{
    //执行 ros 节点初始化
    ros::init(argc,argv,"special_gait_node");
    //创建 ros 节点句柄
    ros::NodeHandle n;
    ros::param::get("/pid_amend/walk_length",step_len);
    ros::param::get("/pid_amend/walk_width",step_wid);
    robotStartSpecialGait(n);
    puber_special_gait = n.advertise<ikid_motion_control::cmd_walk>("/cmd_walk",5);
    // special_gait话题数据
    ros::Subscriber suber = n.subscribe<std_msgs::Int16>("/special_gait",1,doSpecialGaitMsg);
	
    ros::param::set("stop_special_gait_flag", true);
    
    ros::spin();
    return 0;
}
