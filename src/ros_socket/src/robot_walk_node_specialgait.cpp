#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include "ros_socket/robotModel.h"
#include "ros_socket/cmd_walk.h"
#include <std_msgs/Int16.h>

extern robotLink robotModel[26];
extern double theta;
extern double sx;
extern double sy;
extern double init_imu_roll;
extern double init_imu_pitch;
extern double imu_data_roll;
extern double imu_data_yaw;
extern double imu_data_pitch;
extern bool isLeft;

void doWalkMsg(const ros_socket::cmd_walk::ConstPtr& walkMsg){
    ROS_INFO("0000");
    readIkidRobotZeroPoint(0);
    bool stop_special_gait_flag = true;
    ros::param::get("stop_special_gait_flag",stop_special_gait_flag);
    if(!stop_special_gait_flag){
        return;
    }
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
            if(isLeft){
                trajPlan();
            }
            FallUpInitPos();  // 保证停稳
            ros::param::set("stop_walk_flag",true);
        }
        
    }
}

void doSpecialGaitMsg(const std_msgs::Int16::ConstPtr& id_msg){

    bool stop_walk_flag;
    ros::param::get("stop_walk_flag",stop_walk_flag); //如果已经停止，不执行操作
    if(!stop_walk_flag){
        sx = 0;
        trajPlan();
        FallUpInitPos();  // 保证停稳
        ros::param::set("stop_walk_flag",true);
    }
    specialGaitExec(id_msg->data);
    // trajPlan();
    // trajPlan();
    ros::Duration(1).sleep();
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
    ros::init(argc,argv,"robot_walk_node_specialgait");
    //创建 ros 节点句柄
    ros::NodeHandle n;

    ROS_INFO("IkID-ROBOT NB! Let's walk it!");
    robotStart(n);
    ros::Duration(5).sleep();
    // ros::param::get("imu_data_roll",init_imu_roll);
    // ros::param::get("imu_data_pitch",init_imu_pitch);
    std::fstream fin;
	fin.open("/home/wp/ikid_ws/imubuffer.txt", std::ios::in);
	fin >> init_imu_roll >> init_imu_pitch;
	fin.close();


    ros::Subscriber sub = n.subscribe<ros_socket::cmd_walk>("/cmd_walk",1,doWalkMsg);
    ros::param::set("stop_walk_flag",true);  //设置停止行走标志位于参数服务器中
    ros::param::set("stop_turn_flag",true);  //设置停止转弯标志位于参数服务器中
    ros::param::set("walk_with_ball",false);  //设置动态踢球标志位于参数服务器中
    ros::Subscriber specialGaitSuber = n.subscribe<std_msgs::Int16>("/special_gait",1,doSpecialGaitMsg);
    ros::param::set("stop_special_gait_flag", true);

    clearImuDataTxt();
    clearZmpDataTxt();
    ros::Rate rate(80);
    while (ros::ok())
    {
        bool stop_walk_flag;
        judgeFall();
        ros::param::get("stop_walk_flag",stop_walk_flag);
        if(!stop_walk_flag){
            trajPlan();
        }
        rate.sleep();
        ros::spinOnce();
    }
    
    
    return 0;
}
