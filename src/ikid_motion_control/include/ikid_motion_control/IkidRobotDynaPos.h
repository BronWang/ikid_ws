// 向仿真环境中的舵机发送消息的类

#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "robotModel.h"
extern robotLink robotModel[26];

class IkidRobotDynaPos
{
private:
    ros::NodeHandle n_;   
    ros::Publisher pub_left_ankle_front_swing;  
    ros::Publisher pub_left_ankle_side_swing;  
    ros::Publisher pub_left_arm_elbow_front_swing;  
    ros::Publisher pub_left_arm_front_swing;  
    ros::Publisher pub_left_arm_side_swing;  
    ros::Publisher pub_left_hip_front_swing;  
    ros::Publisher pub_left_hip_rotation;  
    ros::Publisher pub_left_hip_side_swing;  
    ros::Publisher pub_left_knee_front_swing;  
    ros::Publisher pub_neck_front_swing;  
    ros::Publisher pub_neck_rotation;  
    ros::Publisher pub_right_ankle_side_swing;  
    ros::Publisher pub_right_ankle_front_swing;  
    ros::Publisher pub_right_ankle_side_swing;  
    ros::Publisher pub_right_arm_elbow_front_swing;  
    ros::Publisher pub_right_arm_front_swing;  
    ros::Publisher pub_right_arm_side_swing;  
    ros::Publisher pub_right_hip_front_swing;  
    ros::Publisher pub_right_hip_rotation;  
    ros::Publisher pub_right_hip_side_swing;  
    ros::Publisher pub_right_knee_front_swing;  
    ros::Subscriber sub_; 
public:
    IkidRobotDynaPos();
    ~IkidRobotDynaPos();
    void pub();
};

IkidRobotDynaPos::IkidRobotDynaPos()
{
    //Topic you want to publish
    pub_left_ankle_front_swing = n_.advertise<std_msgs::Float64>("/ikid_robot/left_ankle_front_swing_position_controller/command", 100);  
    pub_left_ankle_side_swing = n_.advertise<std_msgs::Float64>("/ikid_robot/left_ankle_side_swing_position_controller/command", 100);  
    pub_left_arm_elbow_front_swing = n_.advertise<std_msgs::Float64>("/ikid_robot/left_arm_elbow_front_swing_position_controller/command", 100);  
    pub_left_arm_front_swing = n_.advertise<std_msgs::Float64>("/ikid_robot/left_arm_front_swing_position_controller/command", 100);  
    pub_left_arm_side_swing = n_.advertise<std_msgs::Float64>("/ikid_robot/left_arm_side_swing_position_controller/command", 100);  
    pub_left_hip_front_swing = n_.advertise<std_msgs::Float64>("/ikid_robot/left_hip_front_swing_position_controller/command", 100);  
    pub_left_hip_rotation = n_.advertise<std_msgs::Float64>("/ikid_robot/left_hip_rotation_position_controller/command", 100);  
    pub_left_hip_side_swing = n_.advertise<std_msgs::Float64>("/ikid_robot/left_hip_side_swing_position_controller/command", 100);  
    pub_left_knee_front_swing= n_.advertise<std_msgs::Float64>("/ikid_robot/left_knee_front_swing_position_controller/command", 100);  
    pub_neck_front_swing = n_.advertise<std_msgs::Float64>("/ikid_robot/neck_front_swing_position_controller/command", 100);  
    pub_neck_rotation = n_.advertise<std_msgs::Float64>("/ikid_robot/neck_rotation_position_controller/command", 100);  
    pub_right_ankle_front_swing = n_.advertise<std_msgs::Float64>("/ikid_robot/right_ankle_front_swing_position_controller/command", 100);  
    pub_right_ankle_side_swing = n_.advertise<std_msgs::Float64>("/ikid_robot/right_ankle_side_swing_position_controller/command", 100);  
    pub_right_arm_elbow_front_swing = n_.advertise<std_msgs::Float64>("/ikid_robot/right_arm_elbow_front_swing_position_controller/command", 100);  
    pub_right_arm_front_swing = n_.advertise<std_msgs::Float64>("/ikid_robot/right_arm_front_swing_position_controller/command", 100);  
    pub_right_arm_side_swing = n_.advertise<std_msgs::Float64>("/ikid_robot/right_arm_side_swing_position_controller/command", 100);  
    pub_right_hip_front_swing = n_.advertise<std_msgs::Float64>("/ikid_robot/right_hip_front_swing_position_controller/command", 100);  
    pub_right_hip_rotation = n_.advertise<std_msgs::Float64>("/ikid_robot/right_hip_rotation_position_controller/command", 100);  
    pub_right_hip_side_swing = n_.advertise<std_msgs::Float64>("/ikid_robot/right_hip_side_swing_position_controller/command", 100);  
    pub_right_knee_front_swing = n_.advertise<std_msgs::Float64>("/ikid_robot/right_knee_front_swing_position_controller/command", 100);  
      
    
    //Topic you want to subscribe  
    // sub_ = n_.subscribe("/subscribed_topic", 1, &SubscribeAndPublish::callback, this);  //注意这里，和平时使用回调函数不一样了。
}

IkidRobotDynaPos::~IkidRobotDynaPos()
{
}

void IkidRobotDynaPos::pub(){
    std_msgs::Float64 msg;
    msg.data = robotModel[FRONT_NECK_SWING].q;
    pub_neck_front_swing.publish(msg);
    
}

// void callback(const SUBSCRIBED_MESSAGE_TYPE& input){  
//     PUBLISHED_MESSAGE_TYPE output;  
//     //.... do something with the input and generate the output...  
//     pub_.publish(output);  
// }  
