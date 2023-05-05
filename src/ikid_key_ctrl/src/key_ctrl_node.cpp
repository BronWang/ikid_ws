#include "ikid_key_ctrl/ikid_key.h"
#include "ikid_key_ctrl/cmd_walk.h"
#include "ikid_key_ctrl/key_common.h"
#include "ros/ros.h"
#include <std_msgs/Int16.h>
const int KEYCODE_W = 0x77;
const int KEYCODE_A = 0x61;
const int KEYCODE_S = 0x73;
const int KEYCODE_D = 0x64;
const int KEYCODE_K = 0x6B;
 
const int KEYCODE_A_CAP = 0x41;
const int KEYCODE_D_CAP = 0x44;
const int KEYCODE_S_CAP = 0x53;
const int KEYCODE_W_CAP = 0x57;
 
const int KEYCODE_0 = 0x30;
const int KEYCODE_3 = 0x33;
const int KEYCODE_9 = 0x39;
 
// const int KEYCODE_SHIFT = 0x10;
// const int KEYCODE_CTRL = 0x11;
// const int KEYCODE_ALT = 0x11;
const int KEYCODE_ESC = 0x1B;
const int KEYCODE_ENTER = 0x0A;
const int KEYCODE_SPACE = 0x20;

double step_len = 0.1;
double step_wid = 0.0528 * 2;
 
using namespace KEY_CTRL;
int main(int argc, char **argv) {
  ros::init(argc, argv, "key_ctrl_node");
  ros::NodeHandle nh;
 
  ros::Publisher pub_keyboard = nh.advertise<ikid_key_ctrl::cmd_walk>("/cmd_walk", 5);
  ros::Publisher pub_keyboard2 = nh.advertise<std_msgs::Int16>("/special_gait", 1);
 
  ros::Rate loop_rate(10);
 
  auto KBC = Keyboard_ctrl();

  ros::param::get("/pid_amend/walk_length",step_len);
  ros::param::get("/pid_amend/walk_width",step_wid);
  while (ros::ok()) {
    ROS_INFO("input keyboard value (W forward, A left, S(unused), D right, SPACE stop):\n");
    auto key = KBC.get_keyboard_press_key();
    // ROS_INFO("get keyboard press 0x%02X \n", key);
    ikid_key_ctrl::cmd_walk walk_msg;
    std_msgs::Int16 msg;
    switch (key)
    {
        case KEYCODE_W:
        case KEYCODE_W_CAP:
            walk_msg.stop_walk = false;
            walk_msg.sx = step_len;
            walk_msg.sy = step_wid;
            walk_msg.var_theta = 0;
            walk_msg.walk_with_ball = false;
            pub_keyboard.publish(walk_msg);
            break;
        case KEYCODE_A:
        case KEYCODE_A_CAP:
            walk_msg.stop_walk = false;
            walk_msg.sx = step_len;
            walk_msg.sy = step_wid;
            walk_msg.var_theta = 25/180.0*M_PI;
            walk_msg.walk_with_ball = false;
            pub_keyboard.publish(walk_msg);
            break;
        case KEYCODE_S:
        case KEYCODE_S_CAP:
            break;
        case KEYCODE_D:
        case KEYCODE_D_CAP:
            walk_msg.stop_walk = false;
            walk_msg.sx = step_len;
            walk_msg.sy = step_wid;
            walk_msg.var_theta = -25/180.0*M_PI;
            walk_msg.walk_with_ball = false;
            pub_keyboard.publish(walk_msg);
            break;
        case KEYCODE_SPACE:
            bool stop_walk_flag;
            bool has_value;
            has_value = ros::param::get("stop_walk_flag",stop_walk_flag);
            if(has_value){
                if(!stop_walk_flag){
                    walk_msg.stop_walk = true;
                    walk_msg.sx = step_len;
                    walk_msg.sy = step_wid;
                    walk_msg.var_theta = 0;
                    walk_msg.walk_with_ball = false;
                    pub_keyboard.publish(walk_msg);
                } 
            }
            break;
        case KEYCODE_K:
            walk_msg.stop_walk = false;
            walk_msg.sx = step_len;
            walk_msg.sy = step_wid;
            walk_msg.var_theta = 0;
            walk_msg.walk_with_ball = true;
            pub_keyboard.publish(walk_msg);
            break;
        case KEYCODE_3:
            msg.data = 3;
            pub_keyboard2.publish(msg);
            break;
        default:
            break;
    }
 
    ros::spinOnce();
 
    loop_rate.sleep();
  }
  return 0;
}