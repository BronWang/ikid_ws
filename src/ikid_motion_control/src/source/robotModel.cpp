#include "ikid_motion_control/robotModel.h"
#include <math.h>
#include "ikid_motion_control/matlab_inv.h"
#include <string.h>
#include <stdio.h>
#include <dirent.h>
#include <sys/stat.h>
#include <fstream>
#include "std_msgs/Float64.h"
#include "std_msgs/Byte.h"
#include "ikid_motion_control/robot_joint.h"



robotLink robotModel[26];   // 机器人整体模型数组
static double eye[3][3] = { {1,0,0},{0,1,0},{0,0,1} };

// 特殊步态号
const int FALL_BACK_UP_ID = 1;
const int FALL_FORWARD_UP_ID = 2;

// 机器人零点数据
double ikid_robot_zero_point[26] = {0};

// 机器人零点IMU数据
double init_imu_roll = 0.0;
double init_imu_pitch = 0.0;
double imu_data_roll = 0.0;
double imu_data_yaw = 0.0;
double imu_data_pitch = 0.0;

// 步行单元帧数
const int step_basic_frame = 25;
// 双脚支撑帧数
const int ds_frame = 5;
// 单位为 米，千克，秒, 弧度
// 步长 step_x,这里规划先迈左脚摆右手
double sx = 0.1;
double sy = 0.0528 * 2;
// 肩长
double hy = 0.1125*2;
double hx = sx;
// 行走时腰部的高度
double	c_h = 0.35;
// 预加载可修改参数
double	c_h_para = 0.35;
// 初始手部离地面高度
double	hand_h = c_h-0.1;
// 脚步抬高
double fh = 0.05;
// 帧间隔时间
double frame_T = 0.02;
double T_cell = step_basic_frame * frame_T;

// 转弯角
double theta = 0;
// 脚步初始位置
double pn[3] = { 0,-sy / 2,0 };
// 初始摆动位置
double pn_hand[3] = { 0, hy / 2.0 };
// 左右臂初始摆动角度
double arm_swing_angle = 0;
double left_arm_swing_angle = 0;
double right_arm_swing_angle = 0;
// 当前的支撑点
double support_ZMP[3] = { 0,0,0 };
// 当前的期望ZMP点
double current_ZMP_point[3] = { 0,0,0 };
// 通过公式计算的较为准确的ZMP
double Compute_fact_zmp[3] = {0};

// 开始步行的起步标志位
bool ikid_start_walk_flag = true;
// 质心牵引向量PC_MAIN_BODY
double PC_MAIN_BODY[3] = { 0 };
// 质心世界坐标
double Com[3] = { 0 };
// 实际全身质心与ZMP生成的质心位置误差
double error_Com_ZmpCom[3] = { 0 };
double sum_error_Com_ZmpCom = 0;
// 计算动量和动量矩对时间的一阶导数
double pre_robot_P[3] = {0};
double cur_robot_P[3] = {0};
double pre_robot_L[3] = {0};
double cur_robot_L[3] = {0};
double robot_dPdt[3] = {0};
double robot_dLdt[3] = {0};
// 机器人在Z轴方向的偏摆力矩
double robot_taoz = 0;

// 记录机器人启动时的各关节位置，方便跌倒爬起的姿态恢复
double FallUpRobotPos_q[26] = {0};

// imu PID增益和变量
double imu_roll_p = 0;
double imu_roll_i = 0;
double imu_roll_d = 0;
double imu_roll_err = 0;
double imu_roll_err_sum = 0;
double imu_roll_err_partial = 0;
double imu_pitch_p = 0;
double imu_pitch_i = 0;
double imu_pitch_d = 0;
double imu_pitch_err = 0;
double imu_pitch_err_sum = 0;
double imu_pitch_err_partial = 0;
double imu_yaw_p = 0;
double imu_yaw_i = 0;
double imu_yaw_d = 0;
double imu_yaw_err = 0;
double imu_yaw_err_sum = 0;
double imu_yaw_err_partial = 0;
double imu_com_x_p =  0;
double imu_com_x_i =  0;
double imu_com_x_d =  0;
double imu_com_y_p =  0;
double imu_com_y_i =  0;
double imu_com_y_d =  0;
// 质心位置补偿
double com_x_compen = 0;
double com_y_compen = 0;

double com_const_x_compen = 0.001;
double com_const_y_compen = 0.001;
// 稳定姿态值
double stable_roll = 0;
double stable_pitch = 0;
double stable_yaw = 0;

// 向驱动版发送帧的间隔时间
double walk_frame_T = 0.02;
// 预加载步长步宽
double walk_length = 0.08;
double walk_width = 0.14;

// 偏摆力矩PID
double arm_p = 0;
double arm_i = 0;
double arm_d = 0;
double tao_err = 0;
double tao_err_sum = 0;
double tao_err_partial = 0;
// 偏摆力矩期望值
double stable_tao = 0;


bool isLeft = true;
bool isDsPhase = true;

double basic_left_hand[step_basic_frame][3] =
{ 0 };
double basic_right_hand[step_basic_frame][3] =
{ 0 };
double basic_left_foot[step_basic_frame][3] =
{ 0 };
double basic_right_foot[step_basic_frame][3] =
{ 0 };


double basic_left_foot_angle[step_basic_frame][3] =
{ 0 };
double basic_right_foot_angle[step_basic_frame][3] =
{ 0 };

// 设计ZMP预观控制器控制器所需要的参数，结果由matlab使用里卡提方程求解计算
double Q = 1;
double R = 1e-6;
double zc = c_h-0.059;
double dt = frame_T;
double gravity = 9.8;
double state_space_A[3][3] = { {1, dt, dt * dt / 2},
								{0, 1, dt},
								{0, 0,  1}};
double state_space_B[3] = { powf(dt,3) / 6, dt * dt / 2,dt };
double state_space_C[3] = { 1,0,-zc / gravity };
double state_space_Com[2][3] = {{0,0,0},{0,-0.2,-0.2}}; // 对速度和加速度设置合适的初值，来防止启动颠簸
double sum_e[2] = { 0,0 };
const int N_preview = 3 * step_basic_frame + 2 * ds_frame;
// 计算结果
double ks = 513.065437881280;
double kx[3] = { 9851.88969316012,	1990.28012970936,	56.0842014734676 };
double zmp_weight_f[N_preview] = {
	-513.065437881280,	-669.044492627062,	-787.219987623670,	-820.144778041021,	-784.735591163344,	
	-713.111266237544,	-631.117856575249,	-553.379682040566,	-485.524281248355,	-428.025315098025,	
	-379.266272082517,	-337.295676245997,	-300.545489048873,	-267.957472348649,	-238.867737752140,	
	-212.848618509385,	-189.588765921199,	-168.825449681171,	-150.316025369870,	-133.830999469237,	
	-119.155605186269,	-106.092748641192,	-94.4645867982977,	-84.1123848089720,	-74.8952033384214,	
	-66.6880693827485,	-59.3800871927426,	-52.8727176087803,	-47.0782941373585,	-41.9187621527260,	
	-37.3246001683038,	-33.2338826922904,	-29.5914543325218,	-26.3481951029935,	-23.4603640796227,	
	-20.8890125989995,	-18.5994601113972,	-16.5608266214973,	-14.7456160653243,	-13.1293453213723,	
	-11.6902139505115,	-10.4088102036766,	-9.26784929409089,	-8.25194036951309,	-7.34737902091173,	
	-6.54196252077760,	-5.82482529842180,	-5.18629243584815,	-4.61774921151216,	-4.11152493522781,	
	-3.66078950946079,	-3.25946132324272,	-2.90212523735726,	-2.58395955530568,	-2.30067099561974,	
	-2.04843678892821,	-1.82385311922127,	-1.62388921427297,	-1.44584646632633,	-1.28732203194441,	
	-1.14617642030094,	-1.02050463294051,	-0.90861046590631,	-0.80898362775839,	-0.72027936496178,	
	-0.64130031992237,	-0.57098037704553,	-0.50837027899181,	-0.45262481916853,	-0.40299143774610,	
	-0.35880006740983,	-0.31945409190749,	-0.28442229545623,	-0.25323169443224,	-0.22546115466303,	
	-0.20073570823493,	-0.17872149316096,	-0.15912124765336,	-0.14167029822398,	-0.12613298749508,	
	-0.11229949353325,	-0.09998299779912,	-0.08901716350743,	-0.07925389037833,	-0.07056131548892
};
// double ks = 513.065437881280;
// double kx[3] = { 9851.88969316012,	1990.28012970936,	56.0842014734676 };
// double zmp_weight_f[N_preview] = {
	
// };

// 定义机器人舵机信息发布句柄
#if ROSPUB
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
ros::Publisher pub_right_arm_elbow_front_swing;  
ros::Publisher pub_right_arm_front_swing;  
ros::Publisher pub_right_arm_side_swing;  
ros::Publisher pub_right_hip_front_swing;  
ros::Publisher pub_right_hip_rotation;  
ros::Publisher pub_right_hip_side_swing;  
ros::Publisher pub_right_knee_front_swing;
ros::Publisher pub_imu_data_roll;
ros::Publisher pub_imu_data_pitch;
ros::Publisher pub_imu_data_yaw;
#endif

#if CONTROLBOARDPUB
ros::Publisher pub_control_board_joint_msg;
ros::Publisher pub_control_board_torque_on;
#endif

void ikidRobotDynaPosPubInit(ros::NodeHandle& n_){
	//Topic you want to publish
	#if ROSPUB
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
	#endif
	
	#if CONTROLBOARDPUB
	pub_control_board_joint_msg = n_.advertise<ikid_motion_control::robot_joint>("/ikid_robot/control_board_joint_msg", 1000);
	pub_control_board_torque_on = n_.advertise<std_msgs::Byte>("/torque_on", 1000);
	#endif

    pub_imu_data_roll = n_.advertise<std_msgs::Float64>("/imu_data_roll", 100);
    pub_imu_data_pitch = n_.advertise<std_msgs::Float64>("/imu_data_pitch", 100);
    pub_imu_data_yaw = n_.advertise<std_msgs::Float64>("/imu_data_yaw", 100);
	//Topic you want to subscribe  
    // sub_ = n_.subscribe("/subscribed_topic", 1, &SubscribeAndPublish::callback, this);  //注意这里，和平时使用回调函数不一样了。
	
}

void ikidRobotDynaPosPub(){
	std_msgs::Float64 msg;
    msg.data = robotModel[FRONT_NECK_SWING].q + ikid_robot_zero_point[FRONT_NECK_SWING];
    pub_neck_front_swing.publish(msg);
    msg.data = robotModel[NECK_ROTATION].q + ikid_robot_zero_point[NECK_ROTATION];
    pub_neck_rotation.publish(msg);
    msg.data = robotModel[LEFT_ARM_FRONT_SWING].q + ikid_robot_zero_point[LEFT_ARM_FRONT_SWING];
    pub_left_arm_front_swing.publish(msg);
    msg.data = robotModel[LEFT_ARM_SIDE_SWING].q + ikid_robot_zero_point[LEFT_ARM_SIDE_SWING];
    pub_left_arm_side_swing.publish(msg);
    msg.data = robotModel[LEFT_ARM_ELBOW_FRONT_SWING].q + ikid_robot_zero_point[LEFT_ARM_ELBOW_FRONT_SWING];
    pub_left_arm_elbow_front_swing.publish(msg);
    msg.data = robotModel[RIGHT_ARM_FRONT_SWING].q + ikid_robot_zero_point[RIGHT_ARM_FRONT_SWING];
    pub_right_arm_front_swing.publish(msg);
    msg.data = robotModel[RIGHT_ARM_SIDE_SWING].q + ikid_robot_zero_point[RIGHT_ARM_SIDE_SWING];
    pub_right_arm_side_swing.publish(msg);
    msg.data = robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].q + ikid_robot_zero_point[RIGHT_ARM_ELBOW_FRONT_SWING];
    pub_right_arm_elbow_front_swing.publish(msg);
    msg.data = robotModel[LEFT_HIP_FRONT_SWING].q + ikid_robot_zero_point[LEFT_HIP_FRONT_SWING];
    pub_left_hip_front_swing.publish(msg);
    msg.data = robotModel[LEFT_HIP_SIDE_SWING].q + ikid_robot_zero_point[LEFT_HIP_SIDE_SWING];
    pub_left_hip_side_swing.publish(msg);
    msg.data = robotModel[LEFT_HIP_ROTATION].q + ikid_robot_zero_point[LEFT_HIP_ROTATION];
    pub_left_hip_rotation.publish(msg);
    msg.data = robotModel[RIGHT_HIP_FRONT_SWING].q + ikid_robot_zero_point[RIGHT_HIP_FRONT_SWING];
    pub_right_hip_front_swing.publish(msg);
    msg.data = robotModel[RIGHT_HIP_SIDE_SWING].q + ikid_robot_zero_point[RIGHT_HIP_SIDE_SWING];
    pub_right_hip_side_swing.publish(msg);
    msg.data = robotModel[RIGHT_HIP_ROTATION].q + ikid_robot_zero_point[RIGHT_HIP_ROTATION];
    pub_right_hip_rotation.publish(msg);
    msg.data = robotModel[LEFT_KNEE_FRONT_SWING].q + ikid_robot_zero_point[LEFT_KNEE_FRONT_SWING];
    pub_left_knee_front_swing.publish(msg);
    msg.data = robotModel[RIGHT_KNEE_FRONT_SWING].q + ikid_robot_zero_point[RIGHT_KNEE_FRONT_SWING];
    pub_right_knee_front_swing.publish(msg);
    msg.data = robotModel[LEFT_ANKLE_FRONT_SWING].q + ikid_robot_zero_point[LEFT_ANKLE_FRONT_SWING];
    pub_left_ankle_front_swing.publish(msg);
    msg.data = robotModel[LEFT_ANKLE_SIDE_SWING].q + ikid_robot_zero_point[LEFT_ANKLE_SIDE_SWING];
    pub_left_ankle_side_swing.publish(msg);
    msg.data = robotModel[RIGHT_ANKLE_FRONT_SWING].q + ikid_robot_zero_point[RIGHT_ANKLE_FRONT_SWING];
    pub_right_ankle_front_swing.publish(msg);
    msg.data = robotModel[RIGHT_ANKLE_SIDE_SWING].q + ikid_robot_zero_point[RIGHT_ANKLE_SIDE_SWING];
    pub_right_ankle_side_swing.publish(msg);
	ros::Duration(0.022).sleep();
}

void ikidRobotDynaPosPubSpecialGait(){
	std_msgs::Float64 msg;
    msg.data = robotModel[FRONT_NECK_SWING].q;
    pub_neck_front_swing.publish(msg);
    msg.data = robotModel[NECK_ROTATION].q;
    pub_neck_rotation.publish(msg);
    msg.data = robotModel[LEFT_ARM_FRONT_SWING].q;
    pub_left_arm_front_swing.publish(msg);
    msg.data = robotModel[LEFT_ARM_SIDE_SWING].q;
    pub_left_arm_side_swing.publish(msg);
    msg.data = robotModel[LEFT_ARM_ELBOW_FRONT_SWING].q;
    pub_left_arm_elbow_front_swing.publish(msg);
    msg.data = robotModel[RIGHT_ARM_FRONT_SWING].q;
    pub_right_arm_front_swing.publish(msg);
    msg.data = robotModel[RIGHT_ARM_SIDE_SWING].q;
    pub_right_arm_side_swing.publish(msg);
    msg.data = robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].q;
    pub_right_arm_elbow_front_swing.publish(msg);
    msg.data = robotModel[LEFT_HIP_FRONT_SWING].q;
    pub_left_hip_front_swing.publish(msg);
    msg.data = robotModel[LEFT_HIP_SIDE_SWING].q;
    pub_left_hip_side_swing.publish(msg);
    msg.data = robotModel[LEFT_HIP_ROTATION].q;
    pub_left_hip_rotation.publish(msg);
    msg.data = robotModel[RIGHT_HIP_FRONT_SWING].q;
    pub_right_hip_front_swing.publish(msg);
    msg.data = robotModel[RIGHT_HIP_SIDE_SWING].q;
    pub_right_hip_side_swing.publish(msg);
    msg.data = robotModel[RIGHT_HIP_ROTATION].q;
    pub_right_hip_rotation.publish(msg);
    msg.data = robotModel[LEFT_KNEE_FRONT_SWING].q;
    pub_left_knee_front_swing.publish(msg);
    msg.data = robotModel[RIGHT_KNEE_FRONT_SWING].q;
    pub_right_knee_front_swing.publish(msg);
    msg.data = robotModel[LEFT_ANKLE_FRONT_SWING].q;
    pub_left_ankle_front_swing.publish(msg);
    msg.data = robotModel[LEFT_ANKLE_SIDE_SWING].q;
    pub_left_ankle_side_swing.publish(msg);
    msg.data = robotModel[RIGHT_ANKLE_FRONT_SWING].q;
    pub_right_ankle_front_swing.publish(msg);
    msg.data = robotModel[RIGHT_ANKLE_SIDE_SWING].q;
    pub_right_ankle_side_swing.publish(msg);
	ros::Duration(0.022).sleep();
}

void ikidRobotDynaPosControlBoardPub(){
	ikid_motion_control::robot_joint control_board_joint_msg;
	control_board_joint_msg.joint = {
		0,
		0,
		robotModel[FRONT_NECK_SWING].q + ikid_robot_zero_point[FRONT_NECK_SWING],
		robotModel[NECK_ROTATION].q + ikid_robot_zero_point[NECK_ROTATION],
		robotModel[RIGHT_ARM_FRONT_SWING].q + ikid_robot_zero_point[RIGHT_ARM_FRONT_SWING],
		robotModel[RIGHT_ARM_SIDE_SWING].q + ikid_robot_zero_point[RIGHT_ARM_SIDE_SWING],
		robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].q + ikid_robot_zero_point[RIGHT_ARM_ELBOW_FRONT_SWING],
		0,
		robotModel[LEFT_ARM_FRONT_SWING].q + ikid_robot_zero_point[LEFT_ARM_FRONT_SWING],
		robotModel[LEFT_ARM_SIDE_SWING].q + ikid_robot_zero_point[LEFT_ARM_SIDE_SWING],
		robotModel[LEFT_ARM_ELBOW_FRONT_SWING].q + ikid_robot_zero_point[LEFT_ARM_ELBOW_FRONT_SWING],
		0,
		robotModel[RIGHT_HIP_FRONT_SWING].q + ikid_robot_zero_point[RIGHT_HIP_FRONT_SWING],
		robotModel[RIGHT_HIP_SIDE_SWING].q + ikid_robot_zero_point[RIGHT_HIP_SIDE_SWING],
		robotModel[RIGHT_HIP_ROTATION].q + ikid_robot_zero_point[RIGHT_HIP_ROTATION],
		robotModel[RIGHT_KNEE_FRONT_SWING].q + ikid_robot_zero_point[RIGHT_KNEE_FRONT_SWING],
		robotModel[RIGHT_ANKLE_FRONT_SWING].q + ikid_robot_zero_point[RIGHT_ANKLE_FRONT_SWING],
		robotModel[RIGHT_ANKLE_SIDE_SWING].q + ikid_robot_zero_point[RIGHT_ANKLE_SIDE_SWING],
		0,
		robotModel[LEFT_HIP_FRONT_SWING].q + ikid_robot_zero_point[LEFT_HIP_FRONT_SWING],
		robotModel[LEFT_HIP_SIDE_SWING].q + ikid_robot_zero_point[LEFT_HIP_SIDE_SWING],
		robotModel[LEFT_HIP_ROTATION].q + ikid_robot_zero_point[LEFT_HIP_ROTATION],
		robotModel[LEFT_KNEE_FRONT_SWING].q + ikid_robot_zero_point[LEFT_KNEE_FRONT_SWING],
		robotModel[LEFT_ANKLE_FRONT_SWING].q + ikid_robot_zero_point[LEFT_ANKLE_FRONT_SWING],
		robotModel[LEFT_ANKLE_SIDE_SWING].q + ikid_robot_zero_point[LEFT_ANKLE_SIDE_SWING],
		0
	};
	#if CONTROLBOARDPUB
	pub_control_board_joint_msg.publish(control_board_joint_msg);
	ros::Duration(walk_frame_T).sleep();
	#endif
}

void ikidRobotDynaPosControlBoardPubSpecialGait(){
	ikid_motion_control::robot_joint control_board_joint_msg;
	control_board_joint_msg.joint = {
		0,
		0,
		robotModel[FRONT_NECK_SWING].q,
		robotModel[NECK_ROTATION].q,
		robotModel[RIGHT_ARM_FRONT_SWING].q,
		robotModel[RIGHT_ARM_SIDE_SWING].q,
		robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].q,
		0,
		robotModel[LEFT_ARM_FRONT_SWING].q,
		robotModel[LEFT_ARM_SIDE_SWING].q,
		robotModel[LEFT_ARM_ELBOW_FRONT_SWING].q,
		0,
		robotModel[RIGHT_HIP_FRONT_SWING].q,
		robotModel[RIGHT_HIP_SIDE_SWING].q,
		robotModel[RIGHT_HIP_ROTATION].q,
		robotModel[RIGHT_KNEE_FRONT_SWING].q,
		robotModel[RIGHT_ANKLE_FRONT_SWING].q,
		robotModel[RIGHT_ANKLE_SIDE_SWING].q,
		0,
		robotModel[LEFT_HIP_FRONT_SWING].q,
		robotModel[LEFT_HIP_SIDE_SWING].q,
		robotModel[LEFT_HIP_ROTATION].q,
		robotModel[LEFT_KNEE_FRONT_SWING].q,
		robotModel[LEFT_ANKLE_FRONT_SWING].q,
		robotModel[LEFT_ANKLE_SIDE_SWING].q,
		0
	};
	#if CONTROLBOARDPUB
	pub_control_board_joint_msg.publish(control_board_joint_msg);
	#endif
}

void readIkidRobotZeroPoint(int id){
	DIR *dp = NULL;
	struct dirent *st;  // 文件夹中的子文件数据结构
	struct stat sta;
	int ret = 0;
	char tmp_name[1024] = {0};
	char path[50] = "/home/wp/ikid_ws/specialGaitFile\0";
	dp = opendir(path);
	if (dp == NULL)
	{
		printf("open dir error!!\n");
		return;
	}

	while (1)
	{
		st = readdir(dp);
		if (NULL == st) // 读取完毕
		{
			break;
		}
		strcpy(tmp_name, path);
		if (path[strlen(path) - 1] != '/') // 判断路径名是否带/
			strcat(tmp_name, "/");
		strcat(tmp_name, st->d_name); // 新文件路径名

		//获取文件中步态
		char c[300];
		char *f_ret;
		FILE *fptr = fopen(tmp_name, "r");
		if (fgets(c,sizeof(c),fptr) != NULL)
		{
			//ROS_INFO("%s",tmp_name);
			f_ret = fgets(c,sizeof(c),fptr);
			if(c[strlen(c)-1]=='\n') c[strlen(c)-1] = '\0';
			int temp_id = atoi(c);
			if (temp_id == id){
				//获取步态频率
				f_ret = fgets(c,sizeof(c),fptr);
				if(c[strlen(c)-1]=='\r') c[strlen(c)-1] = '\0'; // 由于windows和linux文本文件的换行符规则不同，这里统一消去
				if(c[strlen(c)-2]=='\r') c[strlen(c)-2] = '\0';
				if(c[strlen(c)-1]=='\n') c[strlen(c)-1] = '\0';
				if(c[strlen(c)-2]=='\n') c[strlen(c)-2] = '\0';
				if(strcmp(c,"GaitRate") != 0) break;
				f_ret = fgets(c,sizeof(c),fptr);
				if(c[strlen(c)-1]=='\r') c[strlen(c)-1] = '\0';
				if(c[strlen(c)-2]=='\r') c[strlen(c)-2] = '\0';
				if(c[strlen(c)-1]=='\n') c[strlen(c)-1] = '\0';
				if(c[strlen(c)-2]=='\n') c[strlen(c)-2] = '\0';
				int gaitRate = atoi(c);

				//读出步态描述
				f_ret = fgets(c,sizeof(c),fptr);
				if(c[strlen(c)-1]=='\r') c[strlen(c)-1] = '\0';
				if(c[strlen(c)-2]=='\r') c[strlen(c)-2] = '\0';
				if(c[strlen(c)-1]=='\n') c[strlen(c)-1] = '\0';
				if(c[strlen(c)-2]=='\n') c[strlen(c)-2] = '\0';
				if(strcmp(c,"GaitDescription") != 0) break;
				f_ret = fgets(c,sizeof(c),fptr);
				
				//读取步态零点
				f_ret = fgets(c,sizeof(c),fptr);
				if(c[strlen(c)-1]=='\r') c[strlen(c)-1] = '\0';
				if(c[strlen(c)-2]=='\r') c[strlen(c)-2] = '\0';
				if(c[strlen(c)-1]=='\n') c[strlen(c)-1] = '\0';
				if(c[strlen(c)-2]=='\n') c[strlen(c)-2] = '\0';
				if(strcmp(c,"zero_point") != 0) break;
				f_ret = fgets(c,sizeof(c),fptr);
				if(c[strlen(c)-1]=='\r') c[strlen(c)-1] = '\0';
				if(c[strlen(c)-2]=='\r') c[strlen(c)-2] = '\0';
				if(c[strlen(c)-1]=='\n') c[strlen(c)-1] = '\0';
				if(c[strlen(c)-2]=='\n') c[strlen(c)-2] = '\0';
				char* token;
                const char spl_chara[2] = ",";
                token = strtok(c,spl_chara);
                if(token != NULL){
					ikid_robot_zero_point[1] = atof(token);
                    for (int i = 2; i <= 25; i++)
                    {
                        token = strtok(NULL, ",");
                        ikid_robot_zero_point[i] = atof(token)/180*M_PI;
                    }
					printf("\n");
                }
				fclose(fptr);
				break;
			}
			else{
				fclose(fptr);
				continue;
			}
		}
	}
	closedir(dp);
}

void robotModelInit(robotLink* robotModel)
{
	// MAIN_BODY
	robotModel[MAIN_BODY].linkID = MAIN_BODY;
	robotModel[MAIN_BODY].name = "主体";
	robotModel[MAIN_BODY].sister = NONE_JOINT;
	robotModel[MAIN_BODY].child = NECK_ROTATION;
	robotModel[MAIN_BODY].mother = NONE_JOINT;
	robotModel[MAIN_BODY].p[0] = 0; robotModel[MAIN_BODY].p[1] = 0; robotModel[MAIN_BODY].p[2] = 0.3714;
	robotModel[MAIN_BODY].R[0][0] = 1; robotModel[MAIN_BODY].R[0][1] = 0; robotModel[MAIN_BODY].R[0][2] = 0; 
	robotModel[MAIN_BODY].R[1][0] = 0; robotModel[MAIN_BODY].R[1][1] = 1; robotModel[MAIN_BODY].R[1][2] = 0;
	robotModel[MAIN_BODY].R[2][0] = 0; robotModel[MAIN_BODY].R[2][1] = 0; robotModel[MAIN_BODY].R[2][2] = 1;
	robotModel[MAIN_BODY].v[0] = 0; robotModel[MAIN_BODY].v[1] = 0; robotModel[MAIN_BODY].v[2] = 0;
	robotModel[MAIN_BODY].w[0] = 0;	robotModel[MAIN_BODY].w[1] = 0; robotModel[MAIN_BODY].w[2] = 0;
	robotModel[MAIN_BODY].q = 0;			
	robotModel[MAIN_BODY].dq = 0;			
	robotModel[MAIN_BODY].ddq = 0;		
	robotModel[MAIN_BODY].a[0] = 0; robotModel[MAIN_BODY].a[1] = 0; robotModel[MAIN_BODY].a[2] = 1;// 主体的关节轴矢量用不着，这里顺便写
	robotModel[MAIN_BODY].b[0] = 0;	robotModel[MAIN_BODY].b[1] = 0; robotModel[MAIN_BODY].b[2] = 0;// 主体自身就是根节点
	robotModel[MAIN_BODY].m = 1.21871;			
	robotModel[MAIN_BODY].c[0] = 0.00461;	robotModel[MAIN_BODY].c[1] = 0.00038; robotModel[MAIN_BODY].c[2] = 0.04435;//这里把整体机器人抽象为一个质点，后续可以对每个连杆建模进行多质点研究
	robotModel[MAIN_BODY].I[0][0] = 0.005761; robotModel[MAIN_BODY].I[0][1] = 0; robotModel[MAIN_BODY].I[0][2] = 0;  // 目前把整体机器人抽象为一个质点，惯性张量先忽略不计，后续研究再使用
	robotModel[MAIN_BODY].I[1][0] = 0; robotModel[MAIN_BODY].I[1][1] = 0.005875; robotModel[MAIN_BODY].I[1][2] = 0;
	robotModel[MAIN_BODY].I[2][0] = 0; robotModel[MAIN_BODY].I[2][1] = 0; robotModel[MAIN_BODY].I[2][2] = 0.002850;

	// id 1
	robotModel[HEAD].linkID = HEAD;
	robotModel[HEAD].name = "头部";
	robotModel[HEAD].sister = NONE_JOINT;
	robotModel[HEAD].child = NONE_JOINT;
	robotModel[HEAD].mother = FRONT_NECK_SWING;
	robotModel[HEAD].p[0] = 0; robotModel[HEAD].p[1] = 0; robotModel[HEAD].p[2] = 0;
	robotModel[HEAD].R[0][0] = 1; robotModel[HEAD].R[0][1] = 0; robotModel[HEAD].R[0][2] = 0;
	robotModel[HEAD].R[1][0] = 0; robotModel[HEAD].R[1][1] = 1; robotModel[HEAD].R[1][2] = 0;
	robotModel[HEAD].R[2][0] = 0; robotModel[HEAD].R[2][1] = 0; robotModel[HEAD].R[2][2] = 1;
	robotModel[HEAD].v[0] = 0; robotModel[HEAD].v[1] = 0; robotModel[HEAD].v[2] = 0;
	robotModel[HEAD].w[0] = 0;	robotModel[HEAD].w[1] = 0; robotModel[HEAD].w[2] = 0;
	robotModel[HEAD].q = 0;
	robotModel[HEAD].dq = 0;
	robotModel[HEAD].ddq = 0;
	robotModel[HEAD].a[0] = 0; robotModel[HEAD].a[1] = 1; robotModel[HEAD].a[2] = 0;
	robotModel[HEAD].b[0] = 0;	robotModel[HEAD].b[1] = 0; robotModel[HEAD].b[2] = 0.0585;
	robotModel[HEAD].m = 0;
	robotModel[HEAD].c[0] = 0;	robotModel[HEAD].c[1] = 0; robotModel[HEAD].c[2] = 0;
	robotModel[HEAD].I[0][0] = 0; robotModel[HEAD].I[0][1] = 0; robotModel[HEAD].I[0][2] = 0;
	robotModel[HEAD].I[1][0] = 0; robotModel[HEAD].I[1][1] = 0; robotModel[HEAD].I[1][2] = 0;
	robotModel[HEAD].I[2][0] = 0; robotModel[HEAD].I[2][1] = 0; robotModel[HEAD].I[2][2] = 0;

	// id 2
	robotModel[FRONT_NECK_SWING].linkID = FRONT_NECK_SWING;
	robotModel[FRONT_NECK_SWING].name = "颈前摆";
	robotModel[FRONT_NECK_SWING].sister = NONE_JOINT;
	robotModel[FRONT_NECK_SWING].child = HEAD;
	robotModel[FRONT_NECK_SWING].mother = NECK_ROTATION;
	robotModel[FRONT_NECK_SWING].p[0] = 0; robotModel[FRONT_NECK_SWING].p[1] = 0; robotModel[FRONT_NECK_SWING].p[2] = 0;
	robotModel[FRONT_NECK_SWING].R[0][0] = 1; robotModel[FRONT_NECK_SWING].R[0][1] = 0; robotModel[FRONT_NECK_SWING].R[0][2] = 0;
	robotModel[FRONT_NECK_SWING].R[1][0] = 0; robotModel[FRONT_NECK_SWING].R[1][1] = 1; robotModel[FRONT_NECK_SWING].R[1][2] = 0;
	robotModel[FRONT_NECK_SWING].R[2][0] = 0; robotModel[FRONT_NECK_SWING].R[2][1] = 0; robotModel[FRONT_NECK_SWING].R[2][2] = 1;
	robotModel[FRONT_NECK_SWING].v[0] = 0; robotModel[FRONT_NECK_SWING].v[1] = 0; robotModel[FRONT_NECK_SWING].v[2] = 0;
	robotModel[FRONT_NECK_SWING].w[0] = 0;	robotModel[FRONT_NECK_SWING].w[1] = 0; robotModel[FRONT_NECK_SWING].w[2] = 0;
	robotModel[FRONT_NECK_SWING].q = 0;
	robotModel[FRONT_NECK_SWING].dq = 0;
	robotModel[FRONT_NECK_SWING].ddq = 0;
	robotModel[FRONT_NECK_SWING].a[0] = 0; robotModel[FRONT_NECK_SWING].a[1] = 1; robotModel[FRONT_NECK_SWING].a[2] = 0;
	robotModel[FRONT_NECK_SWING].b[0] = 0;	robotModel[FRONT_NECK_SWING].b[1] = 0; robotModel[FRONT_NECK_SWING].b[2] = 0.043;
	robotModel[FRONT_NECK_SWING].m = 0.33335;
	robotModel[FRONT_NECK_SWING].c[0] = -0.02112;	robotModel[FRONT_NECK_SWING].c[1] = 0.00080; robotModel[FRONT_NECK_SWING].c[2] = 0.04113;
	robotModel[FRONT_NECK_SWING].I[0][0] = 0.001051; robotModel[FRONT_NECK_SWING].I[0][1] = 0; robotModel[FRONT_NECK_SWING].I[0][2] = 0;
	robotModel[FRONT_NECK_SWING].I[1][0] = 0; robotModel[FRONT_NECK_SWING].I[1][1] = 0.001219; robotModel[FRONT_NECK_SWING].I[1][2] = 0;
	robotModel[FRONT_NECK_SWING].I[2][0] = 0; robotModel[FRONT_NECK_SWING].I[2][1] = 0; robotModel[FRONT_NECK_SWING].I[2][2] = 0.000418;

	// id 3
	robotModel[NECK_ROTATION].linkID = NECK_ROTATION;
	robotModel[NECK_ROTATION].name = "颈旋转";
	robotModel[NECK_ROTATION].sister = RIGHT_ARM_FRONT_SWING;
	robotModel[NECK_ROTATION].child = FRONT_NECK_SWING;
	robotModel[NECK_ROTATION].mother = MAIN_BODY;
	robotModel[NECK_ROTATION].p[0] = 0; robotModel[NECK_ROTATION].p[1] = 0; robotModel[NECK_ROTATION].p[2] = 0;
	robotModel[NECK_ROTATION].R[0][0] = 1; robotModel[NECK_ROTATION].R[0][1] = 0; robotModel[NECK_ROTATION].R[0][2] = 0;
	robotModel[NECK_ROTATION].R[1][0] = 0; robotModel[NECK_ROTATION].R[1][1] = 1; robotModel[NECK_ROTATION].R[1][2] = 0;
	robotModel[NECK_ROTATION].R[2][0] = 0; robotModel[NECK_ROTATION].R[2][1] = 0; robotModel[NECK_ROTATION].R[2][2] = 1;
	robotModel[NECK_ROTATION].v[0] = 0; robotModel[NECK_ROTATION].v[1] = 0; robotModel[NECK_ROTATION].v[2] = 0;
	robotModel[NECK_ROTATION].w[0] = 0;	robotModel[NECK_ROTATION].w[1] = 0; robotModel[NECK_ROTATION].w[2] = 0;
	robotModel[NECK_ROTATION].q = 0;
	robotModel[NECK_ROTATION].dq = 0;
	robotModel[NECK_ROTATION].ddq = 0;
	robotModel[NECK_ROTATION].a[0] = 0; robotModel[NECK_ROTATION].a[1] = 0; robotModel[NECK_ROTATION].a[2] = 1;
	robotModel[NECK_ROTATION].b[0] = 0;	robotModel[NECK_ROTATION].b[1] = 0; robotModel[NECK_ROTATION].b[2] = 0.1042;
	robotModel[NECK_ROTATION].m = 0.07700;
	robotModel[NECK_ROTATION].c[0] = -0.01126;	robotModel[NECK_ROTATION].c[1] = 0; robotModel[NECK_ROTATION].c[2] = 0.00039;
	robotModel[NECK_ROTATION].I[0][0] = 0; robotModel[NECK_ROTATION].I[0][1] = 0; robotModel[NECK_ROTATION].I[0][2] = 0;
	robotModel[NECK_ROTATION].I[1][0] = 0; robotModel[NECK_ROTATION].I[1][1] = 0; robotModel[NECK_ROTATION].I[1][2] = 0;
	robotModel[NECK_ROTATION].I[2][0] = 0; robotModel[NECK_ROTATION].I[2][1] = 0; robotModel[NECK_ROTATION].I[2][2] = 0;
	
	// id 4
	robotModel[RIGHT_ARM_FRONT_SWING].linkID = RIGHT_ARM_FRONT_SWING;
	robotModel[RIGHT_ARM_FRONT_SWING].name = "右大臂前摆";
	robotModel[RIGHT_ARM_FRONT_SWING].sister = LEFT_ARM_FRONT_SWING;
	robotModel[RIGHT_ARM_FRONT_SWING].child = RIGHT_ARM_SIDE_SWING;
	robotModel[RIGHT_ARM_FRONT_SWING].mother = MAIN_BODY;
	robotModel[RIGHT_ARM_FRONT_SWING].p[0] = 0; robotModel[RIGHT_ARM_FRONT_SWING].p[1] = 0; robotModel[RIGHT_ARM_FRONT_SWING ].p[2] = 0;
	robotModel[RIGHT_ARM_FRONT_SWING].R[0][0] = 1; robotModel[RIGHT_ARM_FRONT_SWING].R[0][1] = 0; robotModel[RIGHT_ARM_FRONT_SWING].R[0][2] = 0;
	robotModel[RIGHT_ARM_FRONT_SWING].R[1][0] = 0; robotModel[RIGHT_ARM_FRONT_SWING].R[1][1] = 1; robotModel[RIGHT_ARM_FRONT_SWING].R[1][2] = 0;
	robotModel[RIGHT_ARM_FRONT_SWING].R[2][0] = 0; robotModel[RIGHT_ARM_FRONT_SWING].R[2][1] = 0; robotModel[RIGHT_ARM_FRONT_SWING].R[2][2] = 1;
	robotModel[RIGHT_ARM_FRONT_SWING].v[0] = 0; robotModel[RIGHT_ARM_FRONT_SWING].v[1] = 0; robotModel[RIGHT_ARM_FRONT_SWING].v[2] = 0;
	robotModel[RIGHT_ARM_FRONT_SWING].w[0] = 0;	robotModel[RIGHT_ARM_FRONT_SWING].w[1] = 0; robotModel[RIGHT_ARM_FRONT_SWING].w[2] = 0;
	robotModel[RIGHT_ARM_FRONT_SWING].q = 0;
	robotModel[RIGHT_ARM_FRONT_SWING].dq = 0;
	robotModel[RIGHT_ARM_FRONT_SWING].ddq = 0;
	robotModel[RIGHT_ARM_FRONT_SWING].a[0] = 0; robotModel[RIGHT_ARM_FRONT_SWING].a[1] = 1; robotModel[RIGHT_ARM_FRONT_SWING].a[2] = 0;
	robotModel[RIGHT_ARM_FRONT_SWING].b[0] = 0;	robotModel[RIGHT_ARM_FRONT_SWING].b[1] = -0.0632; robotModel[RIGHT_ARM_FRONT_SWING].b[2] = 0.1042;
	robotModel[RIGHT_ARM_FRONT_SWING].m = 0.09044;
	robotModel[RIGHT_ARM_FRONT_SWING].c[0] = -0.00955;	robotModel[RIGHT_ARM_FRONT_SWING].c[1] = -0.00510; robotModel[RIGHT_ARM_FRONT_SWING].c[2] = 0.00011;
	robotModel[RIGHT_ARM_FRONT_SWING].I[0][0] = 0; robotModel[RIGHT_ARM_FRONT_SWING].I[0][1] = 0; robotModel[RIGHT_ARM_FRONT_SWING].I[0][2] = 0;
	robotModel[RIGHT_ARM_FRONT_SWING].I[1][0] = 0; robotModel[RIGHT_ARM_FRONT_SWING].I[1][1] = 0; robotModel[RIGHT_ARM_FRONT_SWING].I[1][2] = 0;
	robotModel[RIGHT_ARM_FRONT_SWING].I[2][0] = 0; robotModel[RIGHT_ARM_FRONT_SWING].I[2][1] = 0; robotModel[RIGHT_ARM_FRONT_SWING].I[2][2] = 0;

	// id 5
	robotModel[RIGHT_ARM_SIDE_SWING].linkID = RIGHT_ARM_SIDE_SWING;
	robotModel[RIGHT_ARM_SIDE_SWING].name = "右大臂侧摆";
	robotModel[RIGHT_ARM_SIDE_SWING].sister = NONE_JOINT;
	robotModel[RIGHT_ARM_SIDE_SWING].child = RIGHT_ARM_ELBOW_FRONT_SWING;
	robotModel[RIGHT_ARM_SIDE_SWING].mother = RIGHT_ARM_FRONT_SWING;
	robotModel[RIGHT_ARM_SIDE_SWING].p[0] = 0; robotModel[RIGHT_ARM_SIDE_SWING].p[1] = 0; robotModel[RIGHT_ARM_SIDE_SWING].p[2] = 0;
	robotModel[RIGHT_ARM_SIDE_SWING].R[0][0] = 1; robotModel[RIGHT_ARM_SIDE_SWING].R[0][1] = 0; robotModel[RIGHT_ARM_SIDE_SWING].R[0][2] = 0;
	robotModel[RIGHT_ARM_SIDE_SWING].R[1][0] = 0; robotModel[RIGHT_ARM_SIDE_SWING].R[1][1] = 1; robotModel[RIGHT_ARM_SIDE_SWING].R[1][2] = 0;
	robotModel[RIGHT_ARM_SIDE_SWING].R[2][0] = 0; robotModel[RIGHT_ARM_SIDE_SWING].R[2][1] = 0; robotModel[RIGHT_ARM_SIDE_SWING].R[2][2] = 1;
	robotModel[RIGHT_ARM_SIDE_SWING].v[0] = 0; robotModel[RIGHT_ARM_SIDE_SWING].v[1] = 0; robotModel[RIGHT_ARM_SIDE_SWING].v[2] = 0;
	robotModel[RIGHT_ARM_SIDE_SWING].w[0] = 0;	robotModel[RIGHT_ARM_SIDE_SWING].w[1] = 0; robotModel[RIGHT_ARM_SIDE_SWING].w[2] = 0;
	robotModel[RIGHT_ARM_SIDE_SWING].q = 0;
	robotModel[RIGHT_ARM_SIDE_SWING].dq = 0;
	robotModel[RIGHT_ARM_SIDE_SWING].ddq = 0;
	robotModel[RIGHT_ARM_SIDE_SWING].a[0] = 1; robotModel[RIGHT_ARM_SIDE_SWING].a[1] = 0; robotModel[RIGHT_ARM_SIDE_SWING].a[2] = 0;
	robotModel[RIGHT_ARM_SIDE_SWING].b[0] = 0;	robotModel[RIGHT_ARM_SIDE_SWING].b[1] = -0.0493; robotModel[RIGHT_ARM_SIDE_SWING].b[2] = 0;
	robotModel[RIGHT_ARM_SIDE_SWING].m = 0.09274;
	robotModel[RIGHT_ARM_SIDE_SWING].c[0] = 0.00032;	robotModel[RIGHT_ARM_SIDE_SWING].c[1] = 0; robotModel[RIGHT_ARM_SIDE_SWING].c[2] = -0.02095;
	robotModel[RIGHT_ARM_SIDE_SWING].I[0][0] = 0.000110; robotModel[RIGHT_ARM_SIDE_SWING].I[0][1] = 0; robotModel[RIGHT_ARM_SIDE_SWING].I[0][2] = 0;
	robotModel[RIGHT_ARM_SIDE_SWING].I[1][0] = 0; robotModel[RIGHT_ARM_SIDE_SWING].I[1][1] = 0.000116; robotModel[RIGHT_ARM_SIDE_SWING].I[1][2] = 0;
	robotModel[RIGHT_ARM_SIDE_SWING].I[2][0] = 0; robotModel[RIGHT_ARM_SIDE_SWING].I[2][1] = 0; robotModel[RIGHT_ARM_SIDE_SWING].I[2][2] = 0.00002;

	// id 6
	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].linkID = RIGHT_ARM_ELBOW_FRONT_SWING;
	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].name = "右臂肘前摆";
	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].sister = NONE_JOINT;
	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].child = RIGHT_HAND;
	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].mother = RIGHT_ARM_SIDE_SWING;
	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].p[0] = 0; robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].p[1] = 0; robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].p[2] = 0;
	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].R[0][0] = 1; robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].R[0][1] = 0; robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].R[0][2] = 0;
	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].R[1][0] = 0; robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].R[1][1] = 1; robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].R[1][2] = 0;
	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].R[2][0] = 0; robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].R[2][1] = 0; robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].R[2][2] = 1;
	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].v[0] = 0; robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].v[1] = 0; robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].v[2] = 0;
	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].w[0] = 0;	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].w[1] = 0; robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].w[2] = 0;
	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].q = 0;
	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].dq = 0;
	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].ddq = 0;
	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].a[0] = 0; robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].a[1] = 1; robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].a[2] = 0;
	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].b[0] = 0;	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].b[1] = 0; robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].b[2] = -0.1091;
	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].m = 0.11057;
	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].c[0] = -0.00403;	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].c[1] = -0.00269; robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].c[2] = -0.01070;
	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].I[0][0] = 0.000269; robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].I[0][1] = 0; robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].I[0][2] = 0;
	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].I[1][0] = 0; robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].I[1][1] = 0.000264; robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].I[1][2] = 0;
	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].I[2][0] = 0; robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].I[2][1] = 0; robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].I[2][2] = 0.000034;

	// id 7
	robotModel[RIGHT_HAND].linkID = RIGHT_HAND;
	robotModel[RIGHT_HAND].name = "右手端";
	robotModel[RIGHT_HAND].sister = NONE_JOINT;
	robotModel[RIGHT_HAND].child = NONE_JOINT;
	robotModel[RIGHT_HAND].mother = RIGHT_ARM_ELBOW_FRONT_SWING;
	robotModel[RIGHT_HAND].p[0] = 0; robotModel[RIGHT_HAND].p[1] = 0; robotModel[RIGHT_HAND].p[2] = 0;
	robotModel[RIGHT_HAND].R[0][0] = 1; robotModel[RIGHT_HAND].R[0][1] = 0; robotModel[RIGHT_HAND].R[0][2] = 0;
	robotModel[RIGHT_HAND].R[1][0] = 0; robotModel[RIGHT_HAND].R[1][1] = 1; robotModel[RIGHT_HAND].R[1][2] = 0;
	robotModel[RIGHT_HAND].R[2][0] = 0; robotModel[RIGHT_HAND].R[2][1] = 0; robotModel[RIGHT_HAND].R[2][2] = 1;
	robotModel[RIGHT_HAND].v[0] = 0; robotModel[RIGHT_HAND].v[1] = 0; robotModel[RIGHT_HAND].v[2] = 0;
	robotModel[RIGHT_HAND].w[0] = 0; robotModel[RIGHT_HAND].w[1] = 0; robotModel[RIGHT_HAND].w[2] = 0;
	robotModel[RIGHT_HAND].q = 0;
	robotModel[RIGHT_HAND].dq = 0;
	robotModel[RIGHT_HAND].ddq = 0;
	robotModel[RIGHT_HAND].a[0] = 0; robotModel[RIGHT_HAND].a[1] = 1; robotModel[RIGHT_HAND].a[2] = 0;
	robotModel[RIGHT_HAND].b[0] = 0;	robotModel[RIGHT_HAND].b[1] = 0; robotModel[RIGHT_HAND].b[2] = -0.14;
	robotModel[RIGHT_HAND].m = 0;
	robotModel[RIGHT_HAND].c[0] = 0;	robotModel[RIGHT_HAND].c[1] = 0; robotModel[RIGHT_HAND].c[2] = 0;
	robotModel[RIGHT_HAND].I[0][0] = 0; robotModel[RIGHT_HAND].I[0][1] = 0; robotModel[RIGHT_HAND].I[0][2] = 0;
	robotModel[RIGHT_HAND].I[1][0] = 0; robotModel[RIGHT_HAND].I[1][1] = 0; robotModel[RIGHT_HAND].I[1][2] = 0;
	robotModel[RIGHT_HAND].I[2][0] = 0; robotModel[RIGHT_HAND].I[2][1] = 0; robotModel[RIGHT_HAND].I[2][2] = 0;

	// id 8
	robotModel[LEFT_ARM_FRONT_SWING].linkID = LEFT_ARM_FRONT_SWING;
	robotModel[LEFT_ARM_FRONT_SWING].name = "左大臂前摆";
	robotModel[LEFT_ARM_FRONT_SWING].sister = RIGHT_HIP_FRONT_SWING;
	robotModel[LEFT_ARM_FRONT_SWING].child = LEFT_ARM_SIDE_SWING;
	robotModel[LEFT_ARM_FRONT_SWING].mother = MAIN_BODY;
	robotModel[LEFT_ARM_FRONT_SWING].p[0] = 0; robotModel[LEFT_ARM_FRONT_SWING].p[1] = 0; robotModel[LEFT_ARM_FRONT_SWING].p[2] = 0;
	robotModel[LEFT_ARM_FRONT_SWING].R[0][0] = 1; robotModel[LEFT_ARM_FRONT_SWING].R[0][1] = 0; robotModel[LEFT_ARM_FRONT_SWING].R[0][2] = 0;
	robotModel[LEFT_ARM_FRONT_SWING].R[1][0] = 0; robotModel[LEFT_ARM_FRONT_SWING].R[1][1] = 1; robotModel[LEFT_ARM_FRONT_SWING].R[1][2] = 0;
	robotModel[LEFT_ARM_FRONT_SWING].R[2][0] = 0; robotModel[LEFT_ARM_FRONT_SWING].R[2][1] = 0; robotModel[LEFT_ARM_FRONT_SWING].R[2][2] = 1;
	robotModel[LEFT_ARM_FRONT_SWING].v[0] = 0; robotModel[LEFT_ARM_FRONT_SWING].v[1] = 0; robotModel[LEFT_ARM_FRONT_SWING].v[2] = 0;
	robotModel[LEFT_ARM_FRONT_SWING].w[0] = 0;	robotModel[LEFT_ARM_FRONT_SWING].w[1] = 0; robotModel[LEFT_ARM_FRONT_SWING].w[2] = 0;
	robotModel[LEFT_ARM_FRONT_SWING].q = 0;
	robotModel[LEFT_ARM_FRONT_SWING].dq = 0;
	robotModel[LEFT_ARM_FRONT_SWING].ddq = 0;
	robotModel[LEFT_ARM_FRONT_SWING].a[0] = 0; robotModel[LEFT_ARM_FRONT_SWING].a[1] = 1; robotModel[LEFT_ARM_FRONT_SWING].a[2] = 0;
	robotModel[LEFT_ARM_FRONT_SWING].b[0] = 0;	robotModel[LEFT_ARM_FRONT_SWING].b[1] = 0.0632; robotModel[LEFT_ARM_FRONT_SWING].b[2] = 0.1042;
	robotModel[LEFT_ARM_FRONT_SWING].m = 0.09044;
	robotModel[LEFT_ARM_FRONT_SWING].c[0] = -0.00955;	robotModel[LEFT_ARM_FRONT_SWING].c[1] = 0.00506; robotModel[LEFT_ARM_FRONT_SWING].c[2] = 0.00011;
	robotModel[LEFT_ARM_FRONT_SWING].I[0][0] = 0; robotModel[LEFT_ARM_FRONT_SWING].I[0][1] = 0; robotModel[LEFT_ARM_FRONT_SWING].I[0][2] = 0;
	robotModel[LEFT_ARM_FRONT_SWING].I[1][0] = 0; robotModel[LEFT_ARM_FRONT_SWING].I[1][1] = 0; robotModel[LEFT_ARM_FRONT_SWING].I[1][2] = 0;
	robotModel[LEFT_ARM_FRONT_SWING].I[2][0] = 0; robotModel[LEFT_ARM_FRONT_SWING].I[2][1] = 0; robotModel[LEFT_ARM_FRONT_SWING].I[2][2] = 0;

	// id 9
	robotModel[LEFT_ARM_SIDE_SWING].linkID = LEFT_ARM_SIDE_SWING;
	robotModel[LEFT_ARM_SIDE_SWING].name = "左大臂侧摆";
	robotModel[LEFT_ARM_SIDE_SWING].sister = NONE_JOINT;
	robotModel[LEFT_ARM_SIDE_SWING].child = LEFT_ARM_ELBOW_FRONT_SWING;
	robotModel[LEFT_ARM_SIDE_SWING].mother = LEFT_ARM_FRONT_SWING;
	robotModel[LEFT_ARM_SIDE_SWING].p[0] = 0; robotModel[LEFT_ARM_SIDE_SWING].p[1] = 0; robotModel[LEFT_ARM_SIDE_SWING].p[2] = 0;
	robotModel[LEFT_ARM_SIDE_SWING].R[0][0] = 1; robotModel[LEFT_ARM_SIDE_SWING].R[0][1] = 0; robotModel[LEFT_ARM_SIDE_SWING].R[0][2] = 0;
	robotModel[LEFT_ARM_SIDE_SWING].R[1][0] = 0; robotModel[LEFT_ARM_SIDE_SWING].R[1][1] = 1; robotModel[LEFT_ARM_SIDE_SWING].R[1][2] = 0;
	robotModel[LEFT_ARM_SIDE_SWING].R[2][0] = 0; robotModel[LEFT_ARM_SIDE_SWING].R[2][1] = 0; robotModel[LEFT_ARM_SIDE_SWING].R[2][2] = 1;
	robotModel[LEFT_ARM_SIDE_SWING].v[0] = 0; robotModel[LEFT_ARM_SIDE_SWING].v[1] = 0; robotModel[LEFT_ARM_SIDE_SWING].v[2] = 0;
	robotModel[LEFT_ARM_SIDE_SWING].w[0] = 0;	robotModel[LEFT_ARM_SIDE_SWING].w[1] = 0; robotModel[LEFT_ARM_SIDE_SWING].w[2] = 0;
	robotModel[LEFT_ARM_SIDE_SWING].q = 0;
	robotModel[LEFT_ARM_SIDE_SWING].dq = 0;
	robotModel[LEFT_ARM_SIDE_SWING].ddq = 0;
	robotModel[LEFT_ARM_SIDE_SWING].a[0] = 1; robotModel[LEFT_ARM_SIDE_SWING].a[1] = 0; robotModel[LEFT_ARM_SIDE_SWING].a[2] = 0;
	robotModel[LEFT_ARM_SIDE_SWING].b[0] = 0;	robotModel[LEFT_ARM_SIDE_SWING].b[1] = 0.0493; robotModel[LEFT_ARM_SIDE_SWING].b[2] = 0;
	robotModel[LEFT_ARM_SIDE_SWING].m = 0.09274;
	robotModel[LEFT_ARM_SIDE_SWING].c[0] = -0.00032;	robotModel[LEFT_ARM_SIDE_SWING].c[1] = 0; robotModel[LEFT_ARM_SIDE_SWING].c[2] = -0.02095;
	robotModel[LEFT_ARM_SIDE_SWING].I[0][0] = 0.000110; robotModel[LEFT_ARM_SIDE_SWING].I[0][1] = 0; robotModel[LEFT_ARM_SIDE_SWING].I[0][2] = 0;
	robotModel[LEFT_ARM_SIDE_SWING].I[1][0] = 0; robotModel[LEFT_ARM_SIDE_SWING].I[1][1] = 0.000116; robotModel[LEFT_ARM_SIDE_SWING].I[1][2] = 0;
	robotModel[LEFT_ARM_SIDE_SWING].I[2][0] = 0; robotModel[LEFT_ARM_SIDE_SWING].I[2][1] = 0; robotModel[LEFT_ARM_SIDE_SWING].I[2][2] = 0.00002;

	// id 10   
	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].linkID = LEFT_ARM_ELBOW_FRONT_SWING;
	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].name = "左臂肘前摆";
	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].sister = NONE_JOINT;
	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].child = LEFT_HAND;
	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].mother = LEFT_ARM_SIDE_SWING;
	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].p[0] = 0; robotModel[LEFT_ARM_ELBOW_FRONT_SWING].p[1] = 0; robotModel[LEFT_ARM_ELBOW_FRONT_SWING].p[2] = 0;
	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].R[0][0] = 1; robotModel[LEFT_ARM_ELBOW_FRONT_SWING].R[0][1] = 0; robotModel[LEFT_ARM_ELBOW_FRONT_SWING].R[0][2] = 0;
	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].R[1][0] = 0; robotModel[LEFT_ARM_ELBOW_FRONT_SWING].R[1][1] = 1; robotModel[LEFT_ARM_ELBOW_FRONT_SWING].R[1][2] = 0;
	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].R[2][0] = 0; robotModel[LEFT_ARM_ELBOW_FRONT_SWING].R[2][1] = 0; robotModel[LEFT_ARM_ELBOW_FRONT_SWING].R[2][2] = 1;
	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].v[0] = 0; robotModel[LEFT_ARM_ELBOW_FRONT_SWING].v[1] = 0; robotModel[LEFT_ARM_ELBOW_FRONT_SWING].v[2] = 0;
	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].w[0] = 0;	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].w[1] = 0; robotModel[LEFT_ARM_ELBOW_FRONT_SWING].w[2] = 0;
	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].q = 0;
	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].dq = 0;
	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].ddq = 0;
	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].a[0] = 0; robotModel[LEFT_ARM_ELBOW_FRONT_SWING].a[1] = 1; robotModel[LEFT_ARM_ELBOW_FRONT_SWING].a[2] = 0;
	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].b[0] = 0;	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].b[1] = 0; robotModel[LEFT_ARM_ELBOW_FRONT_SWING].b[2] = -0.1091;
	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].m = 0.11269;
	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].c[0] = -0.00356;	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].c[1] = 0.00263; robotModel[LEFT_ARM_ELBOW_FRONT_SWING].c[2] = -0.00845;
	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].I[0][0] = 0.000295; robotModel[LEFT_ARM_ELBOW_FRONT_SWING].I[0][1] = 0; robotModel[LEFT_ARM_ELBOW_FRONT_SWING].I[0][2] = 0;
	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].I[1][0] = 0; robotModel[LEFT_ARM_ELBOW_FRONT_SWING].I[1][1] = 0.000291; robotModel[LEFT_ARM_ELBOW_FRONT_SWING].I[1][2] = 0;
	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].I[2][0] = 0; robotModel[LEFT_ARM_ELBOW_FRONT_SWING].I[2][1] = 0; robotModel[LEFT_ARM_ELBOW_FRONT_SWING].I[2][2] = 0.000035;

	// id 11
	robotModel[LEFT_HAND].linkID = LEFT_HAND;
	robotModel[LEFT_HAND].name = "左手端";
	robotModel[LEFT_HAND].sister = NONE_JOINT;
	robotModel[LEFT_HAND].child = NONE_JOINT;
	robotModel[LEFT_HAND].mother = LEFT_ARM_ELBOW_FRONT_SWING;
	robotModel[LEFT_HAND].p[0] = 0; robotModel[LEFT_HAND].p[1] = 0; robotModel[LEFT_HAND].p[2] = 0;
	robotModel[LEFT_HAND].R[0][0] = 1; robotModel[LEFT_HAND].R[0][1] = 0; robotModel[LEFT_HAND].R[0][2] = 0;
	robotModel[LEFT_HAND].R[1][0] = 0; robotModel[LEFT_HAND].R[1][1] = 1; robotModel[LEFT_HAND].R[1][2] = 0;
	robotModel[LEFT_HAND].R[2][0] = 0; robotModel[LEFT_HAND].R[2][1] = 0; robotModel[LEFT_HAND].R[2][2] = 1;
	robotModel[LEFT_HAND].v[0] = 0; robotModel[LEFT_HAND].v[1] = 0; robotModel[LEFT_HAND].v[2] = 0;
	robotModel[LEFT_HAND].w[0] = 0;	robotModel[LEFT_HAND].w[1] = 0; robotModel[LEFT_HAND].w[2] = 0;
	robotModel[LEFT_HAND].q = 0;
	robotModel[LEFT_HAND].dq = 0;
	robotModel[LEFT_HAND].ddq = 0;
	robotModel[LEFT_HAND].a[0] = 0; robotModel[LEFT_HAND].a[1] = 1; robotModel[LEFT_HAND].a[2] = 0;
	robotModel[LEFT_HAND].b[0] = 0;	robotModel[LEFT_HAND].b[1] = 0; robotModel[LEFT_HAND].b[2] = -0.14;
	robotModel[LEFT_HAND].m = 0;
	robotModel[LEFT_HAND].c[0] = 0;	robotModel[LEFT_HAND].c[1] = 0; robotModel[LEFT_HAND].c[2] = 0;
	robotModel[LEFT_HAND].I[0][0] = 0; robotModel[LEFT_HAND].I[0][1] = 0; robotModel[LEFT_HAND].I[0][2] = 0;
	robotModel[LEFT_HAND].I[1][0] = 0; robotModel[LEFT_HAND].I[1][1] = 0; robotModel[LEFT_HAND].I[1][2] = 0;
	robotModel[LEFT_HAND].I[2][0] = 0; robotModel[LEFT_HAND].I[2][1] = 0; robotModel[LEFT_HAND].I[2][2] = 0;

	// id 12
	robotModel[RIGHT_HIP_FRONT_SWING].linkID = RIGHT_HIP_FRONT_SWING;
	robotModel[RIGHT_HIP_FRONT_SWING].name = "右髋前摆";
	robotModel[RIGHT_HIP_FRONT_SWING].sister = LEFT_HIP_FRONT_SWING;
	robotModel[RIGHT_HIP_FRONT_SWING].child = RIGHT_HIP_SIDE_SWING;
	robotModel[RIGHT_HIP_FRONT_SWING].mother = MAIN_BODY;
	robotModel[RIGHT_HIP_FRONT_SWING].p[0] = 0; robotModel[RIGHT_HIP_FRONT_SWING].p[1] = 0; robotModel[RIGHT_HIP_FRONT_SWING].p[2] = 0;
	robotModel[RIGHT_HIP_FRONT_SWING].R[0][0] = 1; robotModel[RIGHT_HIP_FRONT_SWING].R[0][1] = 0; robotModel[RIGHT_HIP_FRONT_SWING].R[0][2] = 0;
	robotModel[RIGHT_HIP_FRONT_SWING].R[1][0] = 0; robotModel[RIGHT_HIP_FRONT_SWING].R[1][1] = 1; robotModel[RIGHT_HIP_FRONT_SWING].R[1][2] = 0;
	robotModel[RIGHT_HIP_FRONT_SWING].R[2][0] = 0; robotModel[RIGHT_HIP_FRONT_SWING].R[2][1] = 0; robotModel[RIGHT_HIP_FRONT_SWING].R[2][2] = 1;
	robotModel[RIGHT_HIP_FRONT_SWING].v[0] = 0; robotModel[RIGHT_HIP_FRONT_SWING].v[1] = 0; robotModel[RIGHT_HIP_FRONT_SWING].v[2] = 0;
	robotModel[RIGHT_HIP_FRONT_SWING].w[0] = 0;	robotModel[RIGHT_HIP_FRONT_SWING].w[1] = 0; robotModel[RIGHT_HIP_FRONT_SWING].w[2] = 0;
	robotModel[RIGHT_HIP_FRONT_SWING].q = 0;
	robotModel[RIGHT_HIP_FRONT_SWING].dq = 0;
	robotModel[RIGHT_HIP_FRONT_SWING].ddq = 0;
	robotModel[RIGHT_HIP_FRONT_SWING].a[0] = 0; robotModel[RIGHT_HIP_FRONT_SWING].a[1] = 1; robotModel[RIGHT_HIP_FRONT_SWING].a[2] = 0;
	robotModel[RIGHT_HIP_FRONT_SWING].b[0] = 0;	robotModel[RIGHT_HIP_FRONT_SWING].b[1] = -0.0528; robotModel[RIGHT_HIP_FRONT_SWING].b[2] = -0.05;
	robotModel[RIGHT_HIP_FRONT_SWING].m = 0.34981;
	robotModel[RIGHT_HIP_FRONT_SWING].c[0] = -0.01847;	robotModel[RIGHT_HIP_FRONT_SWING].c[1] = 0.00005; robotModel[RIGHT_HIP_FRONT_SWING].c[2] = -0.01966;
	robotModel[RIGHT_HIP_FRONT_SWING].I[0][0] = 0.000380; robotModel[RIGHT_HIP_FRONT_SWING].I[0][1] = 0; robotModel[RIGHT_HIP_FRONT_SWING].I[0][2] = 0;
	robotModel[RIGHT_HIP_FRONT_SWING].I[1][0] = 0; robotModel[RIGHT_HIP_FRONT_SWING].I[1][1] = 0.000656; robotModel[RIGHT_HIP_FRONT_SWING].I[1][2] = 0;
	robotModel[RIGHT_HIP_FRONT_SWING].I[2][0] = 0; robotModel[RIGHT_HIP_FRONT_SWING].I[2][1] = 0; robotModel[RIGHT_HIP_FRONT_SWING].I[2][2] = 0.000410;

	// id 13
	robotModel[RIGHT_HIP_SIDE_SWING].linkID = RIGHT_HIP_SIDE_SWING;
	robotModel[RIGHT_HIP_SIDE_SWING].name = "右髋侧摆";
	robotModel[RIGHT_HIP_SIDE_SWING].sister = NONE_JOINT;
	robotModel[RIGHT_HIP_SIDE_SWING].child = RIGHT_HIP_ROTATION;
	robotModel[RIGHT_HIP_SIDE_SWING].mother = RIGHT_HIP_FRONT_SWING;
	robotModel[RIGHT_HIP_SIDE_SWING].p[0] = 0; robotModel[RIGHT_HIP_SIDE_SWING].p[1] = 0; robotModel[RIGHT_HIP_SIDE_SWING].p[2] = 0;
	robotModel[RIGHT_HIP_SIDE_SWING].R[0][0] = 1; robotModel[RIGHT_HIP_SIDE_SWING].R[0][1] = 0; robotModel[RIGHT_HIP_SIDE_SWING].R[0][2] = 0;
	robotModel[RIGHT_HIP_SIDE_SWING].R[1][0] = 0; robotModel[RIGHT_HIP_SIDE_SWING].R[1][1] = 1; robotModel[RIGHT_HIP_SIDE_SWING].R[1][2] = 0;
	robotModel[RIGHT_HIP_SIDE_SWING].R[2][0] = 0; robotModel[RIGHT_HIP_SIDE_SWING].R[2][1] = 0; robotModel[RIGHT_HIP_SIDE_SWING].R[2][2] = 1;
	robotModel[RIGHT_HIP_SIDE_SWING].v[0] = 0; robotModel[RIGHT_HIP_SIDE_SWING].v[1] = 0; robotModel[RIGHT_HIP_SIDE_SWING].v[2] = 0;
	robotModel[RIGHT_HIP_SIDE_SWING].w[0] = 0;	robotModel[RIGHT_HIP_SIDE_SWING].w[1] = 0; robotModel[RIGHT_HIP_SIDE_SWING].w[2] = 0;
	robotModel[RIGHT_HIP_SIDE_SWING].q = 0;
	robotModel[RIGHT_HIP_SIDE_SWING].dq = 0;
	robotModel[RIGHT_HIP_SIDE_SWING].ddq = 0;
	robotModel[RIGHT_HIP_SIDE_SWING].a[0] = 1; robotModel[RIGHT_HIP_SIDE_SWING].a[1] = 0; robotModel[RIGHT_HIP_SIDE_SWING].a[2] = 0;
	robotModel[RIGHT_HIP_SIDE_SWING].b[0] = 0;	robotModel[RIGHT_HIP_SIDE_SWING].b[1] = 0; robotModel[RIGHT_HIP_SIDE_SWING].b[2] = 0;
	robotModel[RIGHT_HIP_SIDE_SWING].m = 0;
	robotModel[RIGHT_HIP_SIDE_SWING].c[0] = 0;	robotModel[RIGHT_HIP_SIDE_SWING].c[1] = 0; robotModel[RIGHT_HIP_SIDE_SWING].c[2] = 0;
	robotModel[RIGHT_HIP_SIDE_SWING].I[0][0] = 0; robotModel[RIGHT_HIP_SIDE_SWING].I[0][1] = 0; robotModel[RIGHT_HIP_SIDE_SWING].I[0][2] = 0;
	robotModel[RIGHT_HIP_SIDE_SWING].I[1][0] = 0; robotModel[RIGHT_HIP_SIDE_SWING].I[1][1] = 0; robotModel[RIGHT_HIP_SIDE_SWING].I[1][2] = 0;
	robotModel[RIGHT_HIP_SIDE_SWING].I[2][0] = 0; robotModel[RIGHT_HIP_SIDE_SWING].I[2][1] = 0; robotModel[RIGHT_HIP_SIDE_SWING].I[2][2] = 0;

	// id 14
	robotModel[RIGHT_HIP_ROTATION].linkID = RIGHT_HIP_ROTATION;
	robotModel[RIGHT_HIP_ROTATION].name = "右髋旋转";
	robotModel[RIGHT_HIP_ROTATION].sister = NONE_JOINT;
	robotModel[RIGHT_HIP_ROTATION].child = RIGHT_KNEE_FRONT_SWING;
	robotModel[RIGHT_HIP_ROTATION].mother = RIGHT_HIP_SIDE_SWING;
	robotModel[RIGHT_HIP_ROTATION].p[0] = 0; robotModel[RIGHT_HIP_ROTATION].p[1] = 0; robotModel[RIGHT_HIP_ROTATION].p[2] = 0;
	robotModel[RIGHT_HIP_ROTATION].R[0][0] = 1; robotModel[RIGHT_HIP_ROTATION].R[0][1] = 0; robotModel[RIGHT_HIP_ROTATION].R[0][2] = 0;
	robotModel[RIGHT_HIP_ROTATION].R[1][0] = 0; robotModel[RIGHT_HIP_ROTATION].R[1][1] = 1; robotModel[RIGHT_HIP_ROTATION].R[1][2] = 0;
	robotModel[RIGHT_HIP_ROTATION].R[2][0] = 0; robotModel[RIGHT_HIP_ROTATION].R[2][1] = 0; robotModel[RIGHT_HIP_ROTATION].R[2][2] = 1;
	robotModel[RIGHT_HIP_ROTATION].v[0] = 0; robotModel[RIGHT_HIP_ROTATION].v[1] = 0; robotModel[RIGHT_HIP_ROTATION].v[2] = 0;
	robotModel[RIGHT_HIP_ROTATION].w[0] = 0;	robotModel[RIGHT_HIP_ROTATION].w[1] = 0; robotModel[RIGHT_HIP_ROTATION].w[2] = 0;
	robotModel[RIGHT_HIP_ROTATION].q = 0;
	robotModel[RIGHT_HIP_ROTATION].dq = 0;
	robotModel[RIGHT_HIP_ROTATION].ddq = 0;
	robotModel[RIGHT_HIP_ROTATION].a[0] = 0; robotModel[RIGHT_HIP_ROTATION].a[1] = 0; robotModel[RIGHT_HIP_ROTATION].a[2] = 1;
	robotModel[RIGHT_HIP_ROTATION].b[0] = 0;	robotModel[RIGHT_HIP_ROTATION].b[1] = 0; robotModel[RIGHT_HIP_ROTATION].b[2] = -0.1092;
	robotModel[RIGHT_HIP_ROTATION].m = 0.18036;
	robotModel[RIGHT_HIP_ROTATION].c[0] = 0.01481;	robotModel[RIGHT_HIP_ROTATION].c[1] = 0; robotModel[RIGHT_HIP_ROTATION].c[2] = -0.00308;
	robotModel[RIGHT_HIP_ROTATION].I[0][0] = 0.000066; robotModel[RIGHT_HIP_ROTATION].I[0][1] = 0; robotModel[RIGHT_HIP_ROTATION].I[0][2] = 0;
	robotModel[RIGHT_HIP_ROTATION].I[1][0] = 0; robotModel[RIGHT_HIP_ROTATION].I[1][1] = 0.000168; robotModel[RIGHT_HIP_ROTATION].I[1][2] = 0;
	robotModel[RIGHT_HIP_ROTATION].I[2][0] = 0; robotModel[RIGHT_HIP_ROTATION].I[2][1] = 0; robotModel[RIGHT_HIP_ROTATION].I[2][2] = 0.000138;

	// id 15
	robotModel[RIGHT_KNEE_FRONT_SWING].linkID = RIGHT_KNEE_FRONT_SWING;
	robotModel[RIGHT_KNEE_FRONT_SWING].name = "右膝前摆";
	robotModel[RIGHT_KNEE_FRONT_SWING].sister = NONE_JOINT;
	robotModel[RIGHT_KNEE_FRONT_SWING].child = RIGHT_ANKLE_FRONT_SWING;
	robotModel[RIGHT_KNEE_FRONT_SWING].mother = RIGHT_HIP_ROTATION;
	robotModel[RIGHT_KNEE_FRONT_SWING].p[0] = 0; robotModel[RIGHT_KNEE_FRONT_SWING].p[1] = 0; robotModel[RIGHT_KNEE_FRONT_SWING].p[2] = 0;
	robotModel[RIGHT_KNEE_FRONT_SWING].R[0][0] = 1; robotModel[RIGHT_KNEE_FRONT_SWING].R[0][1] = 0; robotModel[RIGHT_KNEE_FRONT_SWING].R[0][2] = 0;
	robotModel[RIGHT_KNEE_FRONT_SWING].R[1][0] = 0; robotModel[RIGHT_KNEE_FRONT_SWING].R[1][1] = 1; robotModel[RIGHT_KNEE_FRONT_SWING].R[1][2] = 0;
	robotModel[RIGHT_KNEE_FRONT_SWING].R[2][0] = 0; robotModel[RIGHT_KNEE_FRONT_SWING].R[2][1] = 0; robotModel[RIGHT_KNEE_FRONT_SWING].R[2][2] = 1;
	robotModel[RIGHT_KNEE_FRONT_SWING].v[0] = 0; robotModel[RIGHT_KNEE_FRONT_SWING].v[1] = 0; robotModel[RIGHT_KNEE_FRONT_SWING].v[2] = 0;
	robotModel[RIGHT_KNEE_FRONT_SWING].w[0] = 0;	robotModel[RIGHT_KNEE_FRONT_SWING].w[1] = 0; robotModel[RIGHT_KNEE_FRONT_SWING].w[2] = 0;
	robotModel[RIGHT_KNEE_FRONT_SWING].q = 0;
	robotModel[RIGHT_KNEE_FRONT_SWING].dq = 0;
	robotModel[RIGHT_KNEE_FRONT_SWING].ddq = 0;
	robotModel[RIGHT_KNEE_FRONT_SWING].a[0] = 0; robotModel[RIGHT_KNEE_FRONT_SWING].a[1] = 1; robotModel[RIGHT_KNEE_FRONT_SWING].a[2] = 0;
	robotModel[RIGHT_KNEE_FRONT_SWING].b[0] = 0;	robotModel[RIGHT_KNEE_FRONT_SWING].b[1] = 0; robotModel[RIGHT_KNEE_FRONT_SWING].b[2] = -0.0467;
	robotModel[RIGHT_KNEE_FRONT_SWING].m = 0.18130;
	robotModel[RIGHT_KNEE_FRONT_SWING].c[0] = 0.01057;	robotModel[RIGHT_KNEE_FRONT_SWING].c[1] = -0.00007; robotModel[RIGHT_KNEE_FRONT_SWING].c[2] = -0.01601;
	robotModel[RIGHT_KNEE_FRONT_SWING].I[0][0] = 0.000307; robotModel[RIGHT_KNEE_FRONT_SWING].I[0][1] = 0; robotModel[RIGHT_KNEE_FRONT_SWING].I[0][2] = 0;
	robotModel[RIGHT_KNEE_FRONT_SWING].I[1][0] = 0; robotModel[RIGHT_KNEE_FRONT_SWING].I[1][1] = 0.000322; robotModel[RIGHT_KNEE_FRONT_SWING].I[1][2] = 0;
	robotModel[RIGHT_KNEE_FRONT_SWING].I[2][0] = 0; robotModel[RIGHT_KNEE_FRONT_SWING].I[2][1] = 0; robotModel[RIGHT_KNEE_FRONT_SWING].I[2][2] = 0.000122;

	// id 16
	robotModel[RIGHT_ANKLE_FRONT_SWING].linkID = RIGHT_ANKLE_FRONT_SWING;
	robotModel[RIGHT_ANKLE_FRONT_SWING].name = "右踝前摆";
	robotModel[RIGHT_ANKLE_FRONT_SWING].sister = NONE_JOINT;
	robotModel[RIGHT_ANKLE_FRONT_SWING].child = RIGHT_ANKLE_SIDE_SWING;
	robotModel[RIGHT_ANKLE_FRONT_SWING].mother = RIGHT_KNEE_FRONT_SWING;
	robotModel[RIGHT_ANKLE_FRONT_SWING].p[0] = 0; robotModel[RIGHT_ANKLE_FRONT_SWING].p[1] = 0; robotModel[RIGHT_ANKLE_FRONT_SWING].p[2] = 0;
	robotModel[RIGHT_ANKLE_FRONT_SWING].R[0][0] = 1; robotModel[RIGHT_ANKLE_FRONT_SWING].R[0][1] = 0; robotModel[RIGHT_ANKLE_FRONT_SWING].R[0][2] = 0;
	robotModel[RIGHT_ANKLE_FRONT_SWING].R[1][0] = 0; robotModel[RIGHT_ANKLE_FRONT_SWING].R[1][1] = 1; robotModel[RIGHT_ANKLE_FRONT_SWING].R[1][2] = 0;
	robotModel[RIGHT_ANKLE_FRONT_SWING].R[2][0] = 0; robotModel[RIGHT_ANKLE_FRONT_SWING].R[2][1] = 0; robotModel[RIGHT_ANKLE_FRONT_SWING].R[2][2] = 1;
	robotModel[RIGHT_ANKLE_FRONT_SWING].v[0] = 0; robotModel[RIGHT_ANKLE_FRONT_SWING].v[1] = 0; robotModel[RIGHT_ANKLE_FRONT_SWING].v[2] = 0;
	robotModel[RIGHT_ANKLE_FRONT_SWING].w[0] = 0;	robotModel[RIGHT_ANKLE_FRONT_SWING].w[1] = 0; robotModel[RIGHT_ANKLE_FRONT_SWING].w[2] = 0;
	robotModel[RIGHT_ANKLE_FRONT_SWING].q = 0;
	robotModel[RIGHT_ANKLE_FRONT_SWING].dq = 0;
	robotModel[RIGHT_ANKLE_FRONT_SWING].ddq = 0;
	robotModel[RIGHT_ANKLE_FRONT_SWING].a[0] = 0; robotModel[RIGHT_ANKLE_FRONT_SWING].a[1] = 1; robotModel[RIGHT_ANKLE_FRONT_SWING].a[2] = 0;
	robotModel[RIGHT_ANKLE_FRONT_SWING].b[0] = 0;	robotModel[RIGHT_ANKLE_FRONT_SWING].b[1] = 0; robotModel[RIGHT_ANKLE_FRONT_SWING].b[2] = -0.12;
	robotModel[RIGHT_ANKLE_FRONT_SWING].m = 0.39215;
	robotModel[RIGHT_ANKLE_FRONT_SWING].c[0] = -0.02239;	robotModel[RIGHT_ANKLE_FRONT_SWING].c[1] = -0.00076; robotModel[RIGHT_ANKLE_FRONT_SWING].c[2] = 0.00860;
	robotModel[RIGHT_ANKLE_FRONT_SWING].I[0][0] = 0.000382; robotModel[RIGHT_ANKLE_FRONT_SWING].I[0][1] = 0; robotModel[RIGHT_ANKLE_FRONT_SWING].I[0][2] = 0;
	robotModel[RIGHT_ANKLE_FRONT_SWING].I[1][0] = 0; robotModel[RIGHT_ANKLE_FRONT_SWING].I[1][1] = 0.000890; robotModel[RIGHT_ANKLE_FRONT_SWING].I[1][2] = 0;
	robotModel[RIGHT_ANKLE_FRONT_SWING].I[2][0] = 0; robotModel[RIGHT_ANKLE_FRONT_SWING].I[2][1] = 0; robotModel[RIGHT_ANKLE_FRONT_SWING].I[2][2] = 0.000634;

	// id 17
	robotModel[RIGHT_ANKLE_SIDE_SWING].linkID = RIGHT_ANKLE_SIDE_SWING;
	robotModel[RIGHT_ANKLE_SIDE_SWING].name = "右踝侧摆";
	robotModel[RIGHT_ANKLE_SIDE_SWING].sister = NONE_JOINT;
	robotModel[RIGHT_ANKLE_SIDE_SWING].child = RIGHT_FOOT;
	robotModel[RIGHT_ANKLE_SIDE_SWING].mother = RIGHT_ANKLE_FRONT_SWING;
	robotModel[RIGHT_ANKLE_SIDE_SWING].p[0] = 0; robotModel[RIGHT_ANKLE_SIDE_SWING].p[1] = 0; robotModel[RIGHT_ANKLE_SIDE_SWING].p[2] = 0;
	robotModel[RIGHT_ANKLE_SIDE_SWING].R[0][0] = 1; robotModel[RIGHT_ANKLE_SIDE_SWING].R[0][1] = 0; robotModel[RIGHT_ANKLE_SIDE_SWING].R[0][2] = 0;
	robotModel[RIGHT_ANKLE_SIDE_SWING].R[1][0] = 0; robotModel[RIGHT_ANKLE_SIDE_SWING].R[1][1] = 1; robotModel[RIGHT_ANKLE_SIDE_SWING].R[1][2] = 0;
	robotModel[RIGHT_ANKLE_SIDE_SWING].R[2][0] = 0; robotModel[RIGHT_ANKLE_SIDE_SWING].R[2][1] = 0; robotModel[RIGHT_ANKLE_SIDE_SWING].R[2][2] = 1;
	robotModel[RIGHT_ANKLE_SIDE_SWING].v[0] = 0; robotModel[RIGHT_ANKLE_SIDE_SWING].v[1] = 0; robotModel[RIGHT_ANKLE_SIDE_SWING].v[2] = 0;
	robotModel[RIGHT_ANKLE_SIDE_SWING].w[0] = 0;	robotModel[RIGHT_ANKLE_SIDE_SWING].w[1] = 0; robotModel[RIGHT_ANKLE_SIDE_SWING].w[2] = 0;
	robotModel[RIGHT_ANKLE_SIDE_SWING].q = 0;
	robotModel[RIGHT_ANKLE_SIDE_SWING].dq = 0;
	robotModel[RIGHT_ANKLE_SIDE_SWING].ddq = 0;
	robotModel[RIGHT_ANKLE_SIDE_SWING].a[0] = 1; robotModel[RIGHT_ANKLE_SIDE_SWING].a[1] = 0; robotModel[RIGHT_ANKLE_SIDE_SWING].a[2] = 0;
	robotModel[RIGHT_ANKLE_SIDE_SWING].b[0] = 0;	robotModel[RIGHT_ANKLE_SIDE_SWING].b[1] = 0; robotModel[RIGHT_ANKLE_SIDE_SWING].b[2] = 0;
	robotModel[RIGHT_ANKLE_SIDE_SWING].m = 0;
	robotModel[RIGHT_ANKLE_SIDE_SWING].c[0] = 0;	robotModel[RIGHT_ANKLE_SIDE_SWING].c[1] = 0; robotModel[RIGHT_ANKLE_SIDE_SWING].c[2] = 0;
	robotModel[RIGHT_ANKLE_SIDE_SWING].I[0][0] = 0; robotModel[RIGHT_ANKLE_SIDE_SWING].I[0][1] = 0; robotModel[RIGHT_ANKLE_SIDE_SWING].I[0][2] = 0;
	robotModel[RIGHT_ANKLE_SIDE_SWING].I[1][0] = 0; robotModel[RIGHT_ANKLE_SIDE_SWING].I[1][1] = 0; robotModel[RIGHT_ANKLE_SIDE_SWING].I[1][2] = 0;
	robotModel[RIGHT_ANKLE_SIDE_SWING].I[2][0] = 0; robotModel[RIGHT_ANKLE_SIDE_SWING].I[2][1] = 0; robotModel[RIGHT_ANKLE_SIDE_SWING].I[2][2] = 0;

	// id 18
	robotModel[RIGHT_FOOT].linkID = RIGHT_FOOT;
	robotModel[RIGHT_FOOT].name = "右脚端";
	robotModel[RIGHT_FOOT].sister = NONE_JOINT;
	robotModel[RIGHT_FOOT].child = NONE_JOINT;
	robotModel[RIGHT_FOOT].mother = RIGHT_ANKLE_SIDE_SWING;
	robotModel[RIGHT_FOOT].p[0] = 0; robotModel[RIGHT_FOOT].p[1] = 0; robotModel[RIGHT_FOOT].p[2] = 0;
	robotModel[RIGHT_FOOT].R[0][0] = 1; robotModel[RIGHT_FOOT].R[0][1] = 0; robotModel[RIGHT_FOOT].R[0][2] = 0;
	robotModel[RIGHT_FOOT].R[1][0] = 0; robotModel[RIGHT_FOOT].R[1][1] = 1; robotModel[RIGHT_FOOT].R[1][2] = 0;
	robotModel[RIGHT_FOOT].R[2][0] = 0; robotModel[RIGHT_FOOT].R[2][1] = 0; robotModel[RIGHT_FOOT].R[2][2] = 1;
	robotModel[RIGHT_FOOT].v[0] = 0; robotModel[RIGHT_FOOT].v[1] = 0; robotModel[RIGHT_FOOT].v[2] = 0;
	robotModel[RIGHT_FOOT].w[0] = 0;	robotModel[RIGHT_FOOT].w[1] = 0; robotModel[RIGHT_FOOT].w[2] = 0;
	robotModel[RIGHT_FOOT].q = 0;
	robotModel[RIGHT_FOOT].dq = 0;
	robotModel[RIGHT_FOOT].ddq = 0;
	robotModel[RIGHT_FOOT].a[0] = 0; robotModel[RIGHT_FOOT].a[1] = 1; robotModel[RIGHT_FOOT].a[2] = 0;
	robotModel[RIGHT_FOOT].b[0] = 0; robotModel[RIGHT_FOOT].b[1] = 0; robotModel[RIGHT_FOOT].b[2] = -0.0455;
	robotModel[RIGHT_FOOT].m = 0;
	robotModel[RIGHT_FOOT].c[0] = 0;	robotModel[RIGHT_FOOT].c[1] = 0; robotModel[RIGHT_FOOT].c[2] = 0;
	robotModel[RIGHT_FOOT].I[0][0] = 0; robotModel[RIGHT_FOOT].I[0][1] = 0; robotModel[RIGHT_FOOT].I[0][2] = 0;
	robotModel[RIGHT_FOOT].I[1][0] = 0; robotModel[RIGHT_FOOT].I[1][1] = 0; robotModel[RIGHT_FOOT].I[1][2] = 0;
	robotModel[RIGHT_FOOT].I[2][0] = 0; robotModel[RIGHT_FOOT].I[2][1] = 0; robotModel[RIGHT_FOOT].I[2][2] = 0;

	// id 19
	robotModel[LEFT_HIP_FRONT_SWING].linkID = LEFT_HIP_FRONT_SWING;
	robotModel[LEFT_HIP_FRONT_SWING].name = "左髋前摆";
	robotModel[LEFT_HIP_FRONT_SWING].sister = NONE_JOINT;
	robotModel[LEFT_HIP_FRONT_SWING].child = LEFT_HIP_SIDE_SWING;
	robotModel[LEFT_HIP_FRONT_SWING].mother = MAIN_BODY;
	robotModel[LEFT_HIP_FRONT_SWING].p[0] = 0; robotModel[LEFT_HIP_FRONT_SWING].p[1] = 0; robotModel[LEFT_HIP_FRONT_SWING].p[2] = 0;
	robotModel[LEFT_HIP_FRONT_SWING].R[0][0] = 1; robotModel[LEFT_HIP_FRONT_SWING].R[0][1] = 0; robotModel[LEFT_HIP_FRONT_SWING].R[0][2] = 0;
	robotModel[LEFT_HIP_FRONT_SWING].R[1][0] = 0; robotModel[LEFT_HIP_FRONT_SWING].R[1][1] = 1; robotModel[LEFT_HIP_FRONT_SWING].R[1][2] = 0;
	robotModel[LEFT_HIP_FRONT_SWING].R[2][0] = 0; robotModel[LEFT_HIP_FRONT_SWING].R[2][1] = 0; robotModel[LEFT_HIP_FRONT_SWING].R[2][2] = 1;
	robotModel[LEFT_HIP_FRONT_SWING].v[0] = 0; robotModel[LEFT_HIP_FRONT_SWING].v[1] = 0; robotModel[LEFT_HIP_FRONT_SWING].v[2] = 0;
	robotModel[LEFT_HIP_FRONT_SWING].w[0] = 0;	robotModel[LEFT_HIP_FRONT_SWING].w[1] = 0; robotModel[LEFT_HIP_FRONT_SWING].w[2] = 0;
	robotModel[LEFT_HIP_FRONT_SWING].q = 0;
	robotModel[LEFT_HIP_FRONT_SWING].dq = 0;
	robotModel[LEFT_HIP_FRONT_SWING].ddq = 0;
	robotModel[LEFT_HIP_FRONT_SWING].a[0] = 0; robotModel[LEFT_HIP_FRONT_SWING].a[1] = 1; robotModel[LEFT_HIP_FRONT_SWING].a[2] = 0;
	robotModel[LEFT_HIP_FRONT_SWING].b[0] = 0;	robotModel[LEFT_HIP_FRONT_SWING].b[1] = 0.0528; robotModel[LEFT_HIP_FRONT_SWING].b[2] = -0.05;
	robotModel[LEFT_HIP_FRONT_SWING].m = 0.34981;
	robotModel[LEFT_HIP_FRONT_SWING].c[0] = -0.01846;	robotModel[LEFT_HIP_FRONT_SWING].c[1] = 0.00008; robotModel[LEFT_HIP_FRONT_SWING].c[2] = -0.01959;
	robotModel[LEFT_HIP_FRONT_SWING].I[0][0] = 0.000379; robotModel[LEFT_HIP_FRONT_SWING].I[0][1] = 0; robotModel[LEFT_HIP_FRONT_SWING].I[0][2] = 0;
	robotModel[LEFT_HIP_FRONT_SWING].I[1][0] = 0; robotModel[LEFT_HIP_FRONT_SWING].I[1][1] = 0.000655; robotModel[LEFT_HIP_FRONT_SWING].I[1][2] = 0;
	robotModel[LEFT_HIP_FRONT_SWING].I[2][0] = 0; robotModel[LEFT_HIP_FRONT_SWING].I[2][1] = 0; robotModel[LEFT_HIP_FRONT_SWING].I[2][2] = 0.000410;

	// id 20
	robotModel[LEFT_HIP_SIDE_SWING].linkID = LEFT_HIP_SIDE_SWING;
	robotModel[LEFT_HIP_SIDE_SWING].name = "左髋侧摆";
	robotModel[LEFT_HIP_SIDE_SWING].sister = NONE_JOINT;
	robotModel[LEFT_HIP_SIDE_SWING].child = LEFT_HIP_ROTATION;
	robotModel[LEFT_HIP_SIDE_SWING].mother = LEFT_HIP_FRONT_SWING;
	robotModel[LEFT_HIP_SIDE_SWING].p[0] = 0; robotModel[LEFT_HIP_SIDE_SWING].p[1] = 0; robotModel[LEFT_HIP_SIDE_SWING].p[2] = 0;
	robotModel[LEFT_HIP_SIDE_SWING].R[0][0] = 1; robotModel[LEFT_HIP_SIDE_SWING].R[0][1] = 0; robotModel[LEFT_HIP_SIDE_SWING].R[0][2] = 0;
	robotModel[LEFT_HIP_SIDE_SWING].R[1][0] = 0; robotModel[LEFT_HIP_SIDE_SWING].R[1][1] = 1; robotModel[LEFT_HIP_SIDE_SWING].R[1][2] = 0;
	robotModel[LEFT_HIP_SIDE_SWING].R[2][0] = 0; robotModel[LEFT_HIP_SIDE_SWING].R[2][1] = 0; robotModel[LEFT_HIP_SIDE_SWING].R[2][2] = 1;
	robotModel[LEFT_HIP_SIDE_SWING].v[0] = 0; robotModel[LEFT_HIP_SIDE_SWING].v[1] = 0; robotModel[LEFT_HIP_SIDE_SWING].v[2] = 0;
	robotModel[LEFT_HIP_SIDE_SWING].w[0] = 0;	robotModel[LEFT_HIP_SIDE_SWING].w[1] = 0; robotModel[LEFT_HIP_SIDE_SWING].w[2] = 0;
	robotModel[LEFT_HIP_SIDE_SWING].q = 0;
	robotModel[LEFT_HIP_SIDE_SWING].dq = 0;
	robotModel[LEFT_HIP_SIDE_SWING].ddq = 0;
	robotModel[LEFT_HIP_SIDE_SWING].a[0] = 1; robotModel[LEFT_HIP_SIDE_SWING].a[1] = 0; robotModel[LEFT_HIP_SIDE_SWING].a[2] = 0;
	robotModel[LEFT_HIP_SIDE_SWING].b[0] = 0;	robotModel[LEFT_HIP_SIDE_SWING].b[1] = 0; robotModel[LEFT_HIP_SIDE_SWING].b[2] = 0;
	robotModel[LEFT_HIP_SIDE_SWING].m = 0;
	robotModel[LEFT_HIP_SIDE_SWING].c[0] = 0;	robotModel[LEFT_HIP_SIDE_SWING].c[1] = 0; robotModel[LEFT_HIP_SIDE_SWING].c[2] = 0;
	robotModel[LEFT_HIP_SIDE_SWING].I[0][0] = 0; robotModel[LEFT_HIP_SIDE_SWING].I[0][1] = 0; robotModel[LEFT_HIP_SIDE_SWING].I[0][2] = 0;
	robotModel[LEFT_HIP_SIDE_SWING].I[1][0] = 0; robotModel[LEFT_HIP_SIDE_SWING].I[1][1] = 0; robotModel[LEFT_HIP_SIDE_SWING].I[1][2] = 0;
	robotModel[LEFT_HIP_SIDE_SWING].I[2][0] = 0; robotModel[LEFT_HIP_SIDE_SWING].I[2][1] = 0; robotModel[LEFT_HIP_SIDE_SWING].I[2][2] = 0;

	// id 21
	robotModel[LEFT_HIP_ROTATION].linkID = LEFT_HIP_ROTATION;
	robotModel[LEFT_HIP_ROTATION].name = "左髋旋转";
	robotModel[LEFT_HIP_ROTATION].sister = NONE_JOINT;
	robotModel[LEFT_HIP_ROTATION].child = LEFT_KNEE_FRONT_SWING;
	robotModel[LEFT_HIP_ROTATION].mother = LEFT_HIP_SIDE_SWING;
	robotModel[LEFT_HIP_ROTATION].p[0] = 0; robotModel[LEFT_HIP_ROTATION].p[1] = 0; robotModel[LEFT_HIP_ROTATION].p[2] = 0;
	robotModel[LEFT_HIP_ROTATION].R[0][0] = 1; robotModel[LEFT_HIP_ROTATION].R[0][1] = 0; robotModel[LEFT_HIP_ROTATION].R[0][2] = 0;
	robotModel[LEFT_HIP_ROTATION].R[1][0] = 0; robotModel[LEFT_HIP_ROTATION].R[1][1] = 1; robotModel[LEFT_HIP_ROTATION].R[1][2] = 0;
	robotModel[LEFT_HIP_ROTATION].R[2][0] = 0; robotModel[LEFT_HIP_ROTATION].R[2][1] = 0; robotModel[LEFT_HIP_ROTATION].R[2][2] = 1;
	robotModel[LEFT_HIP_ROTATION].v[0] = 0; robotModel[LEFT_HIP_ROTATION].v[1] = 0; robotModel[LEFT_HIP_ROTATION].v[2] = 0;
	robotModel[LEFT_HIP_ROTATION].w[0] = 0;	robotModel[LEFT_HIP_ROTATION].w[1] = 0; robotModel[LEFT_HIP_ROTATION].w[2] = 0;
	robotModel[LEFT_HIP_ROTATION].q = 0;
	robotModel[LEFT_HIP_ROTATION].dq = 0;
	robotModel[LEFT_HIP_ROTATION].ddq = 0;
	robotModel[LEFT_HIP_ROTATION].a[0] = 0; robotModel[LEFT_HIP_ROTATION].a[1] = 0; robotModel[LEFT_HIP_ROTATION].a[2] = 1;
	robotModel[LEFT_HIP_ROTATION].b[0] = 0;	robotModel[LEFT_HIP_ROTATION].b[1] = 0; robotModel[LEFT_HIP_ROTATION].b[2] = -0.1092;
	robotModel[LEFT_HIP_ROTATION].m = 0.18036;
	robotModel[LEFT_HIP_ROTATION].c[0] = 0.01481;	robotModel[LEFT_HIP_ROTATION].c[1] = 0; robotModel[LEFT_HIP_ROTATION].c[2] = -0.00308;
	robotModel[LEFT_HIP_ROTATION].I[0][0] = 0.000066; robotModel[LEFT_HIP_ROTATION].I[0][1] = 0; robotModel[LEFT_HIP_ROTATION].I[0][2] = 0;
	robotModel[LEFT_HIP_ROTATION].I[1][0] = 0; robotModel[LEFT_HIP_ROTATION].I[1][1] = 0.000168; robotModel[LEFT_HIP_ROTATION].I[1][2] = 0;
	robotModel[LEFT_HIP_ROTATION].I[2][0] = 0; robotModel[LEFT_HIP_ROTATION].I[2][1] = 0; robotModel[LEFT_HIP_ROTATION].I[2][2] = 0.000138;

	// id 22
	robotModel[LEFT_KNEE_FRONT_SWING].linkID = LEFT_KNEE_FRONT_SWING;
	robotModel[LEFT_KNEE_FRONT_SWING].name = "左膝前摆";
	robotModel[LEFT_KNEE_FRONT_SWING].sister = NONE_JOINT;
	robotModel[LEFT_KNEE_FRONT_SWING].child = LEFT_ANKLE_FRONT_SWING;
	robotModel[LEFT_KNEE_FRONT_SWING].mother = LEFT_HIP_ROTATION;
	robotModel[LEFT_KNEE_FRONT_SWING].p[0] = 0; robotModel[LEFT_KNEE_FRONT_SWING].p[1] = 0; robotModel[LEFT_KNEE_FRONT_SWING].p[2] = 0;
	robotModel[LEFT_KNEE_FRONT_SWING].R[0][0] = 1; robotModel[LEFT_KNEE_FRONT_SWING].R[0][1] = 0; robotModel[LEFT_KNEE_FRONT_SWING].R[0][2] = 0;
	robotModel[LEFT_KNEE_FRONT_SWING].R[1][0] = 0; robotModel[LEFT_KNEE_FRONT_SWING].R[1][1] = 1; robotModel[LEFT_KNEE_FRONT_SWING].R[1][2] = 0;
	robotModel[LEFT_KNEE_FRONT_SWING].R[2][0] = 0; robotModel[LEFT_KNEE_FRONT_SWING].R[2][1] = 0; robotModel[LEFT_KNEE_FRONT_SWING].R[2][2] = 1;
	robotModel[LEFT_KNEE_FRONT_SWING].v[0] = 0; robotModel[LEFT_KNEE_FRONT_SWING].v[1] = 0; robotModel[LEFT_KNEE_FRONT_SWING].v[2] = 0;
	robotModel[LEFT_KNEE_FRONT_SWING].w[0] = 0;	robotModel[LEFT_KNEE_FRONT_SWING].w[1] = 0; robotModel[LEFT_KNEE_FRONT_SWING].w[2] = 0;
	robotModel[LEFT_KNEE_FRONT_SWING].q = 0;
	robotModel[LEFT_KNEE_FRONT_SWING].dq = 0;
	robotModel[LEFT_KNEE_FRONT_SWING].ddq = 0;
	robotModel[LEFT_KNEE_FRONT_SWING].a[0] = 0; robotModel[LEFT_KNEE_FRONT_SWING].a[1] = 1; robotModel[LEFT_KNEE_FRONT_SWING].a[2] = 0;
	robotModel[LEFT_KNEE_FRONT_SWING].b[0] = 0;	robotModel[LEFT_KNEE_FRONT_SWING].b[1] = 0; robotModel[LEFT_KNEE_FRONT_SWING].b[2] = -0.0467;
	robotModel[LEFT_KNEE_FRONT_SWING].m = 0.18130;
	robotModel[LEFT_KNEE_FRONT_SWING].c[0] = 0.01057;	robotModel[LEFT_KNEE_FRONT_SWING].c[1] = 0.00007; robotModel[LEFT_KNEE_FRONT_SWING].c[2] = -0.01601;
	robotModel[LEFT_KNEE_FRONT_SWING].I[0][0] = 0.000307; robotModel[LEFT_KNEE_FRONT_SWING].I[0][1] = 0; robotModel[LEFT_KNEE_FRONT_SWING].I[0][2] = 0;
	robotModel[LEFT_KNEE_FRONT_SWING].I[1][0] = 0; robotModel[LEFT_KNEE_FRONT_SWING].I[1][1] = 0.000322; robotModel[LEFT_KNEE_FRONT_SWING].I[1][2] = 0;
	robotModel[LEFT_KNEE_FRONT_SWING].I[2][0] = 0; robotModel[LEFT_KNEE_FRONT_SWING].I[2][1] = 0; robotModel[LEFT_KNEE_FRONT_SWING].I[2][2] = 0.000122;

	// id 23
	robotModel[LEFT_ANKLE_FRONT_SWING].linkID = LEFT_ANKLE_FRONT_SWING;
	robotModel[LEFT_ANKLE_FRONT_SWING].name = "左踝前摆";
	robotModel[LEFT_ANKLE_FRONT_SWING].sister = NONE_JOINT;
	robotModel[LEFT_ANKLE_FRONT_SWING].child = LEFT_ANKLE_SIDE_SWING;
	robotModel[LEFT_ANKLE_FRONT_SWING].mother = LEFT_KNEE_FRONT_SWING;
	robotModel[LEFT_ANKLE_FRONT_SWING].p[0] = 0; robotModel[LEFT_ANKLE_FRONT_SWING].p[1] = 0; robotModel[LEFT_ANKLE_FRONT_SWING].p[2] = 0;
	robotModel[LEFT_ANKLE_FRONT_SWING].R[0][0] = 1; robotModel[LEFT_ANKLE_FRONT_SWING].R[0][1] = 0; robotModel[LEFT_ANKLE_FRONT_SWING].R[0][2] = 0;
	robotModel[LEFT_ANKLE_FRONT_SWING].R[1][0] = 0; robotModel[LEFT_ANKLE_FRONT_SWING].R[1][1] = 1; robotModel[LEFT_ANKLE_FRONT_SWING].R[1][2] = 0;
	robotModel[LEFT_ANKLE_FRONT_SWING].R[2][0] = 0; robotModel[LEFT_ANKLE_FRONT_SWING].R[2][1] = 0; robotModel[LEFT_ANKLE_FRONT_SWING].R[2][2] = 1;
	robotModel[LEFT_ANKLE_FRONT_SWING].v[0] = 0; robotModel[LEFT_ANKLE_FRONT_SWING].v[1] = 0; robotModel[LEFT_ANKLE_FRONT_SWING].v[2] = 0;
	robotModel[LEFT_ANKLE_FRONT_SWING].w[0] = 0;	robotModel[LEFT_ANKLE_FRONT_SWING].w[1] = 0; robotModel[LEFT_ANKLE_FRONT_SWING].w[2] = 0;
	robotModel[LEFT_ANKLE_FRONT_SWING].q = 0;
	robotModel[LEFT_ANKLE_FRONT_SWING].dq = 0;
	robotModel[LEFT_ANKLE_FRONT_SWING].ddq = 0;
	robotModel[LEFT_ANKLE_FRONT_SWING].a[0] = 0; robotModel[LEFT_ANKLE_FRONT_SWING].a[1] = 1; robotModel[LEFT_ANKLE_FRONT_SWING].a[2] = 0;
	robotModel[LEFT_ANKLE_FRONT_SWING].b[0] = 0;	robotModel[LEFT_ANKLE_FRONT_SWING].b[1] = 0; robotModel[LEFT_ANKLE_FRONT_SWING].b[2] = -0.12;
	robotModel[LEFT_ANKLE_FRONT_SWING].m = 0.39215;
	robotModel[LEFT_ANKLE_FRONT_SWING].c[0] = -0.02239;	robotModel[LEFT_ANKLE_FRONT_SWING].c[1] = 0.00076; robotModel[LEFT_ANKLE_FRONT_SWING].c[2] = 0.00860;
	robotModel[LEFT_ANKLE_FRONT_SWING].I[0][0] = 0.000382; robotModel[LEFT_ANKLE_FRONT_SWING].I[0][1] = 0; robotModel[LEFT_ANKLE_FRONT_SWING].I[0][2] = 0;
	robotModel[LEFT_ANKLE_FRONT_SWING].I[1][0] = 0; robotModel[LEFT_ANKLE_FRONT_SWING].I[1][1] = 0.000891; robotModel[LEFT_ANKLE_FRONT_SWING].I[1][2] = 0;
	robotModel[LEFT_ANKLE_FRONT_SWING].I[2][0] = 0; robotModel[LEFT_ANKLE_FRONT_SWING].I[2][1] = 0; robotModel[LEFT_ANKLE_FRONT_SWING].I[2][2] = 0.000634;

	// id 24
	robotModel[LEFT_ANKLE_SIDE_SWING].linkID = LEFT_ANKLE_SIDE_SWING;
	robotModel[LEFT_ANKLE_SIDE_SWING].name = "左踝侧摆";
	robotModel[LEFT_ANKLE_SIDE_SWING].sister = NONE_JOINT;
	robotModel[LEFT_ANKLE_SIDE_SWING].child = LEFT_FOOT;
	robotModel[LEFT_ANKLE_SIDE_SWING].mother = LEFT_ANKLE_FRONT_SWING;
	robotModel[LEFT_ANKLE_SIDE_SWING].p[0] = 0; robotModel[LEFT_ANKLE_SIDE_SWING].p[1] = 0; robotModel[LEFT_ANKLE_SIDE_SWING].p[2] = 0;
	robotModel[LEFT_ANKLE_SIDE_SWING].R[0][0] = 1; robotModel[LEFT_ANKLE_SIDE_SWING].R[0][1] = 0; robotModel[LEFT_ANKLE_SIDE_SWING].R[0][2] = 0;
	robotModel[LEFT_ANKLE_SIDE_SWING].R[1][0] = 0; robotModel[LEFT_ANKLE_SIDE_SWING].R[1][1] = 1; robotModel[LEFT_ANKLE_SIDE_SWING].R[1][2] = 0;
	robotModel[LEFT_ANKLE_SIDE_SWING].R[2][0] = 0; robotModel[LEFT_ANKLE_SIDE_SWING].R[2][1] = 0; robotModel[LEFT_ANKLE_SIDE_SWING].R[2][2] = 1;
	robotModel[LEFT_ANKLE_SIDE_SWING].v[0] = 0; robotModel[LEFT_ANKLE_SIDE_SWING].v[1] = 0; robotModel[LEFT_ANKLE_SIDE_SWING].v[2] = 0;
	robotModel[LEFT_ANKLE_SIDE_SWING].w[0] = 0;	robotModel[LEFT_ANKLE_SIDE_SWING].w[1] = 0; robotModel[LEFT_ANKLE_SIDE_SWING].w[2] = 0;
	robotModel[LEFT_ANKLE_SIDE_SWING].q = 0;
	robotModel[LEFT_ANKLE_SIDE_SWING].dq = 0;
	robotModel[LEFT_ANKLE_SIDE_SWING].ddq = 0;
	robotModel[LEFT_ANKLE_SIDE_SWING].a[0] = 1; robotModel[LEFT_ANKLE_SIDE_SWING].a[1] = 0; robotModel[LEFT_ANKLE_SIDE_SWING].a[2] = 0;
	robotModel[LEFT_ANKLE_SIDE_SWING].b[0] = 0;	robotModel[LEFT_ANKLE_SIDE_SWING].b[1] = 0; robotModel[LEFT_ANKLE_SIDE_SWING].b[2] = 0;
	robotModel[LEFT_ANKLE_SIDE_SWING].m = 0;
	robotModel[LEFT_ANKLE_SIDE_SWING].c[0] = 0;	robotModel[LEFT_ANKLE_SIDE_SWING].c[1] = 0; robotModel[LEFT_ANKLE_SIDE_SWING].c[2] = 0;
	robotModel[LEFT_ANKLE_SIDE_SWING].I[0][0] = 0; robotModel[LEFT_ANKLE_SIDE_SWING].I[0][1] = 0; robotModel[LEFT_ANKLE_SIDE_SWING].I[0][2] = 0;
	robotModel[LEFT_ANKLE_SIDE_SWING].I[1][0] = 0; robotModel[LEFT_ANKLE_SIDE_SWING].I[1][1] = 0; robotModel[LEFT_ANKLE_SIDE_SWING].I[1][2] = 0;
	robotModel[LEFT_ANKLE_SIDE_SWING].I[2][0] = 0; robotModel[LEFT_ANKLE_SIDE_SWING].I[2][1] = 0; robotModel[LEFT_ANKLE_SIDE_SWING].I[2][2] = 0;

	// id 25
	robotModel[LEFT_FOOT].linkID = LEFT_FOOT;
	robotModel[LEFT_FOOT].name = "左脚端";
	robotModel[LEFT_FOOT].sister = NONE_JOINT;
	robotModel[LEFT_FOOT].child = NONE_JOINT;
	robotModel[LEFT_FOOT].mother = LEFT_ANKLE_SIDE_SWING;
	robotModel[LEFT_FOOT].p[0] = 0; robotModel[LEFT_FOOT].p[1] = 0; robotModel[LEFT_FOOT].p[2] = 0;
	robotModel[LEFT_FOOT].R[0][0] = 1; robotModel[LEFT_FOOT].R[0][1] = 0; robotModel[LEFT_FOOT].R[0][2] = 0;
	robotModel[LEFT_FOOT].R[1][0] = 0; robotModel[LEFT_FOOT].R[1][1] = 1; robotModel[LEFT_FOOT].R[1][2] = 0;
	robotModel[LEFT_FOOT].R[2][0] = 0; robotModel[LEFT_FOOT].R[2][1] = 0; robotModel[LEFT_FOOT].R[2][2] = 1;
	robotModel[LEFT_FOOT].v[0] = 0; robotModel[LEFT_FOOT].v[1] = 0; robotModel[LEFT_FOOT].v[2] = 0;
	robotModel[LEFT_FOOT].w[0] = 0;	robotModel[LEFT_FOOT].w[1] = 0; robotModel[LEFT_FOOT].w[2] = 0;
	robotModel[LEFT_FOOT].q = 0;
	robotModel[LEFT_FOOT].dq = 0;
	robotModel[LEFT_FOOT].ddq = 0;
	robotModel[LEFT_FOOT].a[0] = 0; robotModel[LEFT_FOOT].a[1] = 1; robotModel[LEFT_FOOT].a[2] = 0;
	robotModel[LEFT_FOOT].b[0] = 0;	robotModel[LEFT_FOOT].b[1] = 0; robotModel[LEFT_FOOT].b[2] = -0.0455;
	robotModel[LEFT_FOOT].m = 0;
	robotModel[LEFT_FOOT].c[0] = 0;	robotModel[LEFT_FOOT].c[1] = 0; robotModel[LEFT_FOOT].c[2] = 0;
	robotModel[LEFT_FOOT].I[0][0] = 0; robotModel[LEFT_FOOT].I[0][1] = 0; robotModel[LEFT_FOOT].I[0][2] = 0;
	robotModel[LEFT_FOOT].I[1][0] = 0; robotModel[LEFT_FOOT].I[1][1] = 0; robotModel[LEFT_FOOT].I[1][2] = 0;
	robotModel[LEFT_FOOT].I[2][0] = 0; robotModel[LEFT_FOOT].I[2][1] = 0; robotModel[LEFT_FOOT].I[2][2] = 0;
}

void initRobotPos(){
	robotModel[MAIN_BODY].p[0] = 0;
	robotModel[MAIN_BODY].p[1] = 0;
	robotModel[MAIN_BODY].p[2] = c_h_para;
	forwardKinematics(MAIN_BODY);
	double R[3][3];
	double temp[3];
	rpy2rot(0, 0, 0, R);
	MatrixMultiVector3x1(R, robotModel[LEFT_FOOT].b, temp);
	robotModel[LEFT_ANKLE_SIDE_SWING].p[0] = 0 - temp[0];
	robotModel[LEFT_ANKLE_SIDE_SWING].p[1] = sy / 2 - temp[1];
	robotModel[LEFT_ANKLE_SIDE_SWING].p[2] = 0 - temp[2];
	inverseKinmatics_leftFoot(0, 0, 0);
	MatrixMultiVector3x1(R, robotModel[RIGHT_FOOT].b, temp);
	robotModel[RIGHT_ANKLE_SIDE_SWING].p[0] = 0 - temp[0];
	robotModel[RIGHT_ANKLE_SIDE_SWING].p[1] = -sy / 2 - temp[1];
	robotModel[RIGHT_ANKLE_SIDE_SWING].p[2] = 0 - temp[2];
	inverseKinmatics_rightFoot(0, 0, 0);


	forwardKinematics(MAIN_BODY);
	Calc_com(Com);
	PC_MAIN_BODY[0] = Com[0] - robotModel[MAIN_BODY].p[0];
	PC_MAIN_BODY[1] = Com[1] - robotModel[MAIN_BODY].p[1];
	PC_MAIN_BODY[2] = Com[2] - robotModel[MAIN_BODY].p[2];
	state_space_Com[0][0] = Com[0];
	state_space_Com[1][0] = Com[1];

	robotModel[MAIN_BODY].p[0] = com_const_x_compen;
	robotModel[MAIN_BODY].p[1] = com_const_y_compen;
	robotModel[MAIN_BODY].p[2] = c_h_para;
	forwardKinematics(MAIN_BODY);
	rpy2rot(0, 0, 0, R);
	MatrixMultiVector3x1(R, robotModel[LEFT_FOOT].b, temp);
	robotModel[LEFT_ANKLE_SIDE_SWING].p[0] = 0 - temp[0];
	robotModel[LEFT_ANKLE_SIDE_SWING].p[1] = sy / 2 - temp[1];
	robotModel[LEFT_ANKLE_SIDE_SWING].p[2] = 0 - temp[2];
	inverseKinmatics_leftFoot(0, 0, 0);
	MatrixMultiVector3x1(R, robotModel[RIGHT_FOOT].b, temp);
	robotModel[RIGHT_ANKLE_SIDE_SWING].p[0] = 0 - temp[0];
	robotModel[RIGHT_ANKLE_SIDE_SWING].p[1] = -sy / 2 - temp[1];
	robotModel[RIGHT_ANKLE_SIDE_SWING].p[2] = 0 - temp[2];
	inverseKinmatics_rightFoot(0, 0, 0);
	robotModel[MAIN_BODY].p[0] = 0;
	robotModel[MAIN_BODY].p[1] = 0;
	robotModel[MAIN_BODY].p[2] = c_h_para;

	for(int i = 0; i < 26; i++){
		FallUpRobotPos_q[i] = robotModel[i].q;
	}
#if ROSPUB
	
	for(int i = 0; i < 21; i++){
		std_msgs::Float64 msg;
		ros::Rate ikidPubRate(20);
		msg.data = robotModel[FRONT_NECK_SWING].q/20*i + ikid_robot_zero_point[FRONT_NECK_SWING];
		pub_neck_front_swing.publish(msg);
		msg.data = robotModel[NECK_ROTATION].q/20*i + ikid_robot_zero_point[NECK_ROTATION];
		pub_neck_rotation.publish(msg);
		msg.data = robotModel[LEFT_ARM_FRONT_SWING].q/20*i + ikid_robot_zero_point[LEFT_ARM_FRONT_SWING];
		pub_left_arm_front_swing.publish(msg);
		msg.data = robotModel[LEFT_ARM_SIDE_SWING].q/20*i + ikid_robot_zero_point[LEFT_ARM_SIDE_SWING];
		pub_left_arm_side_swing.publish(msg);
		msg.data = robotModel[LEFT_ARM_ELBOW_FRONT_SWING].q/20*i + ikid_robot_zero_point[LEFT_ARM_ELBOW_FRONT_SWING];
		pub_left_arm_elbow_front_swing.publish(msg);
		msg.data = robotModel[RIGHT_ARM_FRONT_SWING].q/20*i + ikid_robot_zero_point[RIGHT_ARM_FRONT_SWING];
		pub_right_arm_front_swing.publish(msg);
		msg.data = robotModel[RIGHT_ARM_SIDE_SWING].q/20*i + ikid_robot_zero_point[RIGHT_ARM_SIDE_SWING];
		pub_right_arm_side_swing.publish(msg);
		msg.data = robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].q/20*i + ikid_robot_zero_point[RIGHT_ARM_ELBOW_FRONT_SWING];
		pub_right_arm_elbow_front_swing.publish(msg);
		msg.data = robotModel[LEFT_HIP_FRONT_SWING].q/20*i + ikid_robot_zero_point[LEFT_HIP_FRONT_SWING];
		pub_left_hip_front_swing.publish(msg);
		msg.data = robotModel[LEFT_HIP_SIDE_SWING].q/20*i + ikid_robot_zero_point[LEFT_HIP_SIDE_SWING];
		pub_left_hip_side_swing.publish(msg);
		msg.data = robotModel[LEFT_HIP_ROTATION].q/20*i + ikid_robot_zero_point[LEFT_HIP_ROTATION];
		pub_left_hip_rotation.publish(msg);
		msg.data = robotModel[RIGHT_HIP_FRONT_SWING].q/20*i + ikid_robot_zero_point[RIGHT_HIP_FRONT_SWING];
		pub_right_hip_front_swing.publish(msg);
		msg.data = robotModel[RIGHT_HIP_SIDE_SWING].q/20*i + ikid_robot_zero_point[RIGHT_HIP_SIDE_SWING];
		pub_right_hip_side_swing.publish(msg);
		msg.data = robotModel[RIGHT_HIP_ROTATION].q/20*i + ikid_robot_zero_point[RIGHT_HIP_ROTATION];
		pub_right_hip_rotation.publish(msg);
		msg.data = robotModel[LEFT_KNEE_FRONT_SWING].q/20*i + ikid_robot_zero_point[LEFT_KNEE_FRONT_SWING];
		pub_left_knee_front_swing.publish(msg);
		msg.data = robotModel[RIGHT_KNEE_FRONT_SWING].q/20*i + ikid_robot_zero_point[RIGHT_KNEE_FRONT_SWING];
		pub_right_knee_front_swing.publish(msg);
		msg.data = robotModel[LEFT_ANKLE_FRONT_SWING].q/20*i + ikid_robot_zero_point[LEFT_ANKLE_FRONT_SWING];
		pub_left_ankle_front_swing.publish(msg);
		msg.data = robotModel[LEFT_ANKLE_SIDE_SWING].q/20*i + ikid_robot_zero_point[LEFT_ANKLE_SIDE_SWING];
		pub_left_ankle_side_swing.publish(msg);
		msg.data = robotModel[RIGHT_ANKLE_FRONT_SWING].q/20*i + ikid_robot_zero_point[RIGHT_ANKLE_FRONT_SWING];
		pub_right_ankle_front_swing.publish(msg);
		msg.data = robotModel[RIGHT_ANKLE_SIDE_SWING].q/20*i + ikid_robot_zero_point[RIGHT_ANKLE_SIDE_SWING];
		pub_right_ankle_side_swing.publish(msg);
		ikidPubRate.sleep();
	}
#endif
#if CONTROLBOARDPUB
	// 开启舵机扭矩
	std_msgs::Byte torque_msg;
	torque_msg.data = 1;
	pub_control_board_torque_on.publish(torque_msg);
	ros::Duration(0.05).sleep();
	for(int i = 0; i < 21; i++){
		ikid_motion_control::robot_joint control_board_joint_msg;
		control_board_joint_msg.joint = {
			0,
			0,
			robotModel[FRONT_NECK_SWING].q/20*i + ikid_robot_zero_point[FRONT_NECK_SWING],
			robotModel[NECK_ROTATION].q/20*i + ikid_robot_zero_point[NECK_ROTATION],
			robotModel[RIGHT_ARM_FRONT_SWING].q/20*i + ikid_robot_zero_point[RIGHT_ARM_FRONT_SWING],
			robotModel[RIGHT_ARM_SIDE_SWING].q/20*i + ikid_robot_zero_point[RIGHT_ARM_SIDE_SWING],
			robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].q/20*i + ikid_robot_zero_point[RIGHT_ARM_ELBOW_FRONT_SWING],
			0,
			robotModel[LEFT_ARM_FRONT_SWING].q/20*i + ikid_robot_zero_point[LEFT_ARM_FRONT_SWING],
			robotModel[LEFT_ARM_SIDE_SWING].q/20*i + ikid_robot_zero_point[LEFT_ARM_SIDE_SWING],
			robotModel[LEFT_ARM_ELBOW_FRONT_SWING].q/20*i + ikid_robot_zero_point[LEFT_ARM_ELBOW_FRONT_SWING],
			0,
			robotModel[RIGHT_HIP_FRONT_SWING].q/20*i + ikid_robot_zero_point[RIGHT_HIP_FRONT_SWING],
			robotModel[RIGHT_HIP_SIDE_SWING].q/20*i + ikid_robot_zero_point[RIGHT_HIP_SIDE_SWING],
			robotModel[RIGHT_HIP_ROTATION].q/20*i + ikid_robot_zero_point[RIGHT_HIP_ROTATION],
			robotModel[RIGHT_KNEE_FRONT_SWING].q/20*i + ikid_robot_zero_point[RIGHT_KNEE_FRONT_SWING],
			robotModel[RIGHT_ANKLE_FRONT_SWING].q/20*i + ikid_robot_zero_point[RIGHT_ANKLE_FRONT_SWING],
			robotModel[RIGHT_ANKLE_SIDE_SWING].q/20*i + ikid_robot_zero_point[RIGHT_ANKLE_SIDE_SWING],
			0,
			robotModel[LEFT_HIP_FRONT_SWING].q/20*i + ikid_robot_zero_point[LEFT_HIP_FRONT_SWING],
			robotModel[LEFT_HIP_SIDE_SWING].q/20*i + ikid_robot_zero_point[LEFT_HIP_SIDE_SWING],
			robotModel[LEFT_HIP_ROTATION].q/20*i + ikid_robot_zero_point[LEFT_HIP_ROTATION],
			robotModel[LEFT_KNEE_FRONT_SWING].q/20*i + ikid_robot_zero_point[LEFT_KNEE_FRONT_SWING],
			robotModel[LEFT_ANKLE_FRONT_SWING].q/20*i + ikid_robot_zero_point[LEFT_ANKLE_FRONT_SWING],
			robotModel[LEFT_ANKLE_SIDE_SWING].q/20*i + ikid_robot_zero_point[LEFT_ANKLE_SIDE_SWING],
			0
		};
		pub_control_board_joint_msg.publish(control_board_joint_msg);
		ros::Duration(0.02).sleep();
	}

#endif
}

void initRobotPosSpecialGait(){
	robotModel[MAIN_BODY].p[0] = 0;
	robotModel[MAIN_BODY].p[1] = 0;
	robotModel[MAIN_BODY].p[2] = c_h_para;
	forwardKinematics(MAIN_BODY);
	double R[3][3];
	double temp[3];
	rpy2rot(0, 0, 0, R);
	MatrixMultiVector3x1(R, robotModel[LEFT_FOOT].b, temp);
	robotModel[LEFT_ANKLE_SIDE_SWING].p[0] = 0 - temp[0];
	robotModel[LEFT_ANKLE_SIDE_SWING].p[1] = sy / 2 - temp[1];
	robotModel[LEFT_ANKLE_SIDE_SWING].p[2] = 0 - temp[2];
	inverseKinmatics_leftFoot(0, 0, 0);
	MatrixMultiVector3x1(R, robotModel[RIGHT_FOOT].b, temp);
	robotModel[RIGHT_ANKLE_SIDE_SWING].p[0] = 0 - temp[0];
	robotModel[RIGHT_ANKLE_SIDE_SWING].p[1] = -sy / 2 - temp[1];
	robotModel[RIGHT_ANKLE_SIDE_SWING].p[2] = 0 - temp[2];
	inverseKinmatics_rightFoot(0, 0, 0);


	forwardKinematics(MAIN_BODY);
	Calc_com(Com);
	PC_MAIN_BODY[0] = Com[0] - robotModel[MAIN_BODY].p[0];
	PC_MAIN_BODY[1] = Com[1] - robotModel[MAIN_BODY].p[1];
	PC_MAIN_BODY[2] = Com[2] - robotModel[MAIN_BODY].p[2];
	state_space_Com[0][0] = Com[0];
	state_space_Com[1][0] = Com[1];

	for(int i = 0; i < 26; i++){
		FallUpRobotPos_q[i] = robotModel[i].q;
	}
}

void robotStart(ros::NodeHandle& n_)
{
	readIkidRobotZeroPoint(0);
	robotModelInit(robotModel);
	ikidRobotDynaPosPubInit(n_);
#if WRITETXT
	clearTxt();
#endif
	ros::param::get("/pid_amend/imu_roll_p",imu_roll_p);
	ros::param::get("/pid_amend/imu_roll_i",imu_roll_i);
	ros::param::get("/pid_amend/imu_roll_d",imu_roll_d);
	ros::param::get("/pid_amend/imu_pitch_p",imu_pitch_p);
	ros::param::get("/pid_amend/imu_pitch_i",imu_pitch_i);
	ros::param::get("/pid_amend/imu_pitch_d",imu_pitch_d);
	ros::param::get("/pid_amend/imu_yaw_p",imu_yaw_p);
	ros::param::get("/pid_amend/imu_yaw_i",imu_yaw_i);
	ros::param::get("/pid_amend/imu_yaw_d",imu_yaw_d);
	ros::param::get("/pid_amend/imu_com_x_p",imu_com_x_p);
	ros::param::get("/pid_amend/imu_com_x_i",imu_com_x_i);
	ros::param::get("/pid_amend/imu_com_x_d",imu_com_x_d);
	ros::param::get("/pid_amend/imu_com_y_p",imu_com_y_p);
	ros::param::get("/pid_amend/imu_com_y_i",imu_com_y_i);
	ros::param::get("/pid_amend/imu_com_y_d",imu_com_y_d);
	ros::param::get("/pid_amend/com_const_x_compen",com_const_x_compen);
	ros::param::get("/pid_amend/com_const_y_compen",com_const_y_compen);
	ros::param::get("/pid_amend/walk_length",walk_length);
	ros::param::get("/pid_amend/walk_width",walk_width);
	ros::param::get("/pid_amend/walk_frame_T",walk_frame_T);
	ros::param::get("/pid_amend/c_h_para",c_h_para);
	ros::param::get("/pid_amend/foot_hight",fh);
	sx = walk_length;
    sy = walk_width;
	pn[0] = 0.0;
	pn[1] = sy / 2;
	pn[2] = 0.0;
	initRobotPos();
}

void robotStartSpecialGait(ros::NodeHandle& n_)
{
	readIkidRobotZeroPoint(0);
	robotModelInit(robotModel);
	ikidRobotDynaPosPubInit(n_);
	ros::param::get("/pid_amend/walk_length",walk_length);
	ros::param::get("/pid_amend/walk_width",walk_width);
	ros::param::get("/pid_amend/walk_frame_T",walk_frame_T);
	ros::param::get("/pid_amend/walk_frame_T",walk_frame_T);
	ros::param::get("/pid_amend/c_h_para",c_h_para);
	sx = walk_length;
    sy = walk_width;
	pn[0] = 0.0;
	pn[1] = sy / 2;
	pn[2] = 0.0;
	initRobotPosSpecialGait();
}

void MatrixSquare3x3(double a[3][3], double a_square[3][3]) {
	a_square[0][0] = a[0][0] * a[0][0] + a[0][1] * a[1][0] + a[0][2] * a[2][0];
	a_square[0][1] = a[0][0] * a[0][1] + a[0][1] * a[1][1] + a[0][2] * a[2][1];
	a_square[0][2] = a[0][0] * a[0][2] + a[0][1] * a[1][2] + a[0][2] * a[2][2];
	a_square[1][0] = a[1][0] * a[0][0] + a[1][1] * a[1][0] + a[1][2] * a[2][0];
	a_square[1][1] = a[1][0] * a[0][1] + a[1][1] * a[1][1] + a[1][2] * a[2][1];
	a_square[1][2] = a[1][0] * a[0][2] + a[1][1] * a[1][2] + a[1][2] * a[2][2];
	a_square[2][0] = a[2][0] * a[0][0] + a[2][1] * a[1][0] + a[2][2] * a[2][0];
	a_square[2][1] = a[2][0] * a[0][1] + a[2][1] * a[1][1] + a[2][2] * a[2][1];
	a_square[2][2] = a[2][0] * a[0][2] + a[2][1] * a[1][2] + a[2][2] * a[2][2];
}

void MatrixMultiMatrix3x3(double a[3][3], double b[3][3], double result[3][3]) {
	result[0][0] = a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0];
	result[0][1] = a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1];
	result[0][2] = a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2];
	result[1][0] = a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0];
	result[1][1] = a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1];
	result[1][2] = a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2];
	result[2][0] = a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0];
	result[2][1] = a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1];
	result[2][2] = a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2];
}

void MatrixMultiVector3x1(double a[3][3], double b[3], double result[3]) {
	result[0] = a[0][0] * b[0] + a[0][1] * b[1] + a[0][2] * b[2];
	result[1] = a[1][0] * b[0] + a[1][1] * b[1] + a[1][2] * b[2];
	result[2] = a[2][0] * b[0] + a[2][1] * b[1] + a[2][2] * b[2];
}

void MatrixMultiVector6x1(double a[6][6], double b[6], double result[6]) {
	result[0] = a[0][0] * b[0] + a[0][1] * b[1] + a[0][2] * b[2] + a[0][3] * b[3] + a[0][4] * b[4] + a[0][5] * b[5];
	result[1] = a[1][0] * b[0] + a[1][1] * b[1] + a[1][2] * b[2] + a[1][3] * b[3] + a[1][4] * b[4] + a[1][5] * b[5];
	result[2] = a[2][0] * b[0] + a[2][1] * b[1] + a[2][2] * b[2] + a[2][3] * b[3] + a[2][4] * b[4] + a[2][5] * b[5];
	result[3] = a[3][0] * b[0] + a[3][1] * b[1] + a[3][2] * b[2] + a[3][3] * b[3] + a[3][4] * b[4] + a[3][5] * b[5];
	result[4] = a[4][0] * b[0] + a[4][1] * b[1] + a[4][2] * b[2] + a[4][3] * b[3] + a[4][4] * b[4] + a[4][5] * b[5];
	result[5] = a[5][0] * b[0] + a[5][1] * b[1] + a[5][2] * b[2] + a[5][3] * b[3] + a[5][4] * b[4] + a[5][5] * b[5];
}

void MatrixMultiVector8x1(double a[8][8], double b[8], double result[8]){
	result[0] = a[0][0] * b[0] + a[0][1] * b[1] + a[0][2] * b[2] + a[0][3] * b[3] + a[0][4] * b[4] + a[0][5] * b[5] + a[0][6] * b[6] + a[0][7] * b[7];
	result[1] = a[1][0] * b[0] + a[1][1] * b[1] + a[1][2] * b[2] + a[1][3] * b[3] + a[1][4] * b[4] + a[1][5] * b[5] + a[1][6] * b[6] + a[1][7] * b[7];
	result[2] = a[2][0] * b[0] + a[2][1] * b[1] + a[2][2] * b[2] + a[2][3] * b[3] + a[2][4] * b[4] + a[2][5] * b[5] + a[2][6] * b[6] + a[2][7] * b[7];
	result[3] = a[3][0] * b[0] + a[3][1] * b[1] + a[3][2] * b[2] + a[3][3] * b[3] + a[3][4] * b[4] + a[3][5] * b[5] + a[3][6] * b[6] + a[3][7] * b[7];
	result[4] = a[4][0] * b[0] + a[4][1] * b[1] + a[4][2] * b[2] + a[4][3] * b[3] + a[4][4] * b[4] + a[4][5] * b[5] + a[4][6] * b[6] + a[4][7] * b[7];
	result[5] = a[5][0] * b[0] + a[5][1] * b[1] + a[5][2] * b[2] + a[5][3] * b[3] + a[5][4] * b[4] + a[5][5] * b[5] + a[5][6] * b[6] + a[5][7] * b[7];
	result[6] = a[6][0] * b[0] + a[6][1] * b[1] + a[6][2] * b[2] + a[6][3] * b[3] + a[6][4] * b[4] + a[6][5] * b[5] + a[6][6] * b[6] + a[6][7] * b[7];
	result[7] = a[7][0] * b[0] + a[7][1] * b[1] + a[7][2] * b[2] + a[7][3] * b[3] + a[7][4] * b[4] + a[7][5] * b[5] + a[7][6] * b[6] + a[7][7] * b[7];
}

void VectorAddVector3x1(double a[3], double b[3], double result[3]) {
	result[0] = a[0] + b[0];
	result[1] = a[1] + b[1];
	result[2] = a[2] + b[2];
}

void invMatrix3x3(double matrix[3][3], double result[3][3]) {
	double a1 = matrix[0][0]; double b1 = matrix[0][1]; double c1 = matrix[0][2];
	double a2 = matrix[1][0]; double b2 = matrix[1][1]; double c2 = matrix[1][2];
	double a3 = matrix[2][0]; double b3 = matrix[2][1]; double c3 = matrix[2][2];
	double lamda = 1 / (a1 * (b2 * c3 - c2 * b3) - a2 * (b1 * c3 - c1 * b3) + a3 * (b1 * c2 - c1 * b2));
	result[0][0] = lamda * (b2 * c3 - c2 * b3); result[0][1] = lamda * (c1 * b3 - b1 * c3); result[0][2] = lamda * (b1 * c2 - c1 * b2);
	result[1][0] = lamda * (c2 * a3 - a2 * c3); result[1][1] = lamda * (a1 * c3 - c1 * a3); result[1][2] = lamda * (a2 * c1 - a1 * c2);
	result[2][0] = lamda * (a2 * b3 - b2 * a3); result[2][1] = lamda * (b1 * a3 - a1 * b3); result[2][2] = lamda * (a1 * b2 - a2 * b1);
}

void matrix_inverse_LU(double a[6][6], double a_inverse[6][6])
{
	const int N = 6;
	double l[N][N], u[N][N];
	double l_inverse[N][N], u_inverse[N][N];
	int i, j, k;
	double s, t;

	memset(l, 0, sizeof(l));
	memset(u, 0, sizeof(u));
	memset(l_inverse, 0, sizeof(l_inverse));
	memset(u_inverse, 0, sizeof(u_inverse));
	memset(a_inverse, 0, sizeof(u_inverse));

	for (i = 0; i < N; i++)		//计算l矩阵对角线
	{
		l[i][i] = 1;
	}

	for (i = 0; i < N; i++)
	{
		for (j = i; j < N; j++)
		{
			s = 0;
			for (k = 0; k < i; k++)
			{
				s += l[i][k] * u[k][j];
			}
			u[i][j] = a[i][j] - s;		//按行计算u值
		}

		for (j = i + 1; j < N; j++)
		{
			s = 0;
			for (k = 0; k < i; k++)
			{
				s += l[j][k] * u[k][i];
			}
			l[j][i] = (a[j][i] - s) / u[i][i];		//按列计算l值
		}
	}

	for (i = 0; i < N; i++)		//按行序，行内从高到低，计算l的逆矩阵
	{
		l_inverse[i][i] = 1;
	}
	for (i = 1; i < N; i++)
	{
		for (j = 0; j < i; j++)
		{
			s = 0;
			for (k = 0; k < i; k++)
			{
				s += l[i][k] * l_inverse[k][j];
			}
			l_inverse[i][j] = -s;
		}
	}

#if DEBUG
	printf("test l_inverse:\n");
	for (i = 0; i < N; i++)
	{
		for (j = 0; j < N; j++)
		{
			s = 0;
			for (k = 0; k < N; k++)
			{
				s += l[i][k] * l_inverse[k][j];
			}

			printf("%f ", s);
		}
		putchar('\n');
	}
#endif

	for (i = 0; i < N; i++)					//按列序，列内按照从下到上，计算u的逆矩阵
	{
		u_inverse[i][i] = 1 / u[i][i];
	}
	for (i = 1; i < N; i++)
	{
		for (j = i - 1; j >= 0; j--)
		{
			s = 0;
			for (k = j + 1; k <= i; k++)
			{
				s += u[j][k] * u_inverse[k][i];
			}
			u_inverse[j][i] = -s / u[j][j];
		}
	}

#if DEBUG
	printf("test u_inverse:\n");
	for (i = 0; i < N; i++)
	{
		for (j = 0; j < N; j++)
		{
			s = 0;
			for (k = 0; k < N; k++)
			{
				s += u[i][k] * u_inverse[k][j];
			}

			printf("%f ", s);
		}
		putchar('\n');
	}
#endif

	for (i = 0; i < N; i++)			//计算矩阵a的逆矩阵
	{
		for (j = 0; j < N; j++)
		{
			for (k = 0; k < N; k++)
			{
				a_inverse[i][j] += u_inverse[i][k] * l_inverse[k][j];
			}
		}
	}

#if DEBUG
	printf("test a:\n");
	for (i = 0; i < N; i++)
	{
		for (j = 0; j < N; j++)
		{
			s = 0;
			for (k = 0; k < N; k++)
			{
				s += a[i][k] * a_inverse[k][j];
			}

			printf("%f ", s);
		}
		putchar('\n');
	}
#endif
}

void LU_decomposition(double arr[6][6], double W_n[6][6])
{
	const int N = 6;
	double W[N][N], L[N][N], U[N][N], L_n[N][N], U_n[N][N];
	int i, j, k, d;
	double s;

	// 赋初值
	for (i = 0; i < N; i++) {
		for (j = 0; j < N; j++) {
			W[i][j] = (double)arr[i][j];
			L[i][j] = 0;
			U[i][j] = 0;
			L_n[i][j] = 0;
			U_n[i][j] = 0;
			W_n[i][j] = 0;
		}
	}

	for (i = 0; i < N; i++)  // L对角置1
	{
		L[i][i] = 1.0;
	}

	for (j = 0; j < N; j++)
	{
		U[0][j] = W[0][j];
	}

	for (i = 1; i < N; i++)
	{
		L[i][0] = W[i][0] / U[0][0];
	}

	for (i = 1; i < N; i++)
	{
		for (j = i; j < N; j++) // 求U
		{
			s = 0;
			for (k = 0; k < i; k++)
			{
				s += L[i][k] * U[k][j];
			}
			U[i][j] = W[i][j] - s;
		}

		for (d = i; d < N; d++) // 求L
		{
			s = 0;
			for (k = 0; k < i; k++)
			{
				s += L[d][k] * U[k][i];
			}
			L[d][i] = (W[d][i] - s) / U[i][i];
		}
	}

	for (j = 0; j < N; j++)  //求L的逆
	{
		for (i = j; i < N; i++)
		{
			if (i == j)
				L_n[i][j] = 1 / L[i][j];
			else if (i < j)
				L_n[i][j] = 0;
			else
			{
				s = 0.;
				for (k = j; k < i; k++)
				{
					s += L[i][k] * L_n[k][j];
				}
				L_n[i][j] = -L_n[j][j] * s;
			}
		}
	}

	for (i = 0; i < N; i++)  //求U的逆
	{
		for (j = i; j >= 0; j--)
		{
			if (i == j)
				U_n[j][i] = 1 / U[j][i];
			else if (j > i)
				U_n[j][i] = 0;
			else
			{
				s = 0.;
				for (k = j + 1; k <= i; k++)
				{
					s += U[j][k] * U_n[k][i];
				}
				U_n[j][i] = -1 / U[j][j] * s;
			}
		}
	}


	for (i = 0; i < N; i++)
	{
		for (j = 0; j < N; j++)
		{
			for (k = 0; k < N; k++)
			{
				W_n[i][j] += U_n[i][k] * L_n[k][j];
			}
		}
	}

}

bool sign(double value) {
	if (value >= 0)
		return 1;
	else
		return -1;
}

double norm(double a[], int m, int n)
{
	double val = 0;
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++) {
			val = a[i * m + j] * a[i * m + j];
		}
	}
	val = sqrt(val / (m * n));
	return val;
}

void rodrigues(double* a, double q, double R[3][3])
{
	double a_m[3][3];   // 叉乘矩阵，两个向量叉乘可转换为一个矩阵乘以一个向量，此处为a向量转换为叉乘矩阵a_m
	a_m[0][0] = 0; a_m[0][1] = -a[2]; a_m[0][2] = a[1];
	a_m[1][0] = a[2]; a_m[1][1] = 0; a_m[1][2] = -a[0];
	a_m[2][0] = -a[1]; a_m[2][1] = a[0]; a_m[2][2] = 0;
	double a_m_square[3][3];
	MatrixSquare3x3(a_m, a_m_square);
	R[0][0] = eye[0][0]+a_m[0][0]*sin(q)+ a_m_square[0][0]*(1-cos(q));
	R[0][1] = eye[0][1]+a_m[0][1]*sin(q)+ a_m_square[0][1]*(1-cos(q));
	R[0][2] = eye[0][2]+a_m[0][2]*sin(q)+ a_m_square[0][2]*(1-cos(q));
	R[1][0] = eye[1][0]+a_m[1][0]*sin(q)+ a_m_square[1][0]*(1-cos(q));
	R[1][1] = eye[1][1]+a_m[1][1]*sin(q)+ a_m_square[1][1]*(1-cos(q));
	R[1][2] = eye[1][2]+a_m[1][2]*sin(q)+ a_m_square[1][2]*(1-cos(q));
	R[2][0] = eye[2][0]+a_m[2][0]*sin(q)+ a_m_square[2][0]*(1-cos(q));
	R[2][1] = eye[2][1]+a_m[2][1]*sin(q)+ a_m_square[2][1]*(1-cos(q));
	R[2][2] = eye[2][2]+a_m[2][2]*sin(q)+ a_m_square[2][2]*(1-cos(q));
}

void forwardKinematics(unsigned int linkID)
{
	if (linkID == NONE_JOINT)
		return;
	if (linkID != MAIN_BODY) {
		int mother = robotModel[linkID].mother;
		double Rxb[3];
		MatrixMultiVector3x1(robotModel[mother].R, robotModel[linkID].b, Rxb);
		VectorAddVector3x1(Rxb, robotModel[mother].p, robotModel[linkID].p);
		double rodriguesResult[3][3];
		rodrigues(robotModel[linkID].a, robotModel[linkID].q, rodriguesResult);
		MatrixMultiMatrix3x3(robotModel[mother].R, rodriguesResult, robotModel[linkID].R);
	}
#if DEBUG
	printf("id: %d  p:%f %f %f\n", linkID, robotModel[linkID].p[0], robotModel[linkID].p[1],robotModel[linkID].p[2]);
#endif
	forwardKinematics(robotModel[linkID].child);
	forwardKinematics(robotModel[linkID].sister);
}

void rot2omega(double a[3][3], double omega[3]) {
	double sum = 0;
	double theta = 0;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++) {
			sum = fabs(a[i][j] - eye[i][j]) + sum;
		}
	}
	if (sum <= 1e-7) {
		omega[0] = 0; omega[1] = 0; omega[2] = 0;
	}
	else {
		theta = acos((a[0][0] + a[1][1] + a[2][2] - 1) / 2);
		omega[0] = theta / (2 * sin(theta)) * (a[2][1] - a[1][2]);
		omega[1] = theta / (2 * sin(theta)) * (a[0][2] - a[2][0]);
		omega[2] = theta / (2 * sin(theta)) * (a[1][0] - a[0][1]);
	}
}

double totalMass(unsigned int linkID) {
	double m;
	if (linkID == NONE_JOINT)
		m = 0;
	else {
		m = robotModel[linkID].m + totalMass(robotModel[linkID].child) + totalMass(robotModel[linkID].sister);
	}
	return m;
}

void Ryaw(double q, double result[3][3]) {
	// 偏航角对应的旋转矩阵
	result[0][0] = cos(q); result[0][1] = -sin(q); result[0][2] = 0;
	result[1][0] = sin(q); result[1][1] =  cos(q); result[1][2] = 0;
	result[2][0] =      0; result[2][1] =       0; result[2][2] = 1;
}

void Rroll(double q, double result[3][3]) {
	// 翻滚角对应的旋转矩阵
	result[0][0] = 1; result[0][1] =      0; result[0][2] =       0;
	result[1][0] = 0; result[1][1] = cos(q); result[1][2] = -sin(q);
	result[2][0] = 0; result[2][1] = sin(q); result[2][2] =  cos(q);
}

void Rpitch(double q, double result[3][3]) {
	// 俯仰角对应的旋转矩阵
	result[0][0] =  cos(q); result[0][1] = 0; result[0][2] = sin(q);
	result[1][0] =       0; result[1][1] = 1; result[1][2] =      0;
	result[2][0] = -sin(q); result[2][1] = 0; result[2][2] = cos(q);
}

void rpy2rot(double r,double p, double y, double result[3][3]) {
	// 俯仰角对应的旋转矩阵
	double  result_r[3][3];
	double  result_p[3][3];
	double  result_y[3][3];
	double temp[3][3];
	Rroll(r, result_r);
	Rpitch(p, result_p);
	Ryaw(y, result_y);
	MatrixMultiMatrix3x3(result_y, result_p, temp);
	MatrixMultiMatrix3x3(temp, result_r, result);
}

void cross3x1(double a[3], double b[3], double result[3]) {
	result[0] = a[1] * b[2] - b[1] * a[2];
	result[1] = -(a[0] * b[2] - b[0] * a[2]);
	result[2] = a[0] * b[1] - b[0] * a[1];
}

void ForwardVelocity(unsigned int linkID) {
	if (linkID == NONE_JOINT)
		return;
	if (linkID != MAIN_BODY) {
		int mother = robotModel[linkID].mother;
		double Rxb[3];
		double result[3];
		MatrixMultiVector3x1(robotModel[mother].R, robotModel[mother].b, Rxb);
		cross3x1(robotModel[mother].w, Rxb, result);
		VectorAddVector3x1(robotModel[mother].v, result, robotModel[linkID].v);
		double aMultidq[3];
		aMultidq[0] = robotModel[linkID].a[0] * robotModel[linkID].dq;
		aMultidq[1] = robotModel[linkID].a[1] * robotModel[linkID].dq;
		aMultidq[2] = robotModel[linkID].a[2] * robotModel[linkID].dq;
		MatrixMultiVector3x1(robotModel[mother].R, aMultidq, result);
		VectorAddVector3x1(robotModel[mother].w, result, robotModel[linkID].w);
	}
	ForwardVelocity(robotModel[linkID].child);
	ForwardVelocity(robotModel[linkID].sister);
}

void CalcP(unsigned int linkID, double P[3]) {
	if (linkID == NONE_JOINT) {
		P[0] = 0; P[1] = 0; P[2] = 0;
	}
	else {
		double c1[3];
		double result[3];
		double temp[3];
		double temp2[3];
		MatrixMultiVector3x1(robotModel[linkID].R, robotModel[linkID].c, c1);
		cross3x1(robotModel[linkID].w, c1, result);
		temp[0] = robotModel[linkID].v[0] + result[0];
		temp[1] = robotModel[linkID].v[1] + result[1];
		temp[2] = robotModel[linkID].v[2] + result[2];
		result[0] = robotModel[linkID].m * temp[0];
		result[1] = robotModel[linkID].m * temp[1];
		result[2] = robotModel[linkID].m * temp[2];
		CalcP(robotModel[linkID].child, temp);
		CalcP(robotModel[linkID].sister, temp2);
		P[0] = result[0] + temp[0] + temp2[0];
		P[1] = result[1] + temp[1] + temp2[1];
		P[2] = result[2] + temp[2] + temp2[2];
	}
}

void R_T3x3(double R[3][3], double R_T[3][3]) {
	R_T[0][0] = R[0][0]; R_T[0][1] = R[1][0]; R_T[0][2] = R[2][0];
	R_T[1][0] = R[0][1]; R_T[1][1] = R[1][1]; R_T[1][2] = R[2][1];
	R_T[2][0] = R[0][2]; R_T[2][1] = R[1][2]; R_T[2][2] = R[2][2];
}

void CalcL(unsigned int linkID, double L[3]) {
	if (linkID == NONE_JOINT) {
		L[0] = 0; L[1] = 0; L[2] = 0;
	}
	else {
		double c[3];
		double c1[3];
		double result[3];
		double temp[3];
		double temp2[3];
		MatrixMultiVector3x1(robotModel[linkID].R, robotModel[linkID].c, c1);
		VectorAddVector3x1(robotModel[linkID].p, c1, c);
		cross3x1(robotModel[linkID].w, c1, result);
		temp[0] = robotModel[linkID].v[0] + result[0];
		temp[1] = robotModel[linkID].v[1] + result[1];
		temp[2] = robotModel[linkID].v[2] + result[2];
		result[0] = robotModel[linkID].m * temp[0];
		result[1] = robotModel[linkID].m * temp[1];
		result[2] = robotModel[linkID].m * temp[2];
		cross3x1(c, result, temp);
		double RT[3][3];
		double result2[3][3];
		double result3[3][3];
		R_T3x3(robotModel[linkID].R, RT);
		MatrixMultiMatrix3x3(robotModel[linkID].R, robotModel[linkID].I, result2);
		MatrixMultiMatrix3x3(result2, RT, result3);
		MatrixMultiVector3x1(result3, robotModel[linkID].w, result);
		VectorAddVector3x1(temp, result, temp2);

		CalcL(robotModel[linkID].child, temp);
		CalcL(robotModel[linkID].sister, result);
		L[0] = temp2[0] + temp[0] + result[0];
		L[1] = temp2[1] + temp[1] + result[1];
		L[2] = temp2[2] + temp[2] + result[2];
	}
}

void Calc_mc(unsigned int linkID, double mc[3]) {
	if (linkID == NONE_JOINT) {
		mc[0] = 0; mc[1] = 0; mc[2] = 0;
	}
	else
	{
		double result[3];
		double temp[3];
		double temp2[3];
		MatrixMultiVector3x1(robotModel[linkID].R, robotModel[linkID].c, result);
		VectorAddVector3x1(robotModel[linkID].p, result, temp);
		temp2[0] = robotModel[linkID].m * temp[0];
		temp2[1] = robotModel[linkID].m * temp[1];
		temp2[2] = robotModel[linkID].m * temp[2];
		Calc_mc(robotModel[linkID].child, result);
		Calc_mc(robotModel[linkID].sister, temp);
		mc[0] = temp2[0] + result[0] + temp[0];
		mc[1] = temp2[1] + result[1] + temp[1];
		mc[2] = temp2[2] + result[2] + temp[2];
	}
}

void Calc_com(double com[3]) {
	//计算机器人模型的质心世界坐标
	double M;
	double mc[3];
	M = totalMass(MAIN_BODY);
	Calc_mc(MAIN_BODY,mc);
	com[0] = mc[0] / M;
	com[1] = mc[1] / M;
	com[2] = mc[2] / M;
}

void Calc_ZMP(double fact_zmp[3], double *taoz){
	double M;
	M = totalMass(MAIN_BODY);
	double G = 9.8;
	double temp_com[3];
	Calc_com(temp_com);
	CalcP(MAIN_BODY, cur_robot_P);
	CalcL(MAIN_BODY, cur_robot_L);
	robot_dPdt[0] = (cur_robot_P[0]-pre_robot_P[0])/frame_T;
	robot_dPdt[1] = (cur_robot_P[1]-pre_robot_P[1])/frame_T;
	robot_dPdt[2] = (cur_robot_P[2]-pre_robot_P[2])/frame_T;
	robot_dLdt[0] = (cur_robot_L[0]-pre_robot_L[0])/frame_T;
	robot_dLdt[1] = (cur_robot_L[1]-pre_robot_L[1])/frame_T;
	robot_dLdt[2] = (cur_robot_L[2]-pre_robot_L[2])/frame_T;
	fact_zmp[0] = (M*G*temp_com[0]-robot_dLdt[1])/(M*G+robot_dPdt[2]);
	fact_zmp[1] = (M*G*temp_com[1]+robot_dLdt[0])/(M*G+robot_dPdt[2]);
	fact_zmp[2] = 0;
	*taoz = robot_dLdt[2]+robot_dPdt[0]*fact_zmp[1]-robot_dPdt[1]*fact_zmp[0];
	pre_robot_P[0] = cur_robot_P[0];pre_robot_P[1] = cur_robot_P[1];pre_robot_P[2] = cur_robot_P[2];
	pre_robot_L[0] = cur_robot_L[0];pre_robot_L[1] = cur_robot_L[1];pre_robot_L[2] = cur_robot_L[2];
}

void changeFoot()
{
	if (isLeft) {
		isLeft = false;
	}
	else
	{
		isLeft = true;
	}
	support_ZMP[0] = pn[0];
	support_ZMP[1] = pn[1];
}

void angleLimit()
{
	//MAIN_BODY = 0,
	/*HEAD,
		FRONT_NECK_SWING,
		NECK_ROTATION,
		RIGHT_ARM_FRONT_SWING,
		RIGHT_ARM_SIDE_SWING,
		RIGHT_ARM_ELBOW_FRONT_SWING,
		RIGHT_HAND,
		LEFT_ARM_FRONT_SWING,
		LEFT_ARM_SIDE_SWING,
		LEFT_ARM_ELBOW_FRONT_SWING,
		LEFT_HAND,
		RIGHT_HIP_FRONT_SWING,
		RIGHT_HIP_SIDE_SWING,
		RIGHT_HIP_ROTATION,
		RIGHT_KNEE_FRONT_SWING,
		RIGHT_ANKLE_FRONT_SWING,
		RIGHT_ANKLE_SIDE_SWING,
		RIGHT_FOOT,
		LEFT_HIP_FRONT_SWING,
		LEFT_HIP_SIDE_SWING,
		LEFT_HIP_ROTATION,
		LEFT_KNEE_FRONT_SWING,
		LEFT_ANKLE_FRONT_SWING,
		LEFT_ANKLE_SIDE_SWING,
		LEFT_FOOT*/
	if (robotModel[FRONT_NECK_SWING].q < 0 || robotModel[FRONT_NECK_SWING].q > PI / 2)
		robotModel[FRONT_NECK_SWING].q < 0 ? robotModel[FRONT_NECK_SWING].q = 0 : robotModel[FRONT_NECK_SWING].q = PI / 2;
	if (robotModel[NECK_ROTATION].q < -PI/2 || robotModel[NECK_ROTATION].q > PI / 2)
		robotModel[NECK_ROTATION].q < -PI / 2 ? robotModel[NECK_ROTATION].q = -PI / 2 : robotModel[NECK_ROTATION].q = PI / 2;
	if (robotModel[RIGHT_ARM_FRONT_SWING].q < -PI || robotModel[RIGHT_ARM_FRONT_SWING].q > PI / 2)
		robotModel[RIGHT_ARM_FRONT_SWING].q < -PI  ? robotModel[RIGHT_ARM_FRONT_SWING].q = -PI  : robotModel[RIGHT_ARM_FRONT_SWING].q = PI / 3;
	if (robotModel[RIGHT_ARM_SIDE_SWING].q < -PI/10 || robotModel[RIGHT_ARM_SIDE_SWING].q > PI/2)
		robotModel[RIGHT_ARM_SIDE_SWING].q < -PI / 10 ? robotModel[RIGHT_ARM_SIDE_SWING].q = -PI / 10 : robotModel[RIGHT_ARM_SIDE_SWING].q = PI / 2;
	if (robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].q < -2*PI/3 || robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].q > 0)
		robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].q < -2 * PI / 3 ? robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].q = -2 * PI / 3 : robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].q = 0;
	if (robotModel[LEFT_ARM_FRONT_SWING].q < -PI || robotModel[LEFT_ARM_FRONT_SWING].q > PI / 2)
		robotModel[LEFT_ARM_FRONT_SWING].q < -PI ? robotModel[LEFT_ARM_FRONT_SWING].q = -PI : robotModel[LEFT_ARM_FRONT_SWING].q = PI / 3;
	if (robotModel[LEFT_ARM_SIDE_SWING].q < -PI/2 || robotModel[LEFT_ARM_SIDE_SWING].q > PI/10)
		robotModel[LEFT_ARM_SIDE_SWING].q < -PI / 2 ? robotModel[LEFT_ARM_SIDE_SWING].q = -PI / 2 : robotModel[LEFT_ARM_SIDE_SWING].q = PI/10;
	if (robotModel[LEFT_ARM_ELBOW_FRONT_SWING].q < -2 * PI / 3 || robotModel[LEFT_ARM_ELBOW_FRONT_SWING].q > 0)
		robotModel[LEFT_ARM_ELBOW_FRONT_SWING].q < -2 * PI / 3 ? robotModel[LEFT_ARM_ELBOW_FRONT_SWING].q = -2 * PI / 3 : robotModel[LEFT_ARM_ELBOW_FRONT_SWING].q = 0;
}

void inverseKinmatics_head() {
	double R3T[3][3];
	R_T3x3(robotModel[NECK_ROTATION].R, R3T);
	double result3x1[3];
	result3x1[0] = robotModel[HEAD].p[0] - robotModel[NECK_ROTATION].p[0];
	result3x1[1] = robotModel[HEAD].p[1] - robotModel[NECK_ROTATION].p[1];
	result3x1[2] = robotModel[HEAD].p[2] - robotModel[NECK_ROTATION].p[2];
	double temp[3];
	MatrixMultiVector3x1(R3T, result3x1, temp);
	double a = robotModel[HEAD].b[2];
	double b = robotModel[FRONT_NECK_SWING].b[2];
	double a_square = a*a;
	double b_square = b*b;
	double c_square = temp[0] * temp[0] + temp[1] * temp[1] + temp[2] * temp[2];
	robotModel[FRONT_NECK_SWING].dq = robotModel[FRONT_NECK_SWING].q;
	robotModel[NECK_ROTATION].dq = robotModel[NECK_ROTATION].q;
	robotModel[FRONT_NECK_SWING].q = PI - acos((a_square + b_square - c_square) / (2 * a * b));
	robotModel[NECK_ROTATION].q = atan2(temp[1], temp[0]);
	robotModel[FRONT_NECK_SWING].dq = -(robotModel[FRONT_NECK_SWING].dq-robotModel[FRONT_NECK_SWING].q)/frame_T;
	robotModel[NECK_ROTATION].dq = -(robotModel[NECK_ROTATION].dq-robotModel[NECK_ROTATION].q)/frame_T;
	ForwardVelocity(MAIN_BODY);
}

void inverseKinmatics_leftHand() {

	/*double R11T[3][3];
	R_T3x3(robotModel[LEFT_HAND].R, R11T);
	double result3x1[3];
	result3x1[0] = robotModel[LEFT_ARM_SIDE_SWING].p[0] - robotModel[LEFT_HAND].p[0];
	result3x1[1] = robotModel[LEFT_ARM_SIDE_SWING].p[1] - robotModel[LEFT_HAND].p[1];
	result3x1[2] = robotModel[LEFT_ARM_SIDE_SWING].p[2] - robotModel[LEFT_HAND].p[2];
	double temp[3];
	MatrixMultiVector3x1(R11T, result3x1, temp);
	double a = robotModel[LEFT_ARM_ELBOW_FRONT_SWING].b[2];
	double b = robotModel[LEFT_HAND].b[2];
	double a_square = a * a;
	double b_square = b * b;
	double c_square = temp[0] * temp[0] + temp[1] * temp[1] + temp[2] * temp[2];
	double c = sqrt(c_square);
	robotModel[LEFT_HAND].q = -atan2(temp[1], temp[2]);
	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].q = -(PI - acos((a_square + b_square - c_square) / (2 * a * b)));
	double alpha = asin(a * (sin(PI - robotModel[LEFT_ARM_ELBOW_FRONT_SWING].q)) / c);
	double beta = -atan2(temp[0], sqrt(temp[1] * temp[1] + temp[2] * temp[2])) + alpha;
	double R0T[3][3];
	R_T3x3(robotModel[MAIN_BODY].R, R0T);
	double result3x3[3][3];
	MatrixMultiMatrix3x3(R0T, robotModel[LEFT_HAND].R, result3x3);
	double Rxq[3][3];
	Rroll(robotModel[LEFT_HAND].q, Rxq);
	double RxTq[3][3];
	R_T3x3(Rxq, RxTq);
	double Ryq[3][3];
	Rpitch(robotModel[LEFT_ARM_ELBOW_FRONT_SWING].q+beta, Ryq);
	double RyTq[3][3];
	R_T3x3(Ryq, RyTq);
	double result3x3_2[3][3];
	MatrixMultiMatrix3x3(result3x3, RxTq, result3x3_2);
	MatrixMultiMatrix3x3(result3x3_2, RyTq, result3x3);
	robotModel[LEFT_ARM_FRONT_SWING].q = atan2(result3x3[0][2],result3x3[2][2]);
	robotModel[LEFT_ARM_SIDE_SWING].q = -atan2(result3x3[2][1],result3x3[2][2]);*/
	double temp_p[3];
	temp_p[0] = robotModel[LEFT_HAND].p[0]; temp_p[1] = robotModel[LEFT_HAND].p[1]; temp_p[2] = robotModel[LEFT_HAND].p[2];
	robotModel[LEFT_ARM_FRONT_SWING].dq = robotModel[LEFT_ARM_FRONT_SWING].q;
	robotModel[LEFT_ARM_SIDE_SWING].dq = robotModel[LEFT_ARM_SIDE_SWING].q;
	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].dq = robotModel[LEFT_ARM_ELBOW_FRONT_SWING].q;
	double R11T[3][3];
	R_T3x3(robotModel[LEFT_HAND].R, R11T);
	double result3x1[3];
	result3x1[0] = robotModel[LEFT_ARM_SIDE_SWING].p[0] - robotModel[LEFT_HAND].p[0];
	result3x1[1] = robotModel[LEFT_ARM_SIDE_SWING].p[1] - robotModel[LEFT_HAND].p[1];
	result3x1[2] = robotModel[LEFT_ARM_SIDE_SWING].p[2] - robotModel[LEFT_HAND].p[2];
	double temp[3];
	MatrixMultiVector3x1(R11T, result3x1, temp);
	double a = robotModel[LEFT_ARM_ELBOW_FRONT_SWING].b[2];
	double b = robotModel[LEFT_HAND].b[2];
	double a_square = a * a;
	double b_square = b * b;
	double c_square = temp[0] * temp[0] + temp[1] * temp[1] + temp[2] * temp[2];
	double c = sqrt(c_square);
	robotModel[LEFT_ARM_SIDE_SWING].q = -atan2(temp[1], temp[2]);
	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].q = -(PI - acos((a_square + b_square - c_square) / (2 * a * b)));
	double alpha = asin(b * (sin(PI - robotModel[LEFT_ARM_ELBOW_FRONT_SWING].q)) / c);
	robotModel[LEFT_ARM_FRONT_SWING].q = (alpha + atan2(temp[0], sqrt(temp[1] * temp[1] + temp[2] * temp[2])));

	//雅可比迭代
	forwardKinematics(MAIN_BODY);
	for (int i = 0; i < 20; i++) {
		double error_p[3];
		error_p[0] = temp_p[0] - robotModel[LEFT_HAND].p[0];
		error_p[1] = temp_p[1] - robotModel[LEFT_HAND].p[1];
		error_p[2] = temp_p[2] - robotModel[LEFT_HAND].p[2];
		double ret = norm((double*)error_p, 1, 3);
		if (ret < 1e-7)
			return;
		double J[3][3];
		double w_a1[3];
		MatrixMultiVector3x1(robotModel[LEFT_ARM_FRONT_SWING].R, robotModel[LEFT_ARM_FRONT_SWING].a, w_a1);
		double w_a2[3];
		MatrixMultiVector3x1(robotModel[LEFT_ARM_SIDE_SWING].R, robotModel[LEFT_ARM_SIDE_SWING].a, w_a2);
		double w_a3[3];
		MatrixMultiVector3x1(robotModel[LEFT_ARM_ELBOW_FRONT_SWING].R, robotModel[LEFT_ARM_ELBOW_FRONT_SWING].a, w_a3);
		double targetSubP[3];
		double cross1[3];
		double cross2[3];
		double cross3[3];
		targetSubP[0] = temp_p[0] - robotModel[LEFT_ARM_SIDE_SWING].p[0];
		targetSubP[1] = temp_p[1] - robotModel[LEFT_ARM_SIDE_SWING].p[1];
		targetSubP[2] = temp_p[2] - robotModel[LEFT_ARM_SIDE_SWING].p[2];
		cross3x1(w_a1, targetSubP, cross1);
		cross3x1(w_a2, targetSubP, cross2);
		targetSubP[0] = temp_p[0] - robotModel[LEFT_ARM_ELBOW_FRONT_SWING].p[0];
		targetSubP[1] = temp_p[1] - robotModel[LEFT_ARM_ELBOW_FRONT_SWING].p[1];
		targetSubP[2] = temp_p[2] - robotModel[LEFT_ARM_ELBOW_FRONT_SWING].p[2];
		cross3x1(w_a3, targetSubP, cross3);
		J[0][0] = cross1[0]; J[0][1] = cross2[0]; J[0][2] = cross3[0];
		J[1][0] = cross1[1]; J[1][1] = cross2[1]; J[1][2] = cross3[1];
		J[2][0] = cross1[2]; J[2][1] = cross2[2]; J[2][2] = cross3[2];
		double J_inv[3][3];
		invMatrix3x3(J, J_inv);
		double dq[3];
		double lamda = 0.5;
		MatrixMultiVector3x1(J_inv, error_p, dq);
		robotModel[LEFT_ARM_FRONT_SWING].q = robotModel[LEFT_ARM_FRONT_SWING].q + lamda * dq[0];
		robotModel[LEFT_ARM_SIDE_SWING].q = robotModel[LEFT_ARM_SIDE_SWING].q + lamda * dq[1];
		robotModel[LEFT_ARM_ELBOW_FRONT_SWING].q = robotModel[LEFT_ARM_ELBOW_FRONT_SWING].q + lamda * dq[2];
		angleLimit();
		forwardKinematics(MAIN_BODY);
	}
	robotModel[LEFT_ARM_FRONT_SWING].dq = -(robotModel[LEFT_ARM_FRONT_SWING].dq-robotModel[LEFT_ARM_FRONT_SWING].q)/frame_T;
	robotModel[LEFT_ARM_SIDE_SWING].dq = -(robotModel[LEFT_ARM_SIDE_SWING].dq-robotModel[LEFT_ARM_SIDE_SWING].q)/frame_T;
	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].dq = -(robotModel[LEFT_ARM_ELBOW_FRONT_SWING].dq-robotModel[LEFT_ARM_ELBOW_FRONT_SWING].q)/frame_T;
	ForwardVelocity(MAIN_BODY);

}

void inverseKinmatics_rightHand() {
	/*double R7T[3][3];
	R_T3x3(robotModel[RIGHT_HAND].R, R7T);
	double result3x1[3];
	result3x1[0] = robotModel[RIGHT_ARM_SIDE_SWING].p[0] - robotModel[RIGHT_HAND].p[0];
	result3x1[1] = robotModel[RIGHT_ARM_SIDE_SWING].p[1] - robotModel[RIGHT_HAND].p[1];
	result3x1[2] = robotModel[RIGHT_ARM_SIDE_SWING].p[2] - robotModel[RIGHT_HAND].p[2];
	double temp[3];
	MatrixMultiVector3x1(R7T, result3x1, temp);
	double a = robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].b[2];
	double b = robotModel[RIGHT_HAND].b[2];
	double a_square = a * a;
	double b_square = b * b;
	double c_square = temp[0] * temp[0] + temp[1] * temp[1] + temp[2] * temp[2];
	double c = sqrt(c_square);
	robotModel[RIGHT_HAND].q = -atan2(temp[1], temp[2]);
	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].q = -(PI - acos((a_square + b_square - c_square) / (2 * a * b)));
	double alpha = asin(a * (sin(PI - robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].q)) / c);
	double beta = -atan2(temp[0], sqrt(temp[1] * temp[1] + temp[2] * temp[2])) + alpha;
	double R0T[3][3];
	R_T3x3(robotModel[MAIN_BODY].R, R0T);
	double result3x3[3][3];
	MatrixMultiMatrix3x3(R0T, robotModel[LEFT_HAND].R, result3x3);
	double Rxq[3][3];
	Rroll(robotModel[RIGHT_HAND].q, Rxq);
	double RxTq[3][3];
	R_T3x3(Rxq, RxTq);
	double Ryq[3][3];
	Rpitch(robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].q + beta, Ryq);
	double RyTq[3][3];
	R_T3x3(Ryq, RyTq);
	double result3x3_2[3][3];
	MatrixMultiMatrix3x3(result3x3, RxTq, result3x3_2);
	MatrixMultiMatrix3x3(result3x3_2, RyTq, result3x3);
	robotModel[RIGHT_ARM_FRONT_SWING].q = atan2(result3x3[0][2], result3x3[2][2]);
	robotModel[RIGHT_ARM_SIDE_SWING].q = -atan2(result3x3[2][1], result3x3[2][2]);*/
	double temp_p[3];
	temp_p[0] = robotModel[RIGHT_HAND].p[0]; temp_p[1] = robotModel[RIGHT_HAND].p[1]; temp_p[2] = robotModel[RIGHT_HAND].p[2];
	robotModel[RIGHT_ARM_FRONT_SWING].dq = robotModel[RIGHT_ARM_FRONT_SWING].q;
	robotModel[RIGHT_ARM_SIDE_SWING].dq = robotModel[RIGHT_ARM_SIDE_SWING].q;
	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].dq = robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].q;
	double R7T[3][3];
	R_T3x3(robotModel[RIGHT_HAND].R, R7T);
	double result3x1[3];
	result3x1[0] = robotModel[RIGHT_ARM_SIDE_SWING].p[0] - robotModel[RIGHT_HAND].p[0];
	result3x1[1] = robotModel[RIGHT_ARM_SIDE_SWING].p[1] - robotModel[RIGHT_HAND].p[1];
	result3x1[2] = robotModel[RIGHT_ARM_SIDE_SWING].p[2] - robotModel[RIGHT_HAND].p[2];
	double temp[3];
	MatrixMultiVector3x1(R7T, result3x1, temp);
	double a = robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].b[2];
	double b = robotModel[RIGHT_HAND].b[2];
	double a_square = a * a;
	double b_square = b * b;
	double c_square = temp[0] * temp[0] + temp[1] * temp[1] + temp[2] * temp[2];
	double c = sqrt(c_square);
	robotModel[RIGHT_ARM_SIDE_SWING].q = -atan2(temp[1], temp[2]);
	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].q = -(PI - acos((a_square + b_square - c_square) / (2 * a * b)));
	double alpha = asin(b * (sin(PI - robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].q)) / c);
	robotModel[RIGHT_ARM_FRONT_SWING].q = (alpha + atan2(temp[0], sqrt(temp[1] * temp[1] + temp[2] * temp[2])));

	//雅可比迭代
	forwardKinematics(MAIN_BODY);
	for (int i = 0; i < 20; i++) {
		double error_p[3];
		error_p[0] = temp_p[0] - robotModel[RIGHT_HAND].p[0];
		error_p[1] = temp_p[1] - robotModel[RIGHT_HAND].p[1];
		error_p[2] = temp_p[2] - robotModel[RIGHT_HAND].p[2];
		double ret = norm((double*)error_p, 1, 3);
		if (ret < 1e-7)
			return;
		double J[3][3];
		double w_a1[3];
		MatrixMultiVector3x1(robotModel[RIGHT_ARM_FRONT_SWING].R, robotModel[RIGHT_ARM_FRONT_SWING].a, w_a1);
		double w_a2[3];
		MatrixMultiVector3x1(robotModel[RIGHT_ARM_SIDE_SWING].R, robotModel[RIGHT_ARM_SIDE_SWING].a, w_a2);
		double w_a3[3];
		MatrixMultiVector3x1(robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].R, robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].a, w_a3);
		double targetSubP[3];
		double cross1[3];
		double cross2[3];
		double cross3[3];
		targetSubP[0] = temp_p[0] - robotModel[RIGHT_ARM_SIDE_SWING].p[0];
		targetSubP[1] = temp_p[1] - robotModel[RIGHT_ARM_SIDE_SWING].p[1];
		targetSubP[2] = temp_p[2] - robotModel[RIGHT_ARM_SIDE_SWING].p[2];
		cross3x1(w_a1, targetSubP, cross1);
		cross3x1(w_a2, targetSubP, cross2);
		targetSubP[0] = temp_p[0] - robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].p[0];
		targetSubP[1] = temp_p[1] - robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].p[1];
		targetSubP[2] = temp_p[2] - robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].p[2];
		cross3x1(w_a3, targetSubP, cross3);
		J[0][0] = cross1[0]; J[0][1] = cross2[0]; J[0][2] = cross3[0];
		J[1][0] = cross1[1]; J[1][1] = cross2[1]; J[1][2] = cross3[1];
		J[2][0] = cross1[2]; J[2][1] = cross2[2]; J[2][2] = cross3[2];
		double J_inv[3][3];
		invMatrix3x3(J, J_inv);
		double dq[3];
		double lamda = 0.5;
		MatrixMultiVector3x1(J_inv, error_p, dq);
		robotModel[RIGHT_ARM_FRONT_SWING].q = robotModel[RIGHT_ARM_FRONT_SWING].q + lamda * dq[0];
		robotModel[RIGHT_ARM_SIDE_SWING].q = robotModel[RIGHT_ARM_SIDE_SWING].q + lamda * dq[1];
		robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].q = robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].q + lamda * dq[2];
		angleLimit();
		forwardKinematics(MAIN_BODY);
	}
	robotModel[RIGHT_ARM_FRONT_SWING].dq = -(robotModel[RIGHT_ARM_FRONT_SWING].dq-robotModel[RIGHT_ARM_FRONT_SWING].q)/frame_T;
	robotModel[RIGHT_ARM_SIDE_SWING].dq = -(robotModel[RIGHT_ARM_SIDE_SWING].dq-robotModel[RIGHT_ARM_SIDE_SWING].q)/frame_T;
	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].dq = -(robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].dq-robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].q)/frame_T;
	ForwardVelocity(MAIN_BODY);
}

void inverseKinmatics_leftFoot(double x, double y, double z) {
	double temp_p[3];
	temp_p[0] = robotModel[LEFT_ANKLE_SIDE_SWING].p[0]; temp_p[1] = robotModel[LEFT_ANKLE_SIDE_SWING].p[1]; temp_p[2] = robotModel[LEFT_ANKLE_SIDE_SWING].p[2];
	robotModel[LEFT_HIP_FRONT_SWING].dq = robotModel[LEFT_HIP_FRONT_SWING].q;
	robotModel[LEFT_HIP_SIDE_SWING].dq = robotModel[LEFT_HIP_SIDE_SWING].q;
	robotModel[LEFT_HIP_ROTATION].dq = robotModel[LEFT_HIP_ROTATION].q;
	robotModel[LEFT_KNEE_FRONT_SWING].dq = robotModel[LEFT_KNEE_FRONT_SWING].q;
	robotModel[LEFT_ANKLE_FRONT_SWING].dq = robotModel[LEFT_ANKLE_FRONT_SWING].q;
	robotModel[LEFT_ANKLE_SIDE_SWING].dq = robotModel[LEFT_ANKLE_SIDE_SWING].q;
	double ref_R[3][3];
	rpy2rot(x, y, z, ref_R); // 这里更新参考位姿值
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			robotModel[LEFT_ANKLE_SIDE_SWING].R[i][j] = ref_R[i][j];
		}
	}
	double R24T[3][3];
	R_T3x3(robotModel[LEFT_ANKLE_SIDE_SWING].R, R24T);
	double result3x1[3];
	result3x1[0] = robotModel[LEFT_HIP_FRONT_SWING].p[0] - robotModel[LEFT_ANKLE_SIDE_SWING].p[0];
	result3x1[1] = robotModel[LEFT_HIP_FRONT_SWING].p[1] - robotModel[LEFT_ANKLE_SIDE_SWING].p[1];
	result3x1[2] = robotModel[LEFT_HIP_FRONT_SWING].p[2] - robotModel[LEFT_ANKLE_SIDE_SWING].p[2];
	double temp[3];
	MatrixMultiVector3x1(R24T, result3x1, temp);
	double a = robotModel[LEFT_KNEE_FRONT_SWING].b[2]+ robotModel[LEFT_HIP_ROTATION].b[2];
	double b = robotModel[LEFT_ANKLE_FRONT_SWING].b[2];
	double a_square = a * a;
	double b_square = b * b;
	double c_square = temp[0] * temp[0] + temp[1] * temp[1] + temp[2] * temp[2];
	double c = sqrt(c_square);
	robotModel[LEFT_ANKLE_SIDE_SWING].q = atan2(temp[1], temp[2]);
	robotModel[LEFT_KNEE_FRONT_SWING].q = (PI - acos((a_square + b_square - c_square) / (2 * a * b)));
	double alpha = -asin(a * (sin(PI - robotModel[LEFT_KNEE_FRONT_SWING].q)) / c);
	robotModel[LEFT_ANKLE_FRONT_SWING].q = -atan2(temp[0], sqrt(temp[1] * temp[1] + temp[2] * temp[2])) - alpha;
	double R0T[3][3];
	R_T3x3(robotModel[MAIN_BODY].R, R0T);
	double result3x3[3][3];
	MatrixMultiMatrix3x3(R0T, robotModel[LEFT_ANKLE_SIDE_SWING].R, result3x3);
	double Rxq[3][3];
	Rroll(robotModel[LEFT_ANKLE_SIDE_SWING].q, Rxq);
	double RxTq[3][3];
	R_T3x3(Rxq, RxTq);
	double Ryq[3][3];
	Rpitch(robotModel[LEFT_KNEE_FRONT_SWING].q + robotModel[LEFT_ANKLE_FRONT_SWING].q, Ryq);
	double RyTq[3][3];
	R_T3x3(Ryq, RyTq);
	double result3x3_2[3][3];
	MatrixMultiMatrix3x3(result3x3, RxTq, result3x3_2);
	MatrixMultiMatrix3x3(result3x3_2, RyTq, result3x3);
	robotModel[LEFT_HIP_ROTATION].q = atan2(-result3x3[0][1], result3x3[1][1]);
	robotModel[LEFT_HIP_SIDE_SWING].q = atan2(result3x3[2][1], -result3x3[0][1]*sin(robotModel[LEFT_HIP_ROTATION].q)+ result3x3[1][1]*cos(robotModel[LEFT_HIP_ROTATION].q));
	robotModel[LEFT_HIP_FRONT_SWING].q = atan2(-result3x3[2][0], result3x3[2][2]);

	//雅可比迭代
	forwardKinematics(MAIN_BODY);
	for (int i = 0; i < 20; i++) {
		double error_p[6];
		double Rerr[3][3];
		double now_R_inv[3][3];
		invMatrix3x3(robotModel[LEFT_ANKLE_SIDE_SWING].R, now_R_inv);
		MatrixMultiMatrix3x3(now_R_inv, ref_R, Rerr);
		double omega[3];
		rot2omega(Rerr, omega);
		double werr[3];
		MatrixMultiVector3x1(robotModel[LEFT_ANKLE_SIDE_SWING].R, omega, werr);
		error_p[0] = temp_p[0] - robotModel[LEFT_ANKLE_SIDE_SWING].p[0];
		error_p[1] = temp_p[1] - robotModel[LEFT_ANKLE_SIDE_SWING].p[1];
		error_p[2] = temp_p[2] - robotModel[LEFT_ANKLE_SIDE_SWING].p[2];
		error_p[3] = werr[0];
		error_p[4] = werr[1];
		error_p[5] = werr[2];
		double ret = norm((double*)error_p, 1, 6);
		if (ret < 1e-7)
			return;
		double J[6][6];
		double w_a1[3];
		MatrixMultiVector3x1(robotModel[LEFT_HIP_FRONT_SWING].R, robotModel[LEFT_HIP_FRONT_SWING].a, w_a1);
		double w_a2[3];
		MatrixMultiVector3x1(robotModel[LEFT_HIP_SIDE_SWING].R, robotModel[LEFT_HIP_SIDE_SWING].a, w_a2);
		double w_a3[3];
		MatrixMultiVector3x1(robotModel[LEFT_HIP_ROTATION].R, robotModel[LEFT_HIP_ROTATION].a, w_a3);
		double w_a4[3];
		MatrixMultiVector3x1(robotModel[LEFT_KNEE_FRONT_SWING].R, robotModel[LEFT_KNEE_FRONT_SWING].a, w_a4);
		double w_a5[3];
		MatrixMultiVector3x1(robotModel[LEFT_ANKLE_FRONT_SWING].R, robotModel[LEFT_ANKLE_FRONT_SWING].a, w_a5);
		double w_a6[3];
		MatrixMultiVector3x1(robotModel[LEFT_ANKLE_SIDE_SWING].R, robotModel[LEFT_ANKLE_SIDE_SWING].a, w_a6);
		double targetSubP[3];
		double cross1[3];
		double cross2[3];
		double cross3[3];
		double cross4[3];
		double cross5[3];
		double cross6[3];
		targetSubP[0] = temp_p[0] - robotModel[LEFT_HIP_FRONT_SWING].p[0];
		targetSubP[1] = temp_p[1] - robotModel[LEFT_HIP_FRONT_SWING].p[1];
		targetSubP[2] = temp_p[2] - robotModel[LEFT_HIP_FRONT_SWING].p[2];
		cross3x1(w_a1, targetSubP, cross1);
		targetSubP[0] = temp_p[0] - robotModel[LEFT_HIP_SIDE_SWING].p[0];
		targetSubP[1] = temp_p[1] - robotModel[LEFT_HIP_SIDE_SWING].p[1];
		targetSubP[2] = temp_p[2] - robotModel[LEFT_HIP_SIDE_SWING].p[2];
		cross3x1(w_a2, targetSubP, cross2);
		targetSubP[0] = temp_p[0] - robotModel[LEFT_HIP_ROTATION].p[0];
		targetSubP[1] = temp_p[1] - robotModel[LEFT_HIP_ROTATION].p[1];
		targetSubP[2] = temp_p[2] - robotModel[LEFT_HIP_ROTATION].p[2];
		cross3x1(w_a3, targetSubP, cross3);
		targetSubP[0] = temp_p[0] - robotModel[LEFT_KNEE_FRONT_SWING].p[0];
		targetSubP[1] = temp_p[1] - robotModel[LEFT_KNEE_FRONT_SWING].p[1];
		targetSubP[2] = temp_p[2] - robotModel[LEFT_KNEE_FRONT_SWING].p[2];
		cross3x1(w_a4, targetSubP, cross4);
		targetSubP[0] = temp_p[0] - robotModel[LEFT_ANKLE_FRONT_SWING].p[0];
		targetSubP[1] = temp_p[1] - robotModel[LEFT_ANKLE_FRONT_SWING].p[1];
		targetSubP[2] = temp_p[2] - robotModel[LEFT_ANKLE_FRONT_SWING].p[2];
		cross3x1(w_a5, targetSubP, cross5);
		targetSubP[0] = temp_p[0] - robotModel[LEFT_ANKLE_SIDE_SWING].p[0];
		targetSubP[1] = temp_p[1] - robotModel[LEFT_ANKLE_SIDE_SWING].p[1];
		targetSubP[2] = temp_p[2] - robotModel[LEFT_ANKLE_SIDE_SWING].p[2];
		cross3x1(w_a6, targetSubP, cross6);
		J[0][0] = cross1[0]; J[0][1] = cross2[0]; J[0][2] = cross3[0]; J[0][3] = cross4[0]; J[0][4] = cross5[0]; J[0][5] = cross6[0];
		J[1][0] = cross1[1]; J[1][1] = cross2[1]; J[1][2] = cross3[1]; J[1][3] = cross4[1]; J[1][4] = cross5[1]; J[1][5] = cross6[1];
		J[2][0] = cross1[2]; J[2][1] = cross2[2]; J[2][2] = cross3[2]; J[2][3] = cross4[2]; J[2][4] = cross5[2]; J[2][5] = cross6[2];
		J[3][0] = w_a1[0];   J[3][1] = w_a2[0];   J[3][2] = w_a3[0];   J[3][3] = w_a4[0];   J[3][4] = w_a5[0];   J[3][5] = w_a6[0];
		J[4][0] = w_a1[1];   J[4][1] = w_a2[1];   J[4][2] = w_a3[1];   J[4][3] = w_a4[1];   J[4][4] = w_a5[1];   J[4][5] = w_a6[1];
		J[5][0] = w_a1[2];   J[5][1] = w_a2[2];   J[5][2] = w_a3[2];   J[5][3] = w_a4[2];   J[5][4] = w_a5[2];   J[5][5] = w_a6[2];
		double J_inv[6][6];
		double matlab_result[36];
		matlab_inv((double*)J, matlab_result);
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 6; j++) {
				J_inv[i][j] = matlab_result[i * 6 + j];
			}
		}
		double dq[6];
		double lamda = 0.5;
		MatrixMultiVector6x1(J_inv, error_p, dq);
		robotModel[LEFT_HIP_FRONT_SWING].q = robotModel[LEFT_HIP_FRONT_SWING].q + lamda * dq[0];
		robotModel[LEFT_HIP_SIDE_SWING].q = robotModel[LEFT_HIP_SIDE_SWING].q + lamda * dq[1];
		robotModel[LEFT_HIP_ROTATION].q = robotModel[LEFT_HIP_ROTATION].q + lamda * dq[2];
		robotModel[LEFT_KNEE_FRONT_SWING].q = robotModel[LEFT_KNEE_FRONT_SWING].q + lamda * dq[3];
		robotModel[LEFT_ANKLE_FRONT_SWING].q = robotModel[LEFT_ANKLE_FRONT_SWING].q + lamda * dq[4];
		robotModel[LEFT_ANKLE_SIDE_SWING].q = robotModel[LEFT_ANKLE_SIDE_SWING].q + lamda * dq[5];
		forwardKinematics(MAIN_BODY);
	}
	robotModel[LEFT_HIP_FRONT_SWING].dq = -(robotModel[LEFT_HIP_FRONT_SWING].dq-robotModel[LEFT_HIP_FRONT_SWING].q)/frame_T;
	robotModel[LEFT_HIP_SIDE_SWING].dq = -(robotModel[LEFT_HIP_SIDE_SWING].dq-robotModel[LEFT_HIP_SIDE_SWING].q)/frame_T;
	robotModel[LEFT_HIP_ROTATION].dq = -(robotModel[LEFT_HIP_ROTATION].dq-robotModel[LEFT_HIP_ROTATION].q)/frame_T;
	robotModel[LEFT_KNEE_FRONT_SWING].dq = -(robotModel[LEFT_KNEE_FRONT_SWING].dq-robotModel[LEFT_KNEE_FRONT_SWING].q)/frame_T;
	robotModel[LEFT_ANKLE_FRONT_SWING].dq = -(robotModel[LEFT_ANKLE_FRONT_SWING].dq-robotModel[LEFT_ANKLE_FRONT_SWING].q)/frame_T;
	robotModel[LEFT_ANKLE_SIDE_SWING].dq = -(robotModel[LEFT_ANKLE_SIDE_SWING].dq-robotModel[LEFT_ANKLE_SIDE_SWING].q)/frame_T;
	ForwardVelocity(MAIN_BODY);
}

void inverseKinmatics_rightFoot(double x, double y, double z) {
	double temp_p[3];
	temp_p[0] = robotModel[RIGHT_ANKLE_SIDE_SWING].p[0]; temp_p[1] = robotModel[RIGHT_ANKLE_SIDE_SWING].p[1]; temp_p[2] = robotModel[RIGHT_ANKLE_SIDE_SWING].p[2];
	robotModel[RIGHT_HIP_FRONT_SWING].dq = robotModel[RIGHT_HIP_FRONT_SWING].q;
	robotModel[RIGHT_HIP_SIDE_SWING].dq = robotModel[RIGHT_HIP_SIDE_SWING].q;
	robotModel[RIGHT_HIP_ROTATION].dq = robotModel[RIGHT_HIP_ROTATION].q;
	robotModel[RIGHT_KNEE_FRONT_SWING].dq = robotModel[RIGHT_KNEE_FRONT_SWING].q;
	robotModel[RIGHT_ANKLE_FRONT_SWING].dq = robotModel[RIGHT_ANKLE_FRONT_SWING].q;
	robotModel[RIGHT_ANKLE_SIDE_SWING].dq = robotModel[RIGHT_ANKLE_SIDE_SWING].q;
	double ref_R[3][3];
	rpy2rot(x, y, z, ref_R); // 这里更新参考位姿值
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			robotModel[RIGHT_ANKLE_SIDE_SWING].R[i][j] = ref_R[i][j];
		}
	}
	double R17T[3][3];
	R_T3x3(robotModel[RIGHT_ANKLE_SIDE_SWING].R, R17T);
	double result3x1[3];
	result3x1[0] = robotModel[RIGHT_HIP_FRONT_SWING].p[0] - robotModel[RIGHT_ANKLE_SIDE_SWING].p[0];
	result3x1[1] = robotModel[RIGHT_HIP_FRONT_SWING].p[1] - robotModel[RIGHT_ANKLE_SIDE_SWING].p[1];
	result3x1[2] = robotModel[RIGHT_HIP_FRONT_SWING].p[2] - robotModel[RIGHT_ANKLE_SIDE_SWING].p[2];
	double temp[3];
	MatrixMultiVector3x1(R17T, result3x1, temp);
	double a = robotModel[RIGHT_KNEE_FRONT_SWING].b[2] + robotModel[RIGHT_HIP_ROTATION].b[2];
	double b = robotModel[RIGHT_ANKLE_FRONT_SWING].b[2];
	double a_square = a * a;
	double b_square = b * b;
	double c_square = temp[0] * temp[0] + temp[1] * temp[1] + temp[2] * temp[2];
	double c = sqrt(c_square);
	robotModel[RIGHT_ANKLE_SIDE_SWING].q = atan2(temp[1], temp[2]);
	robotModel[RIGHT_KNEE_FRONT_SWING].q = (PI - acos((a_square + b_square - c_square) / (2 * a * b)));
	double alpha = -asin(a * (sin(PI - robotModel[RIGHT_KNEE_FRONT_SWING].q)) / c);
	robotModel[RIGHT_ANKLE_FRONT_SWING].q = -atan2(temp[0], sqrt(temp[1] * temp[1] + temp[2] * temp[2])) - alpha;
	double R0T[3][3];
	R_T3x3(robotModel[MAIN_BODY].R, R0T);
	double result3x3[3][3];
	MatrixMultiMatrix3x3(R0T, robotModel[RIGHT_ANKLE_SIDE_SWING].R, result3x3);
	double Rxq[3][3];
	Rroll(robotModel[RIGHT_ANKLE_SIDE_SWING].q, Rxq);
	double RxTq[3][3];
	R_T3x3(Rxq, RxTq);
	double Ryq[3][3];
	Rpitch(robotModel[RIGHT_KNEE_FRONT_SWING].q + robotModel[RIGHT_ANKLE_FRONT_SWING].q, Ryq);
	double RyTq[3][3];
	R_T3x3(Ryq, RyTq);
	double result3x3_2[3][3];
	MatrixMultiMatrix3x3(result3x3, RxTq, result3x3_2);
	MatrixMultiMatrix3x3(result3x3_2, RyTq, result3x3);
	robotModel[RIGHT_HIP_ROTATION].q = atan2(-result3x3[0][1], result3x3[1][1]);
	robotModel[RIGHT_HIP_SIDE_SWING].q = atan2(result3x3[2][1], -result3x3[0][1] * sin(robotModel[RIGHT_HIP_ROTATION].q) + result3x3[1][1] * cos(robotModel[RIGHT_HIP_ROTATION].q));
	robotModel[RIGHT_HIP_FRONT_SWING].q = atan2(-result3x3[2][0], result3x3[2][2]);

	//雅可比迭代
	forwardKinematics(MAIN_BODY);
	for (int i = 0; i < 20; i++) {
		double error_p[6];
		double Rerr[3][3];
		double now_R_inv[3][3];	
		invMatrix3x3(robotModel[RIGHT_ANKLE_SIDE_SWING].R, now_R_inv);
		MatrixMultiMatrix3x3(now_R_inv, ref_R, Rerr);
		double omega[3];
		rot2omega(Rerr, omega);
		double werr[3];
		MatrixMultiVector3x1(robotModel[RIGHT_ANKLE_SIDE_SWING].R, omega, werr);
		error_p[0] = temp_p[0] - robotModel[RIGHT_ANKLE_SIDE_SWING].p[0];
		error_p[1] = temp_p[1] - robotModel[RIGHT_ANKLE_SIDE_SWING].p[1];
		error_p[2] = temp_p[2] - robotModel[RIGHT_ANKLE_SIDE_SWING].p[2];
		error_p[3] = werr[0];
		error_p[4] = werr[1];
		error_p[5] = werr[2];
		double ret = norm((double*)error_p, 1, 6);
		if (ret < 1e-7)
			return;
		double J[6][6];
		double w_a1[3];
		MatrixMultiVector3x1(robotModel[RIGHT_HIP_FRONT_SWING].R, robotModel[RIGHT_HIP_FRONT_SWING].a, w_a1);
		double w_a2[3];
		MatrixMultiVector3x1(robotModel[RIGHT_HIP_SIDE_SWING].R, robotModel[RIGHT_HIP_SIDE_SWING].a, w_a2);
		double w_a3[3];
		MatrixMultiVector3x1(robotModel[RIGHT_HIP_ROTATION].R, robotModel[RIGHT_HIP_ROTATION].a, w_a3);
		double w_a4[3];
		MatrixMultiVector3x1(robotModel[RIGHT_KNEE_FRONT_SWING].R, robotModel[RIGHT_KNEE_FRONT_SWING].a, w_a4);
		double w_a5[3];
		MatrixMultiVector3x1(robotModel[RIGHT_ANKLE_FRONT_SWING].R, robotModel[RIGHT_ANKLE_FRONT_SWING].a, w_a5);
		double w_a6[3];
		MatrixMultiVector3x1(robotModel[RIGHT_ANKLE_SIDE_SWING].R, robotModel[RIGHT_ANKLE_SIDE_SWING].a, w_a6);
		double targetSubP[3];
		double cross1[3];
		double cross2[3];
		double cross3[3];
		double cross4[3];
		double cross5[3];
		double cross6[3];
		targetSubP[0] = temp_p[0] - robotModel[RIGHT_HIP_FRONT_SWING].p[0];
		targetSubP[1] = temp_p[1] - robotModel[RIGHT_HIP_FRONT_SWING].p[1];
		targetSubP[2] = temp_p[2] - robotModel[RIGHT_HIP_FRONT_SWING].p[2];
		cross3x1(w_a1, targetSubP, cross1);
		targetSubP[0] = temp_p[0] - robotModel[RIGHT_HIP_SIDE_SWING].p[0];
		targetSubP[1] = temp_p[1] - robotModel[RIGHT_HIP_SIDE_SWING].p[1];
		targetSubP[2] = temp_p[2] - robotModel[RIGHT_HIP_SIDE_SWING].p[2];
		cross3x1(w_a2, targetSubP, cross2);
		targetSubP[0] = temp_p[0] - robotModel[RIGHT_HIP_ROTATION].p[0];
		targetSubP[1] = temp_p[1] - robotModel[RIGHT_HIP_ROTATION].p[1];
		targetSubP[2] = temp_p[2] - robotModel[RIGHT_HIP_ROTATION].p[2];
		cross3x1(w_a3, targetSubP, cross3);
		targetSubP[0] = temp_p[0] - robotModel[RIGHT_KNEE_FRONT_SWING].p[0];
		targetSubP[1] = temp_p[1] - robotModel[RIGHT_KNEE_FRONT_SWING].p[1];
		targetSubP[2] = temp_p[2] - robotModel[RIGHT_KNEE_FRONT_SWING].p[2];
		cross3x1(w_a4, targetSubP, cross4);
		targetSubP[0] = temp_p[0] - robotModel[RIGHT_ANKLE_FRONT_SWING].p[0];
		targetSubP[1] = temp_p[1] - robotModel[RIGHT_ANKLE_FRONT_SWING].p[1];
		targetSubP[2] = temp_p[2] - robotModel[RIGHT_ANKLE_FRONT_SWING].p[2];
		cross3x1(w_a5, targetSubP, cross5);
		targetSubP[0] = temp_p[0] - robotModel[RIGHT_ANKLE_SIDE_SWING].p[0];
		targetSubP[1] = temp_p[1] - robotModel[RIGHT_ANKLE_SIDE_SWING].p[1];
		targetSubP[2] = temp_p[2] - robotModel[RIGHT_ANKLE_SIDE_SWING].p[2];
		cross3x1(w_a6, targetSubP, cross6);
		J[0][0] = cross1[0]; J[0][1] = cross2[0]; J[0][2] = cross3[0]; J[0][3] = cross4[0]; J[0][4] = cross5[0]; J[0][5] = cross6[0];
		J[1][0] = cross1[1]; J[1][1] = cross2[1]; J[1][2] = cross3[1]; J[1][3] = cross4[1]; J[1][4] = cross5[1]; J[1][5] = cross6[1];
		J[2][0] = cross1[2]; J[2][1] = cross2[2]; J[2][2] = cross3[2]; J[2][3] = cross4[2]; J[2][4] = cross5[2]; J[2][5] = cross6[2];
		J[3][0] = w_a1[0];   J[3][1] = w_a2[0];   J[3][2] = w_a3[0];   J[3][3] = w_a4[0];   J[3][4] = w_a5[0];   J[3][5] = w_a6[0];
		J[4][0] = w_a1[1];   J[4][1] = w_a2[1];   J[4][2] = w_a3[1];   J[4][3] = w_a4[1];   J[4][4] = w_a5[1];   J[4][5] = w_a6[1];
		J[5][0] = w_a1[2];   J[5][1] = w_a2[2];   J[5][2] = w_a3[2];   J[5][3] = w_a4[2];   J[5][4] = w_a5[2];   J[5][5] = w_a6[2];
		double J_inv[6][6];
		double matlab_result[36];
		matlab_inv((double*)J, matlab_result);
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 6; j++) {
				J_inv[i][j] = matlab_result[i * 6 + j];
			}
		}
		double dq[6];
		double lamda = 0.5;
		MatrixMultiVector6x1(J_inv, error_p, dq);
		robotModel[RIGHT_HIP_FRONT_SWING].q = robotModel[RIGHT_HIP_FRONT_SWING].q + lamda * dq[0];
		robotModel[RIGHT_HIP_SIDE_SWING].q = robotModel[RIGHT_HIP_SIDE_SWING].q + lamda * dq[1];
		robotModel[RIGHT_HIP_ROTATION].q = robotModel[RIGHT_HIP_ROTATION].q + lamda * dq[2];
		robotModel[RIGHT_KNEE_FRONT_SWING].q = robotModel[RIGHT_KNEE_FRONT_SWING].q + lamda * dq[3];
		robotModel[RIGHT_ANKLE_FRONT_SWING].q = robotModel[RIGHT_ANKLE_FRONT_SWING].q + lamda * dq[4];
		robotModel[RIGHT_ANKLE_SIDE_SWING].q = robotModel[RIGHT_ANKLE_SIDE_SWING].q + lamda * dq[5];
		forwardKinematics(MAIN_BODY);
	}
	robotModel[RIGHT_HIP_FRONT_SWING].dq = -(robotModel[RIGHT_HIP_FRONT_SWING].dq-robotModel[RIGHT_HIP_FRONT_SWING].q)/frame_T;
	robotModel[RIGHT_HIP_SIDE_SWING].dq = -(robotModel[RIGHT_HIP_SIDE_SWING].dq-robotModel[RIGHT_HIP_SIDE_SWING].q)/frame_T;
	robotModel[RIGHT_HIP_ROTATION].dq = -(robotModel[RIGHT_HIP_ROTATION].dq-robotModel[RIGHT_HIP_ROTATION].q)/frame_T;
	robotModel[RIGHT_KNEE_FRONT_SWING].dq = -(robotModel[RIGHT_KNEE_FRONT_SWING].dq-robotModel[RIGHT_KNEE_FRONT_SWING].q)/frame_T;
	robotModel[RIGHT_ANKLE_FRONT_SWING].dq = -(robotModel[RIGHT_ANKLE_FRONT_SWING].dq-robotModel[RIGHT_ANKLE_FRONT_SWING].q)/frame_T;
	robotModel[RIGHT_ANKLE_SIDE_SWING].dq = -(robotModel[RIGHT_ANKLE_SIDE_SWING].dq-robotModel[RIGHT_ANKLE_SIDE_SWING].q)/frame_T;
	ForwardVelocity(MAIN_BODY);
}

void clearTxt()
{
	FILE* fp;
	char ch[200];
	char filename[] = "/home/wp/ikid_ws/MOS2023.txt";
	fp = fopen(filename, "w");
	if (fp == NULL)
	{
		exit(0);
	}
	fclose(fp);
	return;
}

void writeTxt() {
	FILE* fp = NULL;
	char ch[200];
	char filename[] = "/home/wp/ikid_ws/MOS2023.txt";
	fp = fopen(filename, "a");
	if(fp == NULL)
	{
		exit(0);
	}
	for (int i = 0; i < PART_NUMBER; i++)
	{
		sprintf(ch, "%d,%s,%d,%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
			robotModel[i].linkID, robotModel[i].name, robotModel[i].sister, robotModel[i].child, robotModel[i].mother,
			robotModel[i].p[0], robotModel[i].p[1], robotModel[i].p[2],
			robotModel[i].R[0][0], robotModel[i].R[0][1], robotModel[i].R[0][2],
			robotModel[i].R[1][0], robotModel[i].R[1][1], robotModel[i].R[1][2],
			robotModel[i].R[2][0], robotModel[i].R[2][1], robotModel[i].R[2][2], robotModel[i].q, 
			current_ZMP_point[0], current_ZMP_point[1], current_ZMP_point[2]);
		fputs(ch, fp);
	}
	fclose(fp);
	/*double temp_com[3] = { 0 };
	Calc_com(temp_com);
	error_Com_ZmpCom[0] = fabs(temp_com[0] - Com[0]);
	error_Com_ZmpCom[1] = fabs(temp_com[1] - Com[1]);
	error_Com_ZmpCom[2] = fabs(temp_com[2] - Com[2]);
	sum_error_Com_ZmpCom += error_Com_ZmpCom[0] + error_Com_ZmpCom[1]+ error_Com_ZmpCom[2];
	printf("error_Com_ZmpCom x: %f\n", error_Com_ZmpCom[0]);
	printf("error_Com_ZmpCom y: %f\n", error_Com_ZmpCom[1]);
	printf("error_Com_ZmpCom z: %f\n", error_Com_ZmpCom[2]);
	printf("sum error_Com_ZmpCom : %f\n", sum_error_Com_ZmpCom);*/
	return;
}


void waistPosition(double x1, double y1, 
				double x2, double y2, 
				double x3, double y3, 
				double x4, double y4) {   // 实际使用时会改，利用全身质心的位置来牵引腰部的位置
	double k1,k2,x5,y5;
	k1 = (x1 - x2) / (y1 - y2);
	k2 = (x3 - x4) / (y3 - y4);
	if (fabs(k1 - k2) < 10e-3) return;
	y5 = ((x3 - x1) + k1 * y1 - k2 * y3) / (k1 - k2);
	x5 = k2 * (y5 - y3) + x3;
	robotModel[MAIN_BODY].p[0] = x5;
	robotModel[MAIN_BODY].p[1] = y5;

}

void waistPosition_com(double r,double p,double y, int current_frame_count) {   // 利用全身质心的位置来牵引腰部的位置
	CalcTrajectory_Com(current_frame_count);
	double R[3][3];
	rpy2rot(r, p, y, R);
	double v[3] = {0};
	v[0] = robotModel[MAIN_BODY].p[0];
	v[1] = robotModel[MAIN_BODY].p[1];
	robotModel[MAIN_BODY].p[0] = Com[0] - PC_MAIN_BODY[0];
	robotModel[MAIN_BODY].p[1] = Com[1] - PC_MAIN_BODY[1];
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++) {
			robotModel[MAIN_BODY].R[i][j] = R[i][j];
		}
	}
	v[0] = (robotModel[MAIN_BODY].p[0] - v[0]) / frame_T;
	v[1] = (robotModel[MAIN_BODY].p[1] - v[1]) / frame_T;
	robotModel[MAIN_BODY].v[0] = v[0];
	robotModel[MAIN_BODY].v[1] = v[1];
}

void startTrajPlan(){
	// 启动步
	ikid_start_walk_flag = true;
	support_ZMP[0] = (robotModel[RIGHT_FOOT].p[0]+robotModel[LEFT_FOOT].p[0])/2;
	support_ZMP[1] = (robotModel[RIGHT_FOOT].p[1]+robotModel[LEFT_FOOT].p[1])/2;
	if(isLeft){
		pn[0] = robotModel[RIGHT_FOOT].p[0];
		pn[1] = robotModel[RIGHT_FOOT].p[1];
	}else{
		pn[0] = robotModel[LEFT_FOOT].p[0];
		pn[1] = robotModel[LEFT_FOOT].p[1];
	}
	dFootSupportPhase(theta,theta,theta);
	support_ZMP[0] = pn[0];
	support_ZMP[1] = pn[1];
	ikid_start_walk_flag = false;
}

void trajPlan() {
	// 第n步
	pn[0] = pn[0] + cos(theta) * sx + (-sin(theta) * sy * (-1) * pow(-1, isLeft));
	pn[1] = pn[1] + sin(theta) * sx + (cos(theta) * sy * (-1) * pow(-1, isLeft));
	isDsPhase = false;
	if (isLeft) {
		//确定轨迹的三点用PQR表示
		// 此时右脚左手位置固定，记录位置
		double solid_right_foot[3] = { robotModel[RIGHT_FOOT].p[0],robotModel[RIGHT_FOOT].p[1],robotModel[RIGHT_FOOT].p[2] };
		double solid_left_hand[3] = { robotModel[LEFT_HAND].p[0],robotModel[LEFT_HAND].p[1],robotModel[LEFT_HAND].p[2] };
		
		// 左脚
		double P[3] = { robotModel[LEFT_FOOT].p[0], robotModel[LEFT_FOOT].p[1], 0 };
		double Q[3] = { (robotModel[LEFT_FOOT].p[0] + pn[0]) / 2, (robotModel[LEFT_FOOT].p[1] + pn[1]) / 2, fh };
		double R[3] = { pn[0], pn[1], 0 };
		double PQ[3] = { Q[0] - P[0], Q[1] - P[1], Q[2] - P[2] };
		double QR[3] = { R[0] - Q[0], R[1] - Q[1], R[2] - Q[2] };
		double n1[3];
		cross3x1(PQ, QR, n1);
		// 三点平面原点C
		double C[3] = { Q[0], Q[1], 0 };
		// x轴单位向量i_prime
		double CP_norm = sqrt((C[0] - P[0]) * (C[0] - P[0]) + (C[1] - P[1]) * (C[1] - P[1]) + (C[2] - P[2]) * (C[2] - P[2]));
		double i_prime[3] = { (P[0] - C[0]) / CP_norm, (P[1] - C[1]) / CP_norm, (P[2] - C[2]) / CP_norm };
		// z轴单位向量k_prime
		double n1_morm = sqrt(n1[0] * n1[0] + n1[1] * n1[1] + n1[2] * n1[2]);
		double k_prime[3] = { n1[0] / n1_morm, n1[1] / n1_morm, n1[2] / n1_morm };
		// y轴单位向量j_prime
		double j_prime[3];
		cross3x1(k_prime, i_prime, j_prime);
		//面到世界坐标系转换矩阵T
		double T[3][3];
		T[0][0] = i_prime[0]; T[0][1] = j_prime[0]; T[0][2] = k_prime[0];
		T[1][0] = i_prime[1]; T[1][1] = j_prime[1]; T[1][2] = k_prime[1];
		T[2][0] = i_prime[2]; T[2][1] = j_prime[2]; T[2][2] = k_prime[2];
		double T_T[3][3];
		invMatrix3x3(T, T_T);

		bool walk_with_ball;
        ros::param::get("walk_with_ball",walk_with_ball);
		double quintic_A[6][4] = {};
		double quintic_B[6][4] = {};
		double three_spline_A[8][2] = {};
		if(walk_with_ball){
			// 求解四段五次多项式插值参数
			quinticPolyInterFour(quintic_A, quintic_B, CP_norm*2);
		}else{
			// 求解两段五次多项式插值参数
			// quinticPolyInterTwo(quintic_A, quintic_B, CP_norm*2);

			// 求解两段三次样条插值参数
			threeSplineInter(three_spline_A,CP_norm*2);
		}
		


#if SWING_ARM
		// 右臂
		double current_arm_angle_right = robotModel[RIGHT_ARM_FRONT_SWING].q;
		double current_arm_angle_left = robotModel[LEFT_ARM_FRONT_SWING].q;
		double arm_abc[3] = {0};
		double arm_length = 0;
		arm_abc[0] = robotModel[RIGHT_ARM_FRONT_SWING].p[0] - robotModel[RIGHT_HAND].p[0];
		arm_abc[1] = robotModel[RIGHT_ARM_FRONT_SWING].p[1] - robotModel[RIGHT_HAND].p[1];
		arm_abc[2] = robotModel[RIGHT_ARM_FRONT_SWING].p[2] - robotModel[RIGHT_HAND].p[2];
		arm_length = norm(arm_abc, 1, 3);
		arm_swing_angle = asin(sx/2 / arm_length);

#endif
		
		for (int i = 0; i < step_basic_frame; i++)
		{
			double world_p[3];
			double local_p[3];
			double x;
			double y;
			// 决策是否动态踢球 开始
			if(walk_with_ball){
				//{0, step_basic_frame/5*2*frame_T, step_basic_frame/5*3*frame_T, step_basic_frame/5*4*frame_T, step_basic_frame*frame_T}
				if(i < step_basic_frame/5*2){
					double temp_t = i*frame_T;
					x = quintic_A[0][0]+quintic_A[1][0]*temp_t+quintic_A[2][0]*pow(temp_t,2)+quintic_A[3][0]*pow(temp_t,3)+quintic_A[4][0]*pow(temp_t,4)+quintic_A[5][0]*pow(temp_t,5);
					y = quintic_B[0][0]+quintic_B[1][0]*temp_t+quintic_B[2][0]*pow(temp_t,2)+quintic_B[3][0]*pow(temp_t,3)+quintic_B[4][0]*pow(temp_t,4)+quintic_B[5][0]*pow(temp_t,5);
				}else if(i < step_basic_frame/5*3){
					double temp_t = i*frame_T;
					x = quintic_A[0][1]+quintic_A[1][1]*temp_t+quintic_A[2][1]*pow(temp_t,2)+quintic_A[3][1]*pow(temp_t,3)+quintic_A[4][1]*pow(temp_t,4)+quintic_A[5][1]*pow(temp_t,5);
					y = quintic_B[0][1]+quintic_B[1][1]*temp_t+quintic_B[2][1]*pow(temp_t,2)+quintic_B[3][1]*pow(temp_t,3)+quintic_B[4][1]*pow(temp_t,4)+quintic_B[5][1]*pow(temp_t,5);
				}else if(i < step_basic_frame/5*4){
					double temp_t = i*frame_T;
					x = quintic_A[0][2]+quintic_A[1][2]*temp_t+quintic_A[2][2]*pow(temp_t,2)+quintic_A[3][2]*pow(temp_t,3)+quintic_A[4][2]*pow(temp_t,4)+quintic_A[5][2]*pow(temp_t,5);
					y = quintic_B[0][2]+quintic_B[1][2]*temp_t+quintic_B[2][2]*pow(temp_t,2)+quintic_B[3][2]*pow(temp_t,3)+quintic_B[4][2]*pow(temp_t,4)+quintic_B[5][2]*pow(temp_t,5);
				}else{
					double temp_t = i*frame_T;
					x = quintic_A[0][3]+quintic_A[1][3]*temp_t+quintic_A[2][3]*pow(temp_t,2)+quintic_A[3][3]*pow(temp_t,3)+quintic_A[4][3]*pow(temp_t,4)+quintic_A[5][3]*pow(temp_t,5);
					y = quintic_B[0][3]+quintic_B[1][3]*temp_t+quintic_B[2][3]*pow(temp_t,2)+quintic_B[3][3]*pow(temp_t,3)+quintic_B[4][3]*pow(temp_t,4)+quintic_B[5][3]*pow(temp_t,5);
				}
				//printf(" x:%f,y:%f ", x, y);
			}else{
				//  x = CP_norm + -(i + 1) * 2 * CP_norm / step_basic_frame; // sin曲线
				//  y = fh * cos(PI / (2 * CP_norm) * x);
				// if(i < step_basic_frame/2){
				// 	double temp_t = i*frame_T;
				// 	x = quintic_A[0][0]+quintic_A[1][0]*temp_t+quintic_A[2][0]*pow(temp_t,2)+quintic_A[3][0]*pow(temp_t,3)+quintic_A[4][0]*pow(temp_t,4)+quintic_A[5][0]*pow(temp_t,5);
				// 	y = quintic_B[0][0]+quintic_B[1][0]*temp_t+quintic_B[2][0]*pow(temp_t,2)+quintic_B[3][0]*pow(temp_t,3)+quintic_B[4][0]*pow(temp_t,4)+quintic_B[5][0]*pow(temp_t,5);
				// }else{
				// 	double temp_t = i*frame_T;
				// 	x = quintic_A[0][1]+quintic_A[1][1]*temp_t+quintic_A[2][1]*pow(temp_t,2)+quintic_A[3][1]*pow(temp_t,3)+quintic_A[4][1]*pow(temp_t,4)+quintic_A[5][1]*pow(temp_t,5);
				// 	y = quintic_B[0][1]+quintic_B[1][1]*temp_t+quintic_B[2][1]*pow(temp_t,2)+quintic_B[3][1]*pow(temp_t,3)+quintic_B[4][1]*pow(temp_t,4)+quintic_B[5][1]*pow(temp_t,5);
				// }
				if(i < step_basic_frame/2){
					double temp_t = i*frame_T;
					x = three_spline_A[0][0]+three_spline_A[1][0]*temp_t+three_spline_A[2][0]*pow(temp_t,2)+three_spline_A[3][0]*pow(temp_t,3);
					y = three_spline_A[0][1]+three_spline_A[1][1]*temp_t+three_spline_A[2][1]*pow(temp_t,2)+three_spline_A[3][1]*pow(temp_t,3);
				}else{
					double temp_t = i*frame_T;
					x = three_spline_A[4][0]+three_spline_A[5][0]*temp_t+three_spline_A[6][0]*pow(temp_t,2)+three_spline_A[7][0]*pow(temp_t,3);
					y = three_spline_A[4][1]+three_spline_A[5][1]*temp_t+three_spline_A[6][1]*pow(temp_t,2)+three_spline_A[7][1]*pow(temp_t,3);
				}
			}
			
			// 决策是否动态踢球 结束
			local_p[0] = x;
			local_p[1] = y;
			local_p[2] = 0;
			MatrixMultiVector3x1(T, local_p, world_p);
			world_p[0] = C[0] + world_p[0];
			world_p[1] = C[1] + world_p[1];
			world_p[2] = C[2] + world_p[2];
			basic_left_foot[i][0] = world_p[0];
			basic_left_foot[i][1] = world_p[1];
			basic_left_foot[i][2] = world_p[2];
			double R[3][3];
			double temp[3];
			//准备PID修正
			double delta_roll = 0;
			double delta_pitch = 0;
			double delta_yaw = 0;
			imuGesturePidControl(delta_roll, delta_pitch, delta_yaw);


			rpy2rot(0, 0, theta, R);
			MatrixMultiVector3x1(R, robotModel[LEFT_FOOT].b, temp);
#if SWING_ARM
			// 右臂
			robotModel[RIGHT_ARM_FRONT_SWING].q = current_arm_angle_right + i * (-arm_swing_angle - current_arm_angle_right) / step_basic_frame;
			robotModel[LEFT_ARM_FRONT_SWING].q = current_arm_angle_left + i * (arm_swing_angle - current_arm_angle_left) / step_basic_frame;
#endif
			// 确定腰部位置，四边形顶点坐标求对角线交点坐标,两手一条直线，两脚一条直线
			double x1, y1, x2, y2, x3, y3, x4, y4;
			x1 = robotModel[LEFT_FOOT].p[0]; y1 = robotModel[LEFT_FOOT].p[1];
			x2 = robotModel[RIGHT_FOOT].p[0]; y2 = robotModel[RIGHT_FOOT].p[1];
			x3 = robotModel[RIGHT_HAND].p[0]; y3 = robotModel[RIGHT_HAND].p[1];
			x4 = robotModel[LEFT_HAND].p[0]; y4 = robotModel[LEFT_HAND].p[1];
			//waistPosition(x1, y1, x2, y2, x3, y3, x4, y4);
			waistPosition_com(delta_roll,delta_pitch,theta,i);


			robotModel[LEFT_ANKLE_SIDE_SWING].p[0] = basic_left_foot[i][0] - temp[0];
			robotModel[LEFT_ANKLE_SIDE_SWING].p[1] = basic_left_foot[i][1] - temp[1];
			robotModel[LEFT_ANKLE_SIDE_SWING].p[2] = basic_left_foot[i][2] - temp[2];
			inverseKinmatics_leftFoot(0, 0, theta);

			rpy2rot(0, 0, theta, R);
			MatrixMultiVector3x1(R, robotModel[RIGHT_FOOT].b, temp);
			robotModel[RIGHT_ANKLE_SIDE_SWING].p[0] = solid_right_foot[0] - temp[0];
			robotModel[RIGHT_ANKLE_SIDE_SWING].p[1] = solid_right_foot[1] - temp[1];
			robotModel[RIGHT_ANKLE_SIDE_SWING].p[2] = solid_right_foot[2] - temp[2];
			inverseKinmatics_rightFoot(0, 0, theta);

			Calc_ZMP(Compute_fact_zmp,&robot_taoz);

			#if PID_AMEND
			robotModel[RIGHT_ANKLE_SIDE_SWING].q -= delta_roll;   // 负号是因为要靠地面反作用力来修正姿态
			robotModel[LEFT_ANKLE_SIDE_SWING].q -= delta_roll;
			robotModel[RIGHT_ANKLE_FRONT_SWING].q -= delta_pitch;
			robotModel[LEFT_ANKLE_FRONT_SWING].q -= delta_pitch;
			forwardKinematics(MAIN_BODY);
			#endif


#if WRITETXT
			writeTxt();
#endif
#if ROSPUB
	ikidRobotDynaPosPub();
#endif
#if CONTROLBOARDPUB
	ikidRobotDynaPosControlBoardPub();

#endif
		}
	}
	else
	{
	// 此时左脚右手位置固定，记录位置
		double solid_left_foot[3] = { robotModel[LEFT_FOOT].p[0],robotModel[LEFT_FOOT].p[1],robotModel[LEFT_FOOT].p[2] };
		double solid_right_hand[3] = { robotModel[RIGHT_HAND].p[0],robotModel[RIGHT_HAND].p[1],robotModel[RIGHT_HAND].p[2] };
		//确定轨迹的三点用PQR表示
		double P[3] = { robotModel[RIGHT_FOOT].p[0], robotModel[RIGHT_FOOT].p[1], 0 };
		double Q[3] = { (robotModel[RIGHT_FOOT].p[0] + pn[0]) / 2, (robotModel[RIGHT_FOOT].p[1] + pn[1]) / 2, fh };
		double R[3] = { pn[0], pn[1], 0 };
		double PQ[3] = { Q[0] - P[0], Q[1] - P[1], Q[2] - P[2] };
		double QR[3] = { R[0] - Q[0], R[1] - Q[1], R[2] - Q[2] };
		double n1[3];
		cross3x1(PQ, QR, n1);
		// 三点平面原点C
		double C[3] = { Q[0], Q[1], 0 };
		// x轴单位向量i_prime
		double CP_norm = sqrt((C[0] - P[0]) * (C[0] - P[0]) + (C[1] - P[1]) * (C[1] - P[1]) + (C[2] - P[2]) * (C[2] - P[2]));
		double i_prime[3] = { (P[0] - C[0]) / CP_norm, (P[1] - C[1]) / CP_norm, (P[2] - C[2]) / CP_norm };
		// z轴单位向量k_prime
		double n1_morm = sqrt(n1[0] * n1[0] + n1[1] * n1[1] + n1[2] * n1[2]);
		double k_prime[3] = { n1[0] / n1_morm, n1[1] / n1_morm, n1[2] / n1_morm };
		// y轴单位向量j_prime
		double j_prime[3];
		cross3x1(k_prime, i_prime, j_prime);
		//面到世界坐标系转换矩阵T
		double T[3][3];
		T[0][0] = i_prime[0]; T[0][1] = j_prime[0]; T[0][2] = k_prime[0];
		T[1][0] = i_prime[1]; T[1][1] = j_prime[1]; T[1][2] = k_prime[1];
		T[2][0] = i_prime[2]; T[2][1] = j_prime[2]; T[2][2] = k_prime[2];
		double T_T[3][3];
		invMatrix3x3(T, T_T);

		bool walk_with_ball;
        ros::param::get("walk_with_ball",walk_with_ball);
		double quintic_A[6][4] = {};
		double quintic_B[6][4] = {};
		double three_spline_A[8][2] = {};
		if(walk_with_ball){
			// 求解四段五次多项式插值参数
			quinticPolyInterFour(quintic_A, quintic_B, CP_norm*2);
		}else{
			// 求解两段五次多项式插值参数
			// quinticPolyInterTwo(quintic_A, quintic_B, CP_norm*2);

			// 求解两段三次样条插值参数
			threeSplineInter(three_spline_A,CP_norm*2);
		}

#if SWING_ARM
		// 左臂
		double current_arm_angle_left = robotModel[LEFT_ARM_FRONT_SWING].q;
		double current_arm_angle_right = robotModel[RIGHT_ARM_FRONT_SWING].q;
		double arm_abc[3] = { 0 };
		double arm_length = 0;
		arm_abc[0] = robotModel[LEFT_ARM_FRONT_SWING].p[0] - robotModel[LEFT_HAND].p[0];
		arm_abc[1] = robotModel[LEFT_ARM_FRONT_SWING].p[1] - robotModel[LEFT_HAND].p[1];
		arm_abc[2] = robotModel[LEFT_ARM_FRONT_SWING].p[2] - robotModel[LEFT_HAND].p[2];
		arm_length = norm(arm_abc, 1, 3);
		arm_swing_angle = asin(sx/2 / arm_length);
#endif
		for (int i = 0; i < step_basic_frame; i++)
		{
			double world_p[3];
			double local_p[3];
			double x;
			double y;
			// 决策是否动态踢球 开始
			if(walk_with_ball){
				//{0, step_basic_frame/5*2*frame_T, step_basic_frame/5*3*frame_T, step_basic_frame/5*4*frame_T, step_basic_frame*frame_T}
				if(i < step_basic_frame/5*2){
					double temp_t = i*frame_T;
					x = quintic_A[0][0]+quintic_A[1][0]*temp_t+quintic_A[2][0]*pow(temp_t,2)+quintic_A[3][0]*pow(temp_t,3)+quintic_A[4][0]*pow(temp_t,4)+quintic_A[5][0]*pow(temp_t,5);
					y = quintic_B[0][0]+quintic_B[1][0]*temp_t+quintic_B[2][0]*pow(temp_t,2)+quintic_B[3][0]*pow(temp_t,3)+quintic_B[4][0]*pow(temp_t,4)+quintic_B[5][0]*pow(temp_t,5);
				}else if(i < step_basic_frame/5*3){
					double temp_t = i*frame_T;
					x = quintic_A[0][1]+quintic_A[1][1]*temp_t+quintic_A[2][1]*pow(temp_t,2)+quintic_A[3][1]*pow(temp_t,3)+quintic_A[4][1]*pow(temp_t,4)+quintic_A[5][1]*pow(temp_t,5);
					y = quintic_B[0][1]+quintic_B[1][1]*temp_t+quintic_B[2][1]*pow(temp_t,2)+quintic_B[3][1]*pow(temp_t,3)+quintic_B[4][1]*pow(temp_t,4)+quintic_B[5][1]*pow(temp_t,5);
				}else if(i < step_basic_frame/5*4){
					double temp_t = i*frame_T;
					x = quintic_A[0][2]+quintic_A[1][2]*temp_t+quintic_A[2][2]*pow(temp_t,2)+quintic_A[3][2]*pow(temp_t,3)+quintic_A[4][2]*pow(temp_t,4)+quintic_A[5][2]*pow(temp_t,5);
					y = quintic_B[0][2]+quintic_B[1][2]*temp_t+quintic_B[2][2]*pow(temp_t,2)+quintic_B[3][2]*pow(temp_t,3)+quintic_B[4][2]*pow(temp_t,4)+quintic_B[5][2]*pow(temp_t,5);
				}else{
					double temp_t = i*frame_T;
					x = quintic_A[0][3]+quintic_A[1][3]*temp_t+quintic_A[2][3]*pow(temp_t,2)+quintic_A[3][3]*pow(temp_t,3)+quintic_A[4][3]*pow(temp_t,4)+quintic_A[5][3]*pow(temp_t,5);
					y = quintic_B[0][3]+quintic_B[1][3]*temp_t+quintic_B[2][3]*pow(temp_t,2)+quintic_B[3][3]*pow(temp_t,3)+quintic_B[4][3]*pow(temp_t,4)+quintic_B[5][3]*pow(temp_t,5);
				}
				//printf(" x:%f,y:%f ", x, y);
			}else{
				//  x = CP_norm + -(i + 1) * 2 * CP_norm / step_basic_frame; // sin曲线
				//  y = fh * cos(PI / (2 * CP_norm) * x);
				// if(i < step_basic_frame/2){
				// 	double temp_t = i*frame_T;
				// 	x = quintic_A[0][0]+quintic_A[1][0]*temp_t+quintic_A[2][0]*pow(temp_t,2)+quintic_A[3][0]*pow(temp_t,3)+quintic_A[4][0]*pow(temp_t,4)+quintic_A[5][0]*pow(temp_t,5);
				// 	y = quintic_B[0][0]+quintic_B[1][0]*temp_t+quintic_B[2][0]*pow(temp_t,2)+quintic_B[3][0]*pow(temp_t,3)+quintic_B[4][0]*pow(temp_t,4)+quintic_B[5][0]*pow(temp_t,5);
				// }else{
				// 	double temp_t = i*frame_T;
				// 	x = quintic_A[0][1]+quintic_A[1][1]*temp_t+quintic_A[2][1]*pow(temp_t,2)+quintic_A[3][1]*pow(temp_t,3)+quintic_A[4][1]*pow(temp_t,4)+quintic_A[5][1]*pow(temp_t,5);
				// 	y = quintic_B[0][1]+quintic_B[1][1]*temp_t+quintic_B[2][1]*pow(temp_t,2)+quintic_B[3][1]*pow(temp_t,3)+quintic_B[4][1]*pow(temp_t,4)+quintic_B[5][1]*pow(temp_t,5);
				// }
				if(i < step_basic_frame/2){
					double temp_t = i*frame_T;
					x = three_spline_A[0][0]+three_spline_A[1][0]*temp_t+three_spline_A[2][0]*pow(temp_t,2)+three_spline_A[3][0]*pow(temp_t,3);
					y = three_spline_A[0][1]+three_spline_A[1][1]*temp_t+three_spline_A[2][1]*pow(temp_t,2)+three_spline_A[3][1]*pow(temp_t,3);
				}else{
					double temp_t = i*frame_T;
					x = three_spline_A[4][0]+three_spline_A[5][0]*temp_t+three_spline_A[6][0]*pow(temp_t,2)+three_spline_A[7][0]*pow(temp_t,3);
					y = three_spline_A[4][1]+three_spline_A[5][1]*temp_t+three_spline_A[6][1]*pow(temp_t,2)+three_spline_A[7][1]*pow(temp_t,3);
				}
			}
			
			// 决策是否动态踢球 结束
			local_p[0] = x;
			local_p[1] = y;
			local_p[2] = 0;
			MatrixMultiVector3x1(T, local_p, world_p);
			world_p[0] = C[0] + world_p[0];
			world_p[1] = C[1] + world_p[1];
			world_p[2] = C[2] + world_p[2];
			basic_right_foot[i][0] = world_p[0];
			basic_right_foot[i][1] = world_p[1];
			basic_right_foot[i][2] = world_p[2];
			double R[3][3];
			double temp[3];
			//准备PID修正
			double delta_roll = 0;
			double delta_pitch = 0;
			double delta_yaw = 0;
			imuGesturePidControl(delta_roll, delta_pitch, delta_yaw);


			rpy2rot(0, 0, theta, R);
			MatrixMultiVector3x1(R, robotModel[RIGHT_FOOT].b, temp);
			
#if SWING_ARM
			// 左臂
			robotModel[LEFT_ARM_FRONT_SWING].q = current_arm_angle_left + i * (-arm_swing_angle - current_arm_angle_left) / step_basic_frame;
			robotModel[RIGHT_ARM_FRONT_SWING].q = current_arm_angle_right + i * (arm_swing_angle - current_arm_angle_right) / step_basic_frame;
#endif
			
			// 确定腰部位置，四边形顶点坐标求对角线交点坐标
			double x1, y1, x2, y2, x3, y3, x4, y4;
			x1 = robotModel[LEFT_FOOT].p[0]; y1 = robotModel[LEFT_FOOT].p[1];
			x2 = robotModel[RIGHT_FOOT].p[0]; y2 = robotModel[RIGHT_FOOT].p[1];
			x3 = robotModel[RIGHT_HAND].p[0]; y3 = robotModel[RIGHT_HAND].p[1];
			x4 = robotModel[LEFT_HAND].p[0]; y4 = robotModel[LEFT_HAND].p[1];
			//waistPosition(x1, y1, x2, y2, x3, y3, x4, y4);
			waistPosition_com(delta_roll,delta_pitch,theta,i);


			robotModel[RIGHT_ANKLE_SIDE_SWING].p[0] = basic_right_foot[i][0] - temp[0];
			robotModel[RIGHT_ANKLE_SIDE_SWING].p[1] = basic_right_foot[i][1] - temp[1];
			robotModel[RIGHT_ANKLE_SIDE_SWING].p[2] = basic_right_foot[i][2] - temp[2];
			inverseKinmatics_rightFoot(0, 0, theta);

			rpy2rot(0, 0, theta, R);
			MatrixMultiVector3x1(R, robotModel[LEFT_FOOT].b, temp);
			robotModel[LEFT_ANKLE_SIDE_SWING].p[0] = solid_left_foot[0] - temp[0];
			robotModel[LEFT_ANKLE_SIDE_SWING].p[1] = solid_left_foot[1] - temp[1];
			robotModel[LEFT_ANKLE_SIDE_SWING].p[2] = solid_left_foot[2] - temp[2];
			inverseKinmatics_leftFoot(0, 0, theta);
			Calc_ZMP(Compute_fact_zmp,&robot_taoz);

			#if PID_AMEND
			robotModel[RIGHT_ANKLE_SIDE_SWING].q -= delta_roll;
			robotModel[LEFT_ANKLE_SIDE_SWING].q -= delta_roll;
			robotModel[RIGHT_ANKLE_FRONT_SWING].q -= delta_pitch;
			robotModel[LEFT_ANKLE_FRONT_SWING].q -= delta_pitch;
			forwardKinematics(MAIN_BODY);
			#endif


#if WRITETXT
			writeTxt();
#endif
#if ROSPUB
	ikidRobotDynaPosPub();
#endif
#if CONTROLBOARDPUB
	ikidRobotDynaPosControlBoardPub();

#endif
		}
	}
	dFootSupportPhase(theta,theta,theta);
	changeFoot();


}

void anglePlan(double delta) {
	pn[0] = pn[0] + cos(theta + delta) * sx + (-sin(theta + delta) * sy * (-1) * pow(-1, isLeft));
	pn[1] = pn[1] + sin(theta + delta) * sx + (cos(theta + delta) * sy * (-1) * pow(-1, isLeft));
	isDsPhase = false;
	if (isLeft) {
		//确定轨迹的三点用PQR表示
		// 此时右脚左手位置固定，记录位置
		double solid_right_foot[3] = { robotModel[RIGHT_FOOT].p[0],robotModel[RIGHT_FOOT].p[1],robotModel[RIGHT_FOOT].p[2] };
		double solid_left_hand[3] = { robotModel[LEFT_HAND].p[0],robotModel[LEFT_HAND].p[1],robotModel[LEFT_HAND].p[2] };

		// 左脚
		double P[3] = { robotModel[LEFT_FOOT].p[0], robotModel[LEFT_FOOT].p[1], 0 };
		double Q[3] = { (robotModel[LEFT_FOOT].p[0] + pn[0]) / 2, (robotModel[LEFT_FOOT].p[1] + pn[1]) / 2, fh };
		double R[3] = { pn[0], pn[1], 0 };
		double PQ[3] = { Q[0] - P[0], Q[1] - P[1], Q[2] - P[2] };
		double QR[3] = { R[0] - Q[0], R[1] - Q[1], R[2] - Q[2] };
		double n1[3];
		cross3x1(PQ, QR, n1);
		// 三点平面原点C
		double C[3] = { Q[0], Q[1], 0 };
		// x轴单位向量i_prime
		double CP_norm = sqrt((C[0] - P[0]) * (C[0] - P[0]) + (C[1] - P[1]) * (C[1] - P[1]) + (C[2] - P[2]) * (C[2] - P[2]));
		double i_prime[3] = { (P[0] - C[0]) / CP_norm, (P[1] - C[1]) / CP_norm, (P[2] - C[2]) / CP_norm };
		// z轴单位向量k_prime
		double n1_morm = sqrt(n1[0] * n1[0] + n1[1] * n1[1] + n1[2] * n1[2]);
		double k_prime[3] = { n1[0] / n1_morm, n1[1] / n1_morm, n1[2] / n1_morm };
		// y轴单位向量j_prime
		double j_prime[3];
		cross3x1(k_prime, i_prime, j_prime);
		//面到世界坐标系转换矩阵T
		double T[3][3];
		T[0][0] = i_prime[0]; T[0][1] = j_prime[0]; T[0][2] = k_prime[0];
		T[1][0] = i_prime[1]; T[1][1] = j_prime[1]; T[1][2] = k_prime[1];
		T[2][0] = i_prime[2]; T[2][1] = j_prime[2]; T[2][2] = k_prime[2];
		double T_T[3][3];
		invMatrix3x3(T, T_T);

		double quintic_A[6][4] = {};
		double quintic_B[6][4] = {};
		double three_spline_A[8][2] = {};
		// 求解两段五次多项式插值参数
		// quinticPolyInterTwo(quintic_A, quintic_B, CP_norm*2);
		// 求解两段三次样条插值参数
		threeSplineInter(three_spline_A,CP_norm*2);
#if SWING_ARM
		// 右臂
		double current_arm_angle_right = robotModel[RIGHT_ARM_FRONT_SWING].q;
		double current_arm_angle_left = robotModel[LEFT_ARM_FRONT_SWING].q;
		double arm_abc[3] = { 0 };
		double arm_length = 0;
		arm_abc[0] = robotModel[RIGHT_ARM_FRONT_SWING].p[0] - robotModel[RIGHT_HAND].p[0];
		arm_abc[1] = robotModel[RIGHT_ARM_FRONT_SWING].p[1] - robotModel[RIGHT_HAND].p[1];
		arm_abc[2] = robotModel[RIGHT_ARM_FRONT_SWING].p[2] - robotModel[RIGHT_HAND].p[2];
		arm_length = norm(arm_abc, 1, 3);
		arm_swing_angle = asin(sx/2 / arm_length);

#endif

		for (int i = 0; i < step_basic_frame; i++)
		{
			// SIN曲线
			double world_p[3];
			double local_p[3];
			double x;
			double y;
			//  x = CP_norm + -(i + 1) * 2 * CP_norm / step_basic_frame; // sin曲线
			//  y = fh * cos(PI / (2 * CP_norm) * x);
			// if(i < step_basic_frame/2){
			// 	double temp_t = i*frame_T;
			// 	x = quintic_A[0][0]+quintic_A[1][0]*temp_t+quintic_A[2][0]*pow(temp_t,2)+quintic_A[3][0]*pow(temp_t,3)+quintic_A[4][0]*pow(temp_t,4)+quintic_A[5][0]*pow(temp_t,5);
			// 	y = quintic_B[0][0]+quintic_B[1][0]*temp_t+quintic_B[2][0]*pow(temp_t,2)+quintic_B[3][0]*pow(temp_t,3)+quintic_B[4][0]*pow(temp_t,4)+quintic_B[5][0]*pow(temp_t,5);
			// }else{
			// 	double temp_t = i*frame_T;
			// 	x = quintic_A[0][1]+quintic_A[1][1]*temp_t+quintic_A[2][1]*pow(temp_t,2)+quintic_A[3][1]*pow(temp_t,3)+quintic_A[4][1]*pow(temp_t,4)+quintic_A[5][1]*pow(temp_t,5);
			// 	y = quintic_B[0][1]+quintic_B[1][1]*temp_t+quintic_B[2][1]*pow(temp_t,2)+quintic_B[3][1]*pow(temp_t,3)+quintic_B[4][1]*pow(temp_t,4)+quintic_B[5][1]*pow(temp_t,5);
			// }
			if(i < step_basic_frame/2){
				double temp_t = i*frame_T;
				x = three_spline_A[0][0]+three_spline_A[1][0]*temp_t+three_spline_A[2][0]*pow(temp_t,2)+three_spline_A[3][0]*pow(temp_t,3);
				y = three_spline_A[0][1]+three_spline_A[1][1]*temp_t+three_spline_A[2][1]*pow(temp_t,2)+three_spline_A[3][1]*pow(temp_t,3);
			}else{
				double temp_t = i*frame_T;
				x = three_spline_A[4][0]+three_spline_A[5][0]*temp_t+three_spline_A[6][0]*pow(temp_t,2)+three_spline_A[7][0]*pow(temp_t,3);
				y = three_spline_A[4][1]+three_spline_A[5][1]*temp_t+three_spline_A[6][1]*pow(temp_t,2)+three_spline_A[7][1]*pow(temp_t,3);
			}
			local_p[0] = x;
			local_p[1] = y;
			local_p[2] = 0;
			MatrixMultiVector3x1(T, local_p, world_p);
			world_p[0] = C[0] + world_p[0];
			world_p[1] = C[1] + world_p[1];
			world_p[2] = C[2] + world_p[2];
			basic_left_foot[i][0] = world_p[0];
			basic_left_foot[i][1] = world_p[1];
			basic_left_foot[i][2] = world_p[2];
			double R[3][3];
			double temp[3];
			double theta_frame = i * delta / step_basic_frame;

			//准备PID修正
			double delta_roll = 0;
			double delta_pitch = 0;
			double delta_yaw = 0;
			imuGesturePidControl(delta_roll, delta_pitch, delta_yaw);


			rpy2rot(0, 0, theta + theta_frame, R);
			MatrixMultiVector3x1(R, robotModel[LEFT_FOOT].b, temp);


#if SWING_ARM
			// 右臂
			robotModel[RIGHT_ARM_FRONT_SWING].q = current_arm_angle_right + i * (-arm_swing_angle - current_arm_angle_right) / step_basic_frame;
			robotModel[LEFT_ARM_FRONT_SWING].q = current_arm_angle_left + i * (arm_swing_angle - current_arm_angle_left) / step_basic_frame;

#endif


			// 确定腰部位置，四边形顶点坐标求对角线交点坐标,两手一条直线，两脚一条直线
			double x1, y1, x2, y2, x3, y3, x4, y4;
			x1 = robotModel[LEFT_FOOT].p[0]; y1 = robotModel[LEFT_FOOT].p[1];
			x2 = robotModel[RIGHT_FOOT].p[0]; y2 = robotModel[RIGHT_FOOT].p[1];
			x3 = robotModel[RIGHT_HAND].p[0]; y3 = robotModel[RIGHT_HAND].p[1];
			x4 = robotModel[LEFT_HAND].p[0]; y4 = robotModel[LEFT_HAND].p[1];
			//waistPosition(x1, y1, x2, y2, x3, y3, x4, y4);
			waistPosition_com(delta_roll, delta_pitch, theta + theta_frame/2,i);


			robotModel[LEFT_ANKLE_SIDE_SWING].p[0] = basic_left_foot[i][0] - temp[0];
			robotModel[LEFT_ANKLE_SIDE_SWING].p[1] = basic_left_foot[i][1] - temp[1];
			robotModel[LEFT_ANKLE_SIDE_SWING].p[2] = basic_left_foot[i][2] - temp[2];
			inverseKinmatics_leftFoot(0, 0, theta + theta_frame);

			rpy2rot(0, 0, theta, R);
			MatrixMultiVector3x1(R, robotModel[RIGHT_FOOT].b, temp);
			robotModel[RIGHT_ANKLE_SIDE_SWING].p[0] = solid_right_foot[0] - temp[0];
			robotModel[RIGHT_ANKLE_SIDE_SWING].p[1] = solid_right_foot[1] - temp[1];
			robotModel[RIGHT_ANKLE_SIDE_SWING].p[2] = solid_right_foot[2] - temp[2];
			inverseKinmatics_rightFoot(0, 0, theta);

			Calc_ZMP(Compute_fact_zmp,&robot_taoz);

			#if PID_AMEND
			robotModel[RIGHT_ANKLE_SIDE_SWING].q -= delta_roll;
			robotModel[LEFT_ANKLE_SIDE_SWING].q -= delta_roll;
			robotModel[RIGHT_ANKLE_FRONT_SWING].q -= delta_pitch;
			robotModel[LEFT_ANKLE_FRONT_SWING].q -= delta_pitch;
			forwardKinematics(MAIN_BODY);
			#endif


#if WRITETXT
			writeTxt();
#endif
#if ROSPUB
	ikidRobotDynaPosPub();
#endif
#if CONTROLBOARDPUB
	ikidRobotDynaPosControlBoardPub();

#endif
		}
	}
	else
	{
		// 此时左脚右手位置固定，记录位置
		double solid_left_foot[3] = { robotModel[LEFT_FOOT].p[0],robotModel[LEFT_FOOT].p[1],robotModel[LEFT_FOOT].p[2] };
		double solid_right_hand[3] = { robotModel[RIGHT_HAND].p[0],robotModel[RIGHT_HAND].p[1],robotModel[RIGHT_HAND].p[2] };
		//确定轨迹的三点用PQR表示
		double P[3] = { robotModel[RIGHT_FOOT].p[0], robotModel[RIGHT_FOOT].p[1], 0 };
		double Q[3] = { (robotModel[RIGHT_FOOT].p[0] + pn[0]) / 2, (robotModel[RIGHT_FOOT].p[1] + pn[1]) / 2, fh };
		double R[3] = { pn[0], pn[1], 0 };
		double PQ[3] = { Q[0] - P[0], Q[1] - P[1], Q[2] - P[2] };
		double QR[3] = { R[0] - Q[0], R[1] - Q[1], R[2] - Q[2] };
		double n1[3];
		cross3x1(PQ, QR, n1);
		// 三点平面原点C
		double C[3] = { Q[0], Q[1], 0 };
		// x轴单位向量i_prime
		double CP_norm = sqrt((C[0] - P[0]) * (C[0] - P[0]) + (C[1] - P[1]) * (C[1] - P[1]) + (C[2] - P[2]) * (C[2] - P[2]));
		double i_prime[3] = { (P[0] - C[0]) / CP_norm, (P[1] - C[1]) / CP_norm, (P[2] - C[2]) / CP_norm };
		// z轴单位向量k_prime
		double n1_morm = sqrt(n1[0] * n1[0] + n1[1] * n1[1] + n1[2] * n1[2]);
		double k_prime[3] = { n1[0] / n1_morm, n1[1] / n1_morm, n1[2] / n1_morm };
		// y轴单位向量j_prime
		double j_prime[3];
		cross3x1(k_prime, i_prime, j_prime);
		//面到世界坐标系转换矩阵T
		double T[3][3];
		T[0][0] = i_prime[0]; T[0][1] = j_prime[0]; T[0][2] = k_prime[0];
		T[1][0] = i_prime[1]; T[1][1] = j_prime[1]; T[1][2] = k_prime[1];
		T[2][0] = i_prime[2]; T[2][1] = j_prime[2]; T[2][2] = k_prime[2];
		double T_T[3][3];
		invMatrix3x3(T, T_T);

		double quintic_A[6][4] = {};
		double quintic_B[6][4] = {};
		double three_spline_A[8][2] = {};
		// 求解两段五次多项式插值参数
		// quinticPolyInterTwo(quintic_A, quintic_B, CP_norm*2);
		// 求解两段三次样条插值参数
		threeSplineInter(three_spline_A,CP_norm*2);
#if SWING_ARM
		// 左臂
		double current_arm_angle_left = robotModel[LEFT_ARM_FRONT_SWING].q;
		double current_arm_angle_right = robotModel[RIGHT_ARM_FRONT_SWING].q;
		double arm_abc[3] = { 0 };
		double arm_length = 0;
		arm_abc[0] = robotModel[LEFT_ARM_FRONT_SWING].p[0] - robotModel[LEFT_HAND].p[0];
		arm_abc[1] = robotModel[LEFT_ARM_FRONT_SWING].p[1] - robotModel[LEFT_HAND].p[1];
		arm_abc[2] = robotModel[LEFT_ARM_FRONT_SWING].p[2] - robotModel[LEFT_HAND].p[2];
		arm_length = norm(arm_abc, 1, 3);
		arm_swing_angle = asin(sx/2 / arm_length);
#endif

		for (int i = 0; i < step_basic_frame; i++)
		{
			double world_p[3];
			double local_p[3];
			double x;
			double y;
			//  x = CP_norm + -(i + 1) * 2 * CP_norm / step_basic_frame; // sin曲线
			//  y = fh * cos(PI / (2 * CP_norm) * x);
			// if(i < step_basic_frame/2){
			// 	double temp_t = i*frame_T;
			// 	x = quintic_A[0][0]+quintic_A[1][0]*temp_t+quintic_A[2][0]*pow(temp_t,2)+quintic_A[3][0]*pow(temp_t,3)+quintic_A[4][0]*pow(temp_t,4)+quintic_A[5][0]*pow(temp_t,5);
			// 	y = quintic_B[0][0]+quintic_B[1][0]*temp_t+quintic_B[2][0]*pow(temp_t,2)+quintic_B[3][0]*pow(temp_t,3)+quintic_B[4][0]*pow(temp_t,4)+quintic_B[5][0]*pow(temp_t,5);
			// }else{
			// 	double temp_t = i*frame_T;
			// 	x = quintic_A[0][1]+quintic_A[1][1]*temp_t+quintic_A[2][1]*pow(temp_t,2)+quintic_A[3][1]*pow(temp_t,3)+quintic_A[4][1]*pow(temp_t,4)+quintic_A[5][1]*pow(temp_t,5);
			// 	y = quintic_B[0][1]+quintic_B[1][1]*temp_t+quintic_B[2][1]*pow(temp_t,2)+quintic_B[3][1]*pow(temp_t,3)+quintic_B[4][1]*pow(temp_t,4)+quintic_B[5][1]*pow(temp_t,5);
			// }
			if(i < step_basic_frame/2){
				double temp_t = i*frame_T;
				x = three_spline_A[0][0]+three_spline_A[1][0]*temp_t+three_spline_A[2][0]*pow(temp_t,2)+three_spline_A[3][0]*pow(temp_t,3);
				y = three_spline_A[0][1]+three_spline_A[1][1]*temp_t+three_spline_A[2][1]*pow(temp_t,2)+three_spline_A[3][1]*pow(temp_t,3);
			}else{
				double temp_t = i*frame_T;
				x = three_spline_A[4][0]+three_spline_A[5][0]*temp_t+three_spline_A[6][0]*pow(temp_t,2)+three_spline_A[7][0]*pow(temp_t,3);
				y = three_spline_A[4][1]+three_spline_A[5][1]*temp_t+three_spline_A[6][1]*pow(temp_t,2)+three_spline_A[7][1]*pow(temp_t,3);
			}
			local_p[0] = x;
			local_p[1] = y;
			local_p[2] = 0;
			MatrixMultiVector3x1(T, local_p, world_p);
			world_p[0] = C[0] + world_p[0];
			world_p[1] = C[1] + world_p[1];
			world_p[2] = C[2] + world_p[2];
			basic_right_foot[i][0] = world_p[0];
			basic_right_foot[i][1] = world_p[1];
			basic_right_foot[i][2] = world_p[2];
			double R[3][3];
			double temp[3];
			double theta_frame = i * delta / step_basic_frame;

			//准备PID修正
			double delta_roll = 0;
			double delta_pitch = 0;
			double delta_yaw = 0;
			imuGesturePidControl(delta_roll, delta_pitch, delta_yaw);



			rpy2rot(0, 0, theta + theta_frame, R);
			MatrixMultiVector3x1(R, robotModel[RIGHT_FOOT].b, temp);


#if SWING_ARM
			// 左臂
			robotModel[LEFT_ARM_FRONT_SWING].q = current_arm_angle_left + i * (-arm_swing_angle - current_arm_angle_left) / step_basic_frame;
			robotModel[RIGHT_ARM_FRONT_SWING].q = current_arm_angle_right + i * (arm_swing_angle - current_arm_angle_right) / step_basic_frame;

#endif

			// 确定腰部位置，四边形顶点坐标求对角线交点坐标
			double x1, y1, x2, y2, x3, y3, x4, y4;
			x1 = robotModel[LEFT_FOOT].p[0]; y1 = robotModel[LEFT_FOOT].p[1];
			x2 = robotModel[RIGHT_FOOT].p[0]; y2 = robotModel[RIGHT_FOOT].p[1];
			x3 = robotModel[RIGHT_HAND].p[0]; y3 = robotModel[RIGHT_HAND].p[1];
			x4 = robotModel[LEFT_HAND].p[0]; y4 = robotModel[LEFT_HAND].p[1];
			//waistPosition(x1, y1, x2, y2, x3, y3, x4, y4);
			waistPosition_com(delta_roll, delta_pitch, theta + theta_frame / 2 ,i);


			robotModel[RIGHT_ANKLE_SIDE_SWING].p[0] = basic_right_foot[i][0] - temp[0];
			robotModel[RIGHT_ANKLE_SIDE_SWING].p[1] = basic_right_foot[i][1] - temp[1];
			robotModel[RIGHT_ANKLE_SIDE_SWING].p[2] = basic_right_foot[i][2] - temp[2];
			inverseKinmatics_rightFoot(0, 0, theta + theta_frame );

			rpy2rot(0, 0, theta, R);
			MatrixMultiVector3x1(R, robotModel[LEFT_FOOT].b, temp);
			robotModel[LEFT_ANKLE_SIDE_SWING].p[0] = solid_left_foot[0] - temp[0];
			robotModel[LEFT_ANKLE_SIDE_SWING].p[1] = solid_left_foot[1] - temp[1];
			robotModel[LEFT_ANKLE_SIDE_SWING].p[2] = solid_left_foot[2] - temp[2];
			inverseKinmatics_leftFoot(0, 0, theta);

			Calc_ZMP(Compute_fact_zmp,&robot_taoz);

			#if PID_AMEND
			robotModel[RIGHT_ANKLE_SIDE_SWING].q -= delta_roll;
			robotModel[LEFT_ANKLE_SIDE_SWING].q -= delta_roll;
			robotModel[RIGHT_ANKLE_FRONT_SWING].q -= delta_pitch;
			robotModel[LEFT_ANKLE_FRONT_SWING].q -= delta_pitch;
			forwardKinematics(MAIN_BODY);
			#endif


#if WRITETXT
			writeTxt();
#endif
#if ROSPUB
	ikidRobotDynaPosPub();
#endif
#if CONTROLBOARDPUB
	ikidRobotDynaPosControlBoardPub();

#endif
		}
	}
	if (isLeft) {
		dFootSupportPhase(theta + delta / 2, theta + delta, theta);
	}
	else
	{
		dFootSupportPhase(theta + delta/2, theta, theta + delta);
	}
	changeFoot();

	// 拐弯第二步

	pn[0] = pn[0] + cos(theta + delta) * sx + (-sin(theta + delta) * sy * (-1) * pow(-1, isLeft));
	pn[1] = pn[1] + sin(theta + delta) * sx + (cos(theta + delta) * sy * (-1) * pow(-1, isLeft));
	isDsPhase = false;
	if (isLeft) {
		//确定轨迹的三点用PQR表示
		// 此时右脚左手位置固定，记录位置
		double solid_right_foot[3] = { robotModel[RIGHT_FOOT].p[0],robotModel[RIGHT_FOOT].p[1],robotModel[RIGHT_FOOT].p[2] };
		double solid_left_hand[3] = { robotModel[LEFT_HAND].p[0],robotModel[LEFT_HAND].p[1],robotModel[LEFT_HAND].p[2] };

		// 左脚
		double P[3] = { robotModel[LEFT_FOOT].p[0], robotModel[LEFT_FOOT].p[1], 0 };
		double Q[3] = { (robotModel[LEFT_FOOT].p[0] + pn[0]) / 2, (robotModel[LEFT_FOOT].p[1] + pn[1]) / 2, fh };
		double R[3] = { pn[0], pn[1], 0 };
		double PQ[3] = { Q[0] - P[0], Q[1] - P[1], Q[2] - P[2] };
		double QR[3] = { R[0] - Q[0], R[1] - Q[1], R[2] - Q[2] };
		double n1[3];
		cross3x1(PQ, QR, n1);
		// 三点平面原点C
		double C[3] = { Q[0], Q[1], 0 };
		// x轴单位向量i_prime
		double CP_norm = sqrt((C[0] - P[0]) * (C[0] - P[0]) + (C[1] - P[1]) * (C[1] - P[1]) + (C[2] - P[2]) * (C[2] - P[2]));
		double i_prime[3] = { (P[0] - C[0]) / CP_norm, (P[1] - C[1]) / CP_norm, (P[2] - C[2]) / CP_norm };
		// z轴单位向量k_prime
		double n1_morm = sqrt(n1[0] * n1[0] + n1[1] * n1[1] + n1[2] * n1[2]);
		double k_prime[3] = { n1[0] / n1_morm, n1[1] / n1_morm, n1[2] / n1_morm };
		// y轴单位向量j_prime
		double j_prime[3];
		cross3x1(k_prime, i_prime, j_prime);
		//面到世界坐标系转换矩阵T
		double T[3][3];
		T[0][0] = i_prime[0]; T[0][1] = j_prime[0]; T[0][2] = k_prime[0];
		T[1][0] = i_prime[1]; T[1][1] = j_prime[1]; T[1][2] = k_prime[1];
		T[2][0] = i_prime[2]; T[2][1] = j_prime[2]; T[2][2] = k_prime[2];
		double T_T[3][3];
		invMatrix3x3(T, T_T);

		double quintic_A[6][4] = {};
		double quintic_B[6][4] = {};
		double three_spline_A[8][2] = {};
		// 求解两段五次多项式插值参数
		// quinticPolyInterTwo(quintic_A, quintic_B, CP_norm*2);
		// 求解两段三次样条插值参数
		threeSplineInter(three_spline_A,CP_norm*2);

#if SWING_ARM
		// 右臂
		double current_arm_angle_right = robotModel[RIGHT_ARM_FRONT_SWING].q;
		double current_arm_angle_left = robotModel[LEFT_ARM_FRONT_SWING].q;
		double arm_abc[3] = { 0 };
		double arm_length = 0;
		arm_abc[0] = robotModel[RIGHT_ARM_FRONT_SWING].p[0] - robotModel[RIGHT_HAND].p[0];
		arm_abc[1] = robotModel[RIGHT_ARM_FRONT_SWING].p[1] - robotModel[RIGHT_HAND].p[1];
		arm_abc[2] = robotModel[RIGHT_ARM_FRONT_SWING].p[2] - robotModel[RIGHT_HAND].p[2];
		arm_length = norm(arm_abc, 1, 3);
		arm_swing_angle = asin(sx/2 / arm_length);

#endif

		for (int i = 0; i < step_basic_frame; i++)
		{
			// SIN曲线
			double world_p[3];
			double local_p[3];
			double x;
			double y;
			//  x = CP_norm + -(i + 1) * 2 * CP_norm / step_basic_frame; // sin曲线
			//  y = fh * cos(PI / (2 * CP_norm) * x);
			// if(i < step_basic_frame/2){
			// 	double temp_t = i*frame_T;
			// 	x = quintic_A[0][0]+quintic_A[1][0]*temp_t+quintic_A[2][0]*pow(temp_t,2)+quintic_A[3][0]*pow(temp_t,3)+quintic_A[4][0]*pow(temp_t,4)+quintic_A[5][0]*pow(temp_t,5);
			// 	y = quintic_B[0][0]+quintic_B[1][0]*temp_t+quintic_B[2][0]*pow(temp_t,2)+quintic_B[3][0]*pow(temp_t,3)+quintic_B[4][0]*pow(temp_t,4)+quintic_B[5][0]*pow(temp_t,5);
			// }else{
			// 	double temp_t = i*frame_T;
			// 	x = quintic_A[0][1]+quintic_A[1][1]*temp_t+quintic_A[2][1]*pow(temp_t,2)+quintic_A[3][1]*pow(temp_t,3)+quintic_A[4][1]*pow(temp_t,4)+quintic_A[5][1]*pow(temp_t,5);
			// 	y = quintic_B[0][1]+quintic_B[1][1]*temp_t+quintic_B[2][1]*pow(temp_t,2)+quintic_B[3][1]*pow(temp_t,3)+quintic_B[4][1]*pow(temp_t,4)+quintic_B[5][1]*pow(temp_t,5);
			// }
			if(i < step_basic_frame/2){
				double temp_t = i*frame_T;
				x = three_spline_A[0][0]+three_spline_A[1][0]*temp_t+three_spline_A[2][0]*pow(temp_t,2)+three_spline_A[3][0]*pow(temp_t,3);
				y = three_spline_A[0][1]+three_spline_A[1][1]*temp_t+three_spline_A[2][1]*pow(temp_t,2)+three_spline_A[3][1]*pow(temp_t,3);
			}else{
				double temp_t = i*frame_T;
				x = three_spline_A[4][0]+three_spline_A[5][0]*temp_t+three_spline_A[6][0]*pow(temp_t,2)+three_spline_A[7][0]*pow(temp_t,3);
				y = three_spline_A[4][1]+three_spline_A[5][1]*temp_t+three_spline_A[6][1]*pow(temp_t,2)+three_spline_A[7][1]*pow(temp_t,3);
			}
			local_p[0] = x;
			local_p[1] = y;
			local_p[2] = 0;
			MatrixMultiVector3x1(T, local_p, world_p);
			world_p[0] = C[0] + world_p[0];
			world_p[1] = C[1] + world_p[1];
			world_p[2] = C[2] + world_p[2];
			basic_left_foot[i][0] = world_p[0];
			basic_left_foot[i][1] = world_p[1];
			basic_left_foot[i][2] = world_p[2];
			double R[3][3];
			double temp[3];
			double theta_frame = i * delta / step_basic_frame;

			//准备PID修正
			double delta_roll = 0;
			double delta_pitch = 0;
			double delta_yaw = 0;
			imuGesturePidControl(delta_roll, delta_pitch, delta_yaw);


			rpy2rot(0, 0, theta + theta_frame , R);
			MatrixMultiVector3x1(R, robotModel[LEFT_FOOT].b, temp);


#if SWING_ARM
			// 右臂
			robotModel[RIGHT_ARM_FRONT_SWING].q = current_arm_angle_right + i * (-arm_swing_angle - current_arm_angle_right) / step_basic_frame;
			robotModel[LEFT_ARM_FRONT_SWING].q = current_arm_angle_left + i * (arm_swing_angle - current_arm_angle_left) / step_basic_frame;

#endif


			// 确定腰部位置，四边形顶点坐标求对角线交点坐标,两手一条直线，两脚一条直线
			double x1, y1, x2, y2, x3, y3, x4, y4;
			x1 = robotModel[LEFT_FOOT].p[0]; y1 = robotModel[LEFT_FOOT].p[1];
			x2 = robotModel[RIGHT_FOOT].p[0]; y2 = robotModel[RIGHT_FOOT].p[1];
			x3 = robotModel[RIGHT_HAND].p[0]; y3 = robotModel[RIGHT_HAND].p[1];
			x4 = robotModel[LEFT_HAND].p[0]; y4 = robotModel[LEFT_HAND].p[1];
			//waistPosition(x1, y1, x2, y2, x3, y3, x4, y4);
			waistPosition_com(delta_roll, delta_pitch, theta + delta / 2+theta_frame / 2 ,i);


			robotModel[LEFT_ANKLE_SIDE_SWING].p[0] = basic_left_foot[i][0] - temp[0];
			robotModel[LEFT_ANKLE_SIDE_SWING].p[1] = basic_left_foot[i][1] - temp[1];
			robotModel[LEFT_ANKLE_SIDE_SWING].p[2] = basic_left_foot[i][2] - temp[2];
			inverseKinmatics_leftFoot(0, 0, theta + theta_frame );

			rpy2rot(0, 0, theta + delta, R);
			MatrixMultiVector3x1(R, robotModel[RIGHT_FOOT].b, temp);
			robotModel[RIGHT_ANKLE_SIDE_SWING].p[0] = solid_right_foot[0] - temp[0];
			robotModel[RIGHT_ANKLE_SIDE_SWING].p[1] = solid_right_foot[1] - temp[1];
			robotModel[RIGHT_ANKLE_SIDE_SWING].p[2] = solid_right_foot[2] - temp[2];
			inverseKinmatics_rightFoot(0, 0, theta + delta );

			Calc_ZMP(Compute_fact_zmp,&robot_taoz);

			#if PID_AMEND
			robotModel[RIGHT_ANKLE_SIDE_SWING].q -= delta_roll;
			robotModel[LEFT_ANKLE_SIDE_SWING].q -= delta_roll;
			robotModel[RIGHT_ANKLE_FRONT_SWING].q -= delta_pitch;
			robotModel[LEFT_ANKLE_FRONT_SWING].q -= delta_pitch;
			forwardKinematics(MAIN_BODY);
			#endif


#if WRITETXT
			writeTxt();
#endif
#if ROSPUB
	ikidRobotDynaPosPub();
#endif
#if CONTROLBOARDPUB
	ikidRobotDynaPosControlBoardPub();

#endif
		}
	}
	else
	{
		// 此时左脚右手位置固定，记录位置
		double solid_left_foot[3] = { robotModel[LEFT_FOOT].p[0],robotModel[LEFT_FOOT].p[1],robotModel[LEFT_FOOT].p[2] };
		double solid_right_hand[3] = { robotModel[RIGHT_HAND].p[0],robotModel[RIGHT_HAND].p[1],robotModel[RIGHT_HAND].p[2] };
		//确定轨迹的三点用PQR表示
		double P[3] = { robotModel[RIGHT_FOOT].p[0], robotModel[RIGHT_FOOT].p[1], 0 };
		double Q[3] = { (robotModel[RIGHT_FOOT].p[0] + pn[0]) / 2, (robotModel[RIGHT_FOOT].p[1] + pn[1]) / 2, fh };
		double R[3] = { pn[0], pn[1], 0 };
		double PQ[3] = { Q[0] - P[0], Q[1] - P[1], Q[2] - P[2] };
		double QR[3] = { R[0] - Q[0], R[1] - Q[1], R[2] - Q[2] };
		double n1[3];
		cross3x1(PQ, QR, n1);
		// 三点平面原点C
		double C[3] = { Q[0], Q[1], 0 };
		// x轴单位向量i_prime
		double CP_norm = sqrt((C[0] - P[0]) * (C[0] - P[0]) + (C[1] - P[1]) * (C[1] - P[1]) + (C[2] - P[2]) * (C[2] - P[2]));
		double i_prime[3] = { (P[0] - C[0]) / CP_norm, (P[1] - C[1]) / CP_norm, (P[2] - C[2]) / CP_norm };
		// z轴单位向量k_prime
		double n1_morm = sqrt(n1[0] * n1[0] + n1[1] * n1[1] + n1[2] * n1[2]);
		double k_prime[3] = { n1[0] / n1_morm, n1[1] / n1_morm, n1[2] / n1_morm };
		// y轴单位向量j_prime
		double j_prime[3];
		cross3x1(k_prime, i_prime, j_prime);
		//面到世界坐标系转换矩阵T
		double T[3][3];
		T[0][0] = i_prime[0]; T[0][1] = j_prime[0]; T[0][2] = k_prime[0];
		T[1][0] = i_prime[1]; T[1][1] = j_prime[1]; T[1][2] = k_prime[1];
		T[2][0] = i_prime[2]; T[2][1] = j_prime[2]; T[2][2] = k_prime[2];
		double T_T[3][3];
		invMatrix3x3(T, T_T);

		double quintic_A[6][4] = {};
		double quintic_B[6][4] = {};
		double three_spline_A[8][2] = {};
		// 求解两段五次多项式插值参数
		// quinticPolyInterTwo(quintic_A, quintic_B, CP_norm*2);
		// 求解两段三次样条插值参数
		threeSplineInter(three_spline_A,CP_norm*2);
#if SWING_ARM
		// 左臂
		double current_arm_angle_left = robotModel[LEFT_ARM_FRONT_SWING].q;
		double current_arm_angle_right = robotModel[RIGHT_ARM_FRONT_SWING].q;
		double arm_abc[3] = { 0 };
		double arm_length = 0;
		arm_abc[0] = robotModel[LEFT_ARM_FRONT_SWING].p[0] - robotModel[LEFT_HAND].p[0];
		arm_abc[1] = robotModel[LEFT_ARM_FRONT_SWING].p[1] - robotModel[LEFT_HAND].p[1];
		arm_abc[2] = robotModel[LEFT_ARM_FRONT_SWING].p[2] - robotModel[LEFT_HAND].p[2];
		arm_length = norm(arm_abc, 1, 3);
		arm_swing_angle = asin(sx/2 / arm_length);
#endif

		for (int i = 0; i < step_basic_frame; i++)
		{
			double world_p[3];
			double local_p[3];
			double x;
			double y;
			//  x = CP_norm + -(i + 1) * 2 * CP_norm / step_basic_frame; // sin曲线
			//  y = fh * cos(PI / (2 * CP_norm) * x);
			// if(i < step_basic_frame/2){
			// 	double temp_t = i*frame_T;
			// 	x = quintic_A[0][0]+quintic_A[1][0]*temp_t+quintic_A[2][0]*pow(temp_t,2)+quintic_A[3][0]*pow(temp_t,3)+quintic_A[4][0]*pow(temp_t,4)+quintic_A[5][0]*pow(temp_t,5);
			// 	y = quintic_B[0][0]+quintic_B[1][0]*temp_t+quintic_B[2][0]*pow(temp_t,2)+quintic_B[3][0]*pow(temp_t,3)+quintic_B[4][0]*pow(temp_t,4)+quintic_B[5][0]*pow(temp_t,5);
			// }else{
			// 	double temp_t = i*frame_T;
			// 	x = quintic_A[0][1]+quintic_A[1][1]*temp_t+quintic_A[2][1]*pow(temp_t,2)+quintic_A[3][1]*pow(temp_t,3)+quintic_A[4][1]*pow(temp_t,4)+quintic_A[5][1]*pow(temp_t,5);
			// 	y = quintic_B[0][1]+quintic_B[1][1]*temp_t+quintic_B[2][1]*pow(temp_t,2)+quintic_B[3][1]*pow(temp_t,3)+quintic_B[4][1]*pow(temp_t,4)+quintic_B[5][1]*pow(temp_t,5);
			// }
			if(i < step_basic_frame/2){
				double temp_t = i*frame_T;
				x = three_spline_A[0][0]+three_spline_A[1][0]*temp_t+three_spline_A[2][0]*pow(temp_t,2)+three_spline_A[3][0]*pow(temp_t,3);
				y = three_spline_A[0][1]+three_spline_A[1][1]*temp_t+three_spline_A[2][1]*pow(temp_t,2)+three_spline_A[3][1]*pow(temp_t,3);
			}else{
				double temp_t = i*frame_T;
				x = three_spline_A[4][0]+three_spline_A[5][0]*temp_t+three_spline_A[6][0]*pow(temp_t,2)+three_spline_A[7][0]*pow(temp_t,3);
				y = three_spline_A[4][1]+three_spline_A[5][1]*temp_t+three_spline_A[6][1]*pow(temp_t,2)+three_spline_A[7][1]*pow(temp_t,3);
			}
			local_p[0] = x;
			local_p[1] = y;
			local_p[2] = 0;
			MatrixMultiVector3x1(T, local_p, world_p);
			world_p[0] = C[0] + world_p[0];
			world_p[1] = C[1] + world_p[1];
			world_p[2] = C[2] + world_p[2];
			basic_right_foot[i][0] = world_p[0];
			basic_right_foot[i][1] = world_p[1];
			basic_right_foot[i][2] = world_p[2];
			double R[3][3];
			double temp[3];
			double theta_frame = i * delta / step_basic_frame;

			//准备PID修正
			double delta_roll = 0;
			double delta_pitch = 0;
			double delta_yaw = 0;
			imuGesturePidControl(delta_roll, delta_pitch, delta_yaw);


			rpy2rot(0, 0, theta + theta_frame , R);
			MatrixMultiVector3x1(R, robotModel[RIGHT_FOOT].b, temp);


#if SWING_ARM
			// 左臂
			robotModel[LEFT_ARM_FRONT_SWING].q = current_arm_angle_left + i * (-arm_swing_angle - current_arm_angle_left) / step_basic_frame;
			robotModel[RIGHT_ARM_FRONT_SWING].q = current_arm_angle_right + i * (arm_swing_angle - current_arm_angle_right) / step_basic_frame;

#endif


			// 确定腰部位置，四边形顶点坐标求对角线交点坐标
			double x1, y1, x2, y2, x3, y3, x4, y4;
			x1 = robotModel[LEFT_FOOT].p[0]; y1 = robotModel[LEFT_FOOT].p[1];
			x2 = robotModel[RIGHT_FOOT].p[0]; y2 = robotModel[RIGHT_FOOT].p[1];
			x3 = robotModel[RIGHT_HAND].p[0]; y3 = robotModel[RIGHT_HAND].p[1];
			x4 = robotModel[LEFT_HAND].p[0]; y4 = robotModel[LEFT_HAND].p[1];
			//waistPosition(x1, y1, x2, y2, x3, y3, x4, y4);
			waistPosition_com(delta_roll, delta_pitch, theta + delta / 2 + theta_frame / 2 ,i);


			robotModel[RIGHT_ANKLE_SIDE_SWING].p[0] = basic_right_foot[i][0] - temp[0];
			robotModel[RIGHT_ANKLE_SIDE_SWING].p[1] = basic_right_foot[i][1] - temp[1];
			robotModel[RIGHT_ANKLE_SIDE_SWING].p[2] = basic_right_foot[i][2] - temp[2];
			inverseKinmatics_rightFoot(0, 0, theta + theta_frame );

			rpy2rot(0, 0, theta + delta , R);
			MatrixMultiVector3x1(R, robotModel[LEFT_FOOT].b, temp);
			robotModel[LEFT_ANKLE_SIDE_SWING].p[0] = solid_left_foot[0] - temp[0];
			robotModel[LEFT_ANKLE_SIDE_SWING].p[1] = solid_left_foot[1] - temp[1];
			robotModel[LEFT_ANKLE_SIDE_SWING].p[2] = solid_left_foot[2] - temp[2];
			inverseKinmatics_leftFoot(0, 0, theta + delta );

			Calc_ZMP(Compute_fact_zmp,&robot_taoz);

			#if PID_AMEND
			robotModel[RIGHT_ANKLE_SIDE_SWING].q -= delta_roll;
			robotModel[LEFT_ANKLE_SIDE_SWING].q -= delta_roll;
			robotModel[RIGHT_ANKLE_FRONT_SWING].q -= delta_pitch;
			robotModel[LEFT_ANKLE_FRONT_SWING].q -= delta_pitch;
			forwardKinematics(MAIN_BODY);
			#endif

#if WRITETXT
			writeTxt();
#endif
#if ROSPUB
	ikidRobotDynaPosPub();
#endif
#if CONTROLBOARDPUB
	ikidRobotDynaPosControlBoardPub();

#endif
		}
	}
	theta = theta + delta;
	dFootSupportPhase(theta, theta, theta);
	changeFoot();
	
}

void CalcTrajectory_Com(int current_frame_count) {
	double zmp_preview[2][N_preview] = {0};
	double temp_pn[2] = { 0 };
	if(ikid_start_walk_flag){  // 起步状态单独处理
		temp_pn[0] = pn[0] + cos(theta) * sx + (-sin(theta) * sy * (-1) * pow(-1, isLeft));
		temp_pn[1] = pn[1] + sin(theta) * sx + (cos(theta) * sy * (-1) * pow(-1, isLeft));
	}else{
		temp_pn[0] = pn[0] + cos(theta) * sx + (-sin(theta) * sy * (-1) * pow(-1, !isLeft));
		temp_pn[1] = pn[1] + sin(theta) * sx + (cos(theta) * sy * (-1) * pow(-1, !isLeft));
	}
	
	
	
	for (int i = 0; i < N_preview; i++)
	{
		if (!isDsPhase) {
			if (step_basic_frame - current_frame_count > i) {
				zmp_preview[0][i] = support_ZMP[0];
				zmp_preview[1][i] = support_ZMP[1];
			}
			else if (i - (step_basic_frame - current_frame_count) < ds_frame) {
				zmp_preview[0][i] = support_ZMP[0] + (i - (step_basic_frame - current_frame_count)) * (pn[0] - support_ZMP[0]) / ds_frame;
				zmp_preview[1][i] = support_ZMP[1] + (i - (step_basic_frame - current_frame_count)) * (pn[1] - support_ZMP[1]) / ds_frame;
			}
			else if (i - (step_basic_frame - current_frame_count) - ds_frame < step_basic_frame) {
				zmp_preview[0][i] = pn[0];
				zmp_preview[1][i] = pn[1];
			}
			else if (i - (step_basic_frame - current_frame_count) - ds_frame - step_basic_frame < ds_frame) {
				zmp_preview[0][i] = pn[0] + (i - (step_basic_frame - current_frame_count) - ds_frame - step_basic_frame) * (temp_pn[0] - pn[0]) / ds_frame;
				zmp_preview[1][i] = pn[1] + (i - (step_basic_frame - current_frame_count) - ds_frame - step_basic_frame) * (temp_pn[1] - pn[1]) / ds_frame;
			}
			else if(i - (step_basic_frame*2 - current_frame_count) - ds_frame*2 < step_basic_frame)
			{
				zmp_preview[0][i] = temp_pn[0];
				zmp_preview[1][i] = temp_pn[1];
			}else if(i - (step_basic_frame*3 - current_frame_count) - ds_frame*2 < ds_frame){
				double ttpn[2] = {0};
				if(ikid_start_walk_flag){
					ttpn[0] = temp_pn[0] + cos(theta) * sx + (-sin(theta) * sy * (-1) * pow(-1, !isLeft));
					ttpn[1] = temp_pn[1] + sin(theta) * sx + (cos(theta) * sy * (-1) * pow(-1, !isLeft));
				}else{
					ttpn[0] = temp_pn[0] + cos(theta) * sx + (-sin(theta) * sy * (-1) * pow(-1, isLeft));
					ttpn[1] = temp_pn[1] + sin(theta) * sx + (cos(theta) * sy * (-1) * pow(-1, isLeft));
				}
				zmp_preview[0][i] = temp_pn[0] + (i - (step_basic_frame*3 - current_frame_count) - ds_frame*2)*(ttpn[0] - temp_pn[0]) / ds_frame;
				zmp_preview[1][i] = temp_pn[1] + (i - (step_basic_frame*3 - current_frame_count) - ds_frame*2)*(ttpn[1] - temp_pn[1]) / ds_frame;
			}else{
				double ttpn[2] = {0};
				if(ikid_start_walk_flag){
					ttpn[0] = temp_pn[0] + cos(theta) * sx + (-sin(theta) * sy * (-1) * pow(-1, !isLeft));
					ttpn[1] = temp_pn[1] + sin(theta) * sx + (cos(theta) * sy * (-1) * pow(-1, !isLeft));
				}else{
					ttpn[0] = temp_pn[0] + cos(theta) * sx + (-sin(theta) * sy * (-1) * pow(-1, isLeft));
					ttpn[1] = temp_pn[1] + sin(theta) * sx + (cos(theta) * sy * (-1) * pow(-1, isLeft));
				}
				zmp_preview[0][i] = ttpn[0];
				zmp_preview[1][i] = ttpn[1];
			}
		}
		else
		{
			if (ds_frame - current_frame_count > i) {
				zmp_preview[0][i] = support_ZMP[0] + (current_frame_count + i) * (pn[0] - support_ZMP[0]) / ds_frame;
				zmp_preview[1][i] = support_ZMP[1] + (current_frame_count + i) * (pn[1] - support_ZMP[1]) / ds_frame;
			}
			else if (i - (ds_frame - current_frame_count) < step_basic_frame) {
				zmp_preview[0][i] = pn[0];
				zmp_preview[1][i] = pn[1];
			}
			else if (i - (ds_frame - current_frame_count)- step_basic_frame  < ds_frame) {
				zmp_preview[0][i] = pn[0] + (i - (ds_frame - current_frame_count) - step_basic_frame) * (temp_pn[0] - pn[0]) / ds_frame;
				zmp_preview[1][i] = pn[1] + (i - (ds_frame - current_frame_count) - step_basic_frame) * (temp_pn[1] - pn[1]) / ds_frame;
			}
			else if(i - (ds_frame - current_frame_count)- step_basic_frame - ds_frame < step_basic_frame)
			{
				zmp_preview[0][i] = temp_pn[0];
				zmp_preview[1][i] = temp_pn[1];
			}else if(i - (ds_frame*2 - current_frame_count)- step_basic_frame*2 < ds_frame){
				double ttpn[2] = {0};
				if(ikid_start_walk_flag){
					ttpn[0] = temp_pn[0] + cos(theta) * sx + (-sin(theta) * sy * (-1) * pow(-1, !isLeft));
					ttpn[1] = temp_pn[1] + sin(theta) * sx + (cos(theta) * sy * (-1) * pow(-1, !isLeft));
				}else{
					ttpn[0] = temp_pn[0] + cos(theta) * sx + (-sin(theta) * sy * (-1) * pow(-1, isLeft));
					ttpn[1] = temp_pn[1] + sin(theta) * sx + (cos(theta) * sy * (-1) * pow(-1, isLeft));
				}
				zmp_preview[0][i] = temp_pn[0] + (i - (ds_frame - current_frame_count)- step_basic_frame - ds_frame - step_basic_frame)*(ttpn[0] - temp_pn[0]) / ds_frame;
				zmp_preview[1][i] = temp_pn[1] + (i - (ds_frame - current_frame_count)- step_basic_frame - ds_frame - step_basic_frame)*(ttpn[1] - temp_pn[1]) / ds_frame;
			}else{
				double ttpn[2] = {0};
				if(ikid_start_walk_flag){
					ttpn[0] = temp_pn[0] + cos(theta) * sx + (-sin(theta) * sy * (-1) * pow(-1, !isLeft));
					ttpn[1] = temp_pn[1] + sin(theta) * sx + (cos(theta) * sy * (-1) * pow(-1, !isLeft));
				}else{
					ttpn[0] = temp_pn[0] + cos(theta) * sx + (-sin(theta) * sy * (-1) * pow(-1, isLeft));
					ttpn[1] = temp_pn[1] + sin(theta) * sx + (cos(theta) * sy * (-1) * pow(-1, isLeft));
				}
				zmp_preview[0][i] = ttpn[0];
				zmp_preview[1][i] = ttpn[1];
			}
		}
	}
	
	double zmp_x = state_space_C[0] * state_space_Com[0][0] + state_space_C[1] * state_space_Com[0][1] + state_space_C[2] * state_space_Com[0][2];
	double zmp_y = state_space_C[0] * state_space_Com[1][0] + state_space_C[1] * state_space_Com[1][1] + state_space_C[2] * state_space_Com[1][2];
	// double zmp_x = Compute_fact_zmp[0];
	// double zmp_y = Compute_fact_zmp[1];
	sum_e[0] = sum_e[0] + zmp_x - zmp_preview[0][0];
	sum_e[1] = sum_e[1] + zmp_y - zmp_preview[1][0];
	writeZmpData(zmp_preview, zmp_preview[0][0],zmp_preview[1][0],zmp_x,zmp_y,Compute_fact_zmp[0],Compute_fact_zmp[1]);
	current_ZMP_point[0] = zmp_preview[0][0];
	current_ZMP_point[1] = zmp_preview[1][0];
	current_ZMP_point[2] = 0;
	double FMultiZmpPre[2] = { 0 };
	for (int i = 0; i < N_preview; i++)
	{
		FMultiZmpPre[0] = FMultiZmpPre[0] + zmp_weight_f[i] * zmp_preview[0][i];
		FMultiZmpPre[1] = FMultiZmpPre[1] + zmp_weight_f[i] * zmp_preview[1][i];
	}
	double u_x = -ks * sum_e[0] - (kx[0] * state_space_Com[0][0] + kx[1] * state_space_Com[0][1] + kx[2] * state_space_Com[0][2])- FMultiZmpPre[0];
	double u_y = -ks * sum_e[1] - (kx[0] * state_space_Com[1][0] + kx[1] * state_space_Com[1][1] + kx[2] * state_space_Com[1][2])- FMultiZmpPre[1];
	state_space_Com[0][0] = state_space_A[0][0] * state_space_Com[0][0] + state_space_A[0][1] * state_space_Com[0][1] + state_space_A[0][2] * state_space_Com[0][2] + state_space_B[0] * u_x;
	state_space_Com[0][1] = state_space_A[1][0] * state_space_Com[0][0] + state_space_A[1][1] * state_space_Com[0][1] + state_space_A[1][2] * state_space_Com[0][2] + state_space_B[1] * u_x;
	state_space_Com[0][2] = state_space_A[2][0] * state_space_Com[0][0] + state_space_A[2][1] * state_space_Com[0][1] + state_space_A[2][2] * state_space_Com[0][2] + state_space_B[2] * u_x;
	state_space_Com[1][0] = state_space_A[0][0] * state_space_Com[1][0] + state_space_A[0][1] * state_space_Com[1][1] + state_space_A[0][2] * state_space_Com[1][2] + state_space_B[0] * u_y;
	state_space_Com[1][1] = state_space_A[1][0] * state_space_Com[1][0] + state_space_A[1][1] * state_space_Com[1][1] + state_space_A[1][2] * state_space_Com[1][2] + state_space_B[1] * u_y;
	state_space_Com[1][2] = state_space_A[2][0] * state_space_Com[1][0] + state_space_A[2][1] * state_space_Com[1][1] + state_space_A[2][2] * state_space_Com[1][2] + state_space_B[2] * u_y;
	Com[0] = state_space_Com[0][0] + com_const_x_compen;
	Com[1] = state_space_Com[1][0] + com_const_y_compen;
}

void dFootSupportPhase(double theta_mainbody, double theta_left, double theta_right)
{
	isDsPhase = true;
	double solid_left_foot[3] = { robotModel[LEFT_FOOT].p[0],robotModel[LEFT_FOOT].p[1],robotModel[LEFT_FOOT].p[2] };
	double solid_right_foot[3] = { robotModel[RIGHT_FOOT].p[0],robotModel[RIGHT_FOOT].p[1],robotModel[RIGHT_FOOT].p[2] };
	double delta_roll = 0;
	double delta_pitch = 0;
	double delta_yaw = 0;
	for (int i = 0; i < ds_frame; i++)
	{
		//准备PID修正
		imuGesturePidControl(delta_roll, delta_pitch, delta_yaw);
		waistPosition_com(delta_roll, delta_pitch, theta_mainbody, i);
		double R[3][3];
		double temp[3];
		rpy2rot(0, 0, theta_right, R);
		MatrixMultiVector3x1(R, robotModel[RIGHT_FOOT].b, temp);
		robotModel[RIGHT_ANKLE_SIDE_SWING].p[0] = solid_right_foot[0] - temp[0];
		robotModel[RIGHT_ANKLE_SIDE_SWING].p[1] = solid_right_foot[1] - temp[1];
		robotModel[RIGHT_ANKLE_SIDE_SWING].p[2] = solid_right_foot[2] - temp[2];
		inverseKinmatics_rightFoot(0, 0, theta_right);
		rpy2rot(0, 0, theta_left, R);
		MatrixMultiVector3x1(R, robotModel[LEFT_FOOT].b, temp);
		robotModel[LEFT_ANKLE_SIDE_SWING].p[0] = solid_left_foot[0] - temp[0];
		robotModel[LEFT_ANKLE_SIDE_SWING].p[1] = solid_left_foot[1] - temp[1];
		robotModel[LEFT_ANKLE_SIDE_SWING].p[2] = solid_left_foot[2] - temp[2];
		inverseKinmatics_leftFoot(0, 0, theta_left);

		Calc_ZMP(Compute_fact_zmp,&robot_taoz);

		#if PID_AMEND
		robotModel[RIGHT_ANKLE_SIDE_SWING].q -= delta_roll;
		robotModel[LEFT_ANKLE_SIDE_SWING].q -= delta_roll;
		robotModel[RIGHT_ANKLE_FRONT_SWING].q -= delta_pitch;
		robotModel[LEFT_ANKLE_FRONT_SWING].q -= delta_pitch;
		forwardKinematics(MAIN_BODY);
		#endif
#if WRITETXT
	writeTxt();
#endif
#if ROSPUB
	ikidRobotDynaPosPub();
#endif
#if CONTROLBOARDPUB
	ikidRobotDynaPosControlBoardPub();

#endif
	}
}

void imuGesturePidControl(double &delta_roll, double &delta_pitch, double &delta_yaw){
	std_msgs::Float64 msg;
	double data_roll=0,data_pitch=0,data_yaw=0;
	//ros::param::get("imu_data_roll",data_roll);
	std::fstream fin;
	fin.open("/home/wp/ikid_ws/imubuffer.txt", std::ios::in);
	fin >> imu_data_roll >> imu_data_pitch >> imu_data_yaw;
	fin.close();
	// printf("imu_data_roll: %f\n", imu_data_roll);
	// printf("imu_data_pitch: %f\n", imu_data_pitch);
	// printf("imu_data_yaw: %f\n", imu_data_yaw);
	data_roll = imu_data_roll;
	data_roll -= init_imu_roll;
	msg.data = data_roll;
	
	pub_imu_data_roll.publish(msg);
	//ros::param::get("imu_data_pitch",data_pitch);
	data_pitch = imu_data_pitch;
	data_pitch -= init_imu_pitch;
	msg.data = data_pitch;
	pub_imu_data_pitch.publish(msg);
	//ros::param::get("imu_data_yaw",data_yaw);
	data_yaw = imu_data_yaw;
	msg.data = data_yaw;
	pub_imu_data_yaw.publish(msg);
	//printf("%f, %f\n",data_roll, data_pitch);
	
	writeImuData();
	
	//单位是度
	#if PID_AMEND
	double temp_roll_err = 0;
	double temp_pitch_err = 0;
	double temp_yaw_err = 0;
	temp_roll_err = stable_roll - data_roll;
	imu_roll_err_partial = (temp_roll_err - imu_roll_err)/frame_T;
	imu_roll_err = temp_roll_err;
	imu_roll_err_sum += imu_roll_err*frame_T;

	temp_pitch_err = stable_pitch - data_pitch;
	imu_pitch_err_partial = (temp_pitch_err - imu_pitch_err)/frame_T;
	imu_pitch_err = temp_pitch_err;
	imu_pitch_err_sum += imu_pitch_err*frame_T;

	//stable_yaw = theta; 目前不对偏航角用PID
	stable_yaw = data_yaw;
	temp_yaw_err = stable_yaw - data_yaw;
	imu_yaw_err_partial = (temp_yaw_err - imu_yaw_err)/frame_T;
	imu_yaw_err = temp_yaw_err;
	imu_yaw_err_sum += imu_yaw_err*frame_T;
	//printf("%f,%f,%f\n", imu_pitch_err, imu_pitch_err_sum, imu_pitch_err_partial);

	delta_roll = imu_roll_p*imu_roll_err + imu_roll_i*imu_roll_err_sum + imu_roll_d*imu_roll_err_partial;
	delta_pitch = imu_pitch_p*imu_pitch_err + imu_pitch_i*imu_pitch_err_sum + imu_pitch_d*imu_pitch_err_partial;
	delta_yaw = imu_yaw_p*imu_yaw_err + imu_yaw_i*imu_yaw_err_sum + imu_yaw_d*imu_yaw_err_partial;

	// 质心位置补偿
	com_y_compen = -imu_com_y_p*imu_roll_err + -imu_com_y_i*imu_roll_err_sum + -imu_com_y_d*imu_roll_err_partial;
	//printf("com_y_compen: %f\n", com_y_compen);
	com_x_compen = -imu_com_x_p*imu_pitch_err + -imu_com_x_i*imu_pitch_err_sum + -imu_com_x_d*imu_pitch_err_partial;
	//printf("com_x_compen: %f\n", com_x_compen);
	// 分配到关节
	ROS_INFO("3333");
	if(abs(delta_roll) > 5){
		if(delta_roll > 0) delta_roll = 5;
		else delta_roll = -5;
	}
	if(abs(delta_pitch) > 5){
		if(delta_pitch > 0) delta_pitch = 5;
		else delta_pitch = -5;
	}
	if(abs(delta_yaw) > 5){
		if(delta_yaw > 0) delta_yaw = 5;
		else delta_yaw = -5;
	}
	if(abs(com_y_compen) > 0.02){
		if(com_y_compen > 0) com_y_compen = 0.02;
		else com_y_compen = -0.02;
	}
	if(abs(com_x_compen) > 0.02){
		if(com_x_compen > 0) com_x_compen = 0.02;
		else com_x_compen = -0.02;
	}
	
	// 转换为弧度
	delta_roll = delta_roll/180*M_PI;
	delta_pitch = delta_pitch/180*M_PI;
	delta_yaw = delta_yaw/180*M_PI;
	delta_roll /= 2;
	delta_pitch /= 2;
	delta_yaw /= 2;
	#endif
	
}

void specialGaitExec(int id){
	
	DIR *dp = NULL;
	struct dirent *st;  // 文件夹中的子文件数据结构
	struct stat sta;
	int ret = 0;
	char tmp_name[1024] = {0};
	char path[50] = "/home/wp/ikid_ws/specialGaitFile\0";
	dp = opendir(path);
	if (dp == NULL)
	{
		printf("open dir error!!\n");
		return;
	}

	// 保证当前特殊步态是在上一个特殊步态执行完毕后执行
	bool temp_stop_special_gait_flag = true;
	ros::param::get("stop_special_gait_flag", temp_stop_special_gait_flag);
    while(!temp_stop_special_gait_flag){
        ros::param::get("stop_special_gait_flag", temp_stop_special_gait_flag);
    }

	ros::param::set("stop_special_gait_flag", false);
	ros::Duration(2).sleep();
	while (1)
	{
		st = readdir(dp);
		if (NULL == st) // 读取完毕
		{
			break;
		}
		strcpy(tmp_name, path);
		if (path[strlen(path) - 1] != '/') // 判断路径名是否带/
			strcat(tmp_name, "/");
		strcat(tmp_name, st->d_name); // 新文件路径名

		//获取文件中步态
		char c[300];
		char *f_ret;
		FILE *fptr = fopen(tmp_name, "r");
		if (fgets(c,sizeof(c),fptr) != NULL)
		{
			//ROS_INFO("%s",tmp_name);
			f_ret = fgets(c,sizeof(c),fptr);
			if(c[strlen(c)-1]=='\n') c[strlen(c)-1] = '\0';
			int temp_id = atoi(c);
			if (temp_id == id){
				//获取步态频率
				f_ret = fgets(c,sizeof(c),fptr);
				if(c[strlen(c)-1]=='\r') c[strlen(c)-1] = '\0'; // 由于windows和linux文本文件的换行符规则不同，这里统一消去
				if(c[strlen(c)-2]=='\r') c[strlen(c)-2] = '\0';
				if(c[strlen(c)-1]=='\n') c[strlen(c)-1] = '\0';
				if(c[strlen(c)-2]=='\n') c[strlen(c)-2] = '\0';
				if(strcmp(c,"GaitRate") != 0) break;
				f_ret = fgets(c,sizeof(c),fptr);
				if(c[strlen(c)-1]=='\r') c[strlen(c)-1] = '\0';
				if(c[strlen(c)-2]=='\r') c[strlen(c)-2] = '\0';
				if(c[strlen(c)-1]=='\n') c[strlen(c)-1] = '\0';
				if(c[strlen(c)-2]=='\n') c[strlen(c)-2] = '\0';
				int gaitRate = atoi(c);

				//读出步态描述
				f_ret = fgets(c,sizeof(c),fptr);
				if(c[strlen(c)-1]=='\r') c[strlen(c)-1] = '\0';
				if(c[strlen(c)-2]=='\r') c[strlen(c)-2] = '\0';
				if(c[strlen(c)-1]=='\n') c[strlen(c)-1] = '\0';
				if(c[strlen(c)-2]=='\n') c[strlen(c)-2] = '\0';
				if(strcmp(c,"GaitDescription") != 0) break;
				f_ret = fgets(c,sizeof(c),fptr);
				
				//读取步态零点
				f_ret = fgets(c,sizeof(c),fptr);
				if(c[strlen(c)-1]=='\r') c[strlen(c)-1] = '\0';
				if(c[strlen(c)-2]=='\r') c[strlen(c)-2] = '\0';
				if(c[strlen(c)-1]=='\n') c[strlen(c)-1] = '\0';
				if(c[strlen(c)-2]=='\n') c[strlen(c)-2] = '\0';
				if(strcmp(c,"zero_point") != 0) break;
				f_ret = fgets(c,sizeof(c),fptr);
				if(c[strlen(c)-1]=='\r') c[strlen(c)-1] = '\0';
				if(c[strlen(c)-2]=='\r') c[strlen(c)-2] = '\0';
				if(c[strlen(c)-1]=='\n') c[strlen(c)-1] = '\0';
				if(c[strlen(c)-2]=='\n') c[strlen(c)-2] = '\0';
				char* token;
                const char spl_chara[2] = ",";
				double zero_point[26] = {0};
                token = strtok(c,spl_chara);
                if(token != NULL){
					zero_point[1] = atof(token);
                    for (int i = 2; i <= 25; i++)
                    {
                        token = strtok(NULL, ",");
                        zero_point[i] = atof(token);
                    }
					printf("\n");
                }

				//读取步态数据并发布
				f_ret = fgets(c,sizeof(c),fptr);
				if(c[strlen(c)-1]=='\r') c[strlen(c)-1] = '\0';
				if(c[strlen(c)-2]=='\r') c[strlen(c)-2] = '\0';
				if(c[strlen(c)-1]=='\n') c[strlen(c)-1] = '\0';
				if(c[strlen(c)-2]=='\n') c[strlen(c)-2] = '\0';
				if(strcmp(c,"Gait_Frame") != 0) break;
				double before_gait_frame_data[26] = {0};  // 使用before数组是为了保持动作的连贯，如果直接用当前机器人的关节角，会有误差，造成顿感
				double gait_frame_data[26] = {0};
				int count_frame = 1;
				while (fgets(c,sizeof(c),fptr) != NULL)
				{
					if(c[strlen(c)-1]=='\r') c[strlen(c)-1] = '\0';
					if(c[strlen(c)-2]=='\r') c[strlen(c)-2] = '\0';
					if(c[strlen(c)-1]=='\n') c[strlen(c)-1] = '\0';
					if(c[strlen(c)-2]=='\n') c[strlen(c)-2] = '\0';
					token = strtok(c,spl_chara);
                	if(token != NULL){
						if(count_frame !=1 ){
							for (int i = 1; i <= 25; i++)
							{
								before_gait_frame_data[i] = gait_frame_data[i];
							}
						}
						gait_frame_data[1] = (atof(token)+zero_point[1])/180*M_PI;
						// gait_frame_data[1] = (atof(token))/180*M_PI;
						for (int i = 2; i <= 25; i++)
						{
							token = strtok(NULL, ",");
							gait_frame_data[i] = (atof(token)+zero_point[i])/180*M_PI;
							// gait_frame_data[i] = (atof(token))/180*M_PI;
						}
						token = strtok(NULL, ","); // 获取当前帧到下一帧之间的插帧数
						int temp_frame_rate = atoi(token);
						if(count_frame == 1){
							for (int i = 1; i <= 25; i++)
							{
								before_gait_frame_data[i] = gait_frame_data[i];
							}
						}
						for(int i = 1; i <= temp_frame_rate; i++){
							for (int j = 1; j <= 25; j++)
							{
								if(count_frame == 1){
									robotModel[j].q = robotModel[j].q+(gait_frame_data[j]-robotModel[j].q)*i/temp_frame_rate;
								}else{
									robotModel[j].q = before_gait_frame_data[j]+(gait_frame_data[j]-before_gait_frame_data[j])*i/temp_frame_rate;
								}
							}
							#if ROSPUB
							ikidRobotDynaPosPub();
							#endif
							#if CONTROLBOARDPUB
							ikidRobotDynaPosControlBoardPubSpecialGait();
							ros::Duration(0.02).sleep();
							#endif
							count_frame++;
						}
                	}

				}
				fclose(fptr);
				break;
			}
			else{
				fclose(fptr);
				continue;
			}
		}
	}

	closedir(dp);
	FallUpInitPos();
	ros::Duration(2).sleep();
	ros::param::set("stop_special_gait_flag", true);
}


void judgeFall(){
	//判断机器人是否跌倒,并执行步态
	double data_roll=0.0,data_pitch=0.0,data_yaw=0.0;
	std::fstream fin;
	fin.open("/home/wp/ikid_ws/imubuffer.txt", std::ios::in);
	fin >> data_roll >> data_pitch >> data_yaw;
	fin.close();
	//ros::param::get("imu_data_roll",data_roll);
	data_roll -= init_imu_roll;
	//ros::param::get("imu_data_pitch",data_pitch);
	data_pitch -= init_imu_pitch;
	//ros::param::get("imu_data_yaw",data_yaw);
	if(data_pitch < 110 && data_pitch > 70){
		double temp_sx = sx;
		sx = 0;   // 执行跌倒爬起时，一定要先把前进步长变为0走一步，要不然复位时之前保存的ZMP点可能已经不相对存在于脚底板下
		trajPlan();
		ros::param::set("stop_walk_flag",true);
		specialGaitExec(FALL_FORWARD_UP_ID);
		sx = temp_sx;
	}else if(data_pitch > -110 && data_pitch < -70){
		double temp_sx = sx;
		sx = 0;
		trajPlan();
		ros::param::set("stop_walk_flag",true);
		specialGaitExec(FALL_BACK_UP_ID);
		sx = temp_sx;
	}
}

void FallUpInitPos(){
	imu_roll_err = 0;
	imu_roll_err_sum = 0;
	imu_roll_err_partial = 0;
	imu_pitch_err = 0;
	imu_pitch_err_sum = 0;
	imu_pitch_err_partial = 0;
	com_x_compen = 0;
	com_y_compen = 0;
	pre_robot_P[0] = 0;pre_robot_P[1] = 0;pre_robot_P[2] = 0;
	cur_robot_P[0] = 0;cur_robot_P[1] = 0;cur_robot_P[2] = 0;
	pre_robot_L[0] = 0;pre_robot_L[1] = 0;pre_robot_L[2] = 0;
	cur_robot_L[0] = 0;cur_robot_L[1] = 0;cur_robot_L[2] = 0;
	robot_dPdt[0] = 0;robot_dPdt[1] = 0;robot_dPdt[2] = 0;
	robot_dLdt[0] = 0;robot_dLdt[1] = 0;robot_dLdt[2] = 0;
	state_space_Com[0][1] += -state_space_Com[0][1]*0.25; //微调，为了再次稳定的起步
	state_space_Com[0][2] += -state_space_Com[0][2]*0.25;
	state_space_Com[1][1] += -state_space_Com[1][1]*0.25;
	state_space_Com[1][2] += -state_space_Com[1][2]*0.25;

#if ROSPUB
	for(int i = 0; i < 21; i++){
		std_msgs::Float64 msg;
		ros::Rate ikidPubRate(20);
		msg.data = ikid_robot_zero_point[FRONT_NECK_SWING] + robotModel[FRONT_NECK_SWING].q + (FallUpRobotPos_q[FRONT_NECK_SWING] - robotModel[FRONT_NECK_SWING].q)/20*i;
		pub_neck_front_swing.publish(msg);
		msg.data = ikid_robot_zero_point[NECK_ROTATION] + robotModel[NECK_ROTATION].q + (FallUpRobotPos_q[NECK_ROTATION] - robotModel[NECK_ROTATION].q)/20*i;
		pub_neck_rotation.publish(msg);
		msg.data = ikid_robot_zero_point[LEFT_ARM_FRONT_SWING] + robotModel[LEFT_ARM_FRONT_SWING].q + (FallUpRobotPos_q[LEFT_ARM_FRONT_SWING] - robotModel[LEFT_ARM_FRONT_SWING].q)/20*i;
		pub_left_arm_front_swing.publish(msg);
		msg.data = ikid_robot_zero_point[LEFT_ARM_SIDE_SWING] + robotModel[LEFT_ARM_SIDE_SWING].q + (FallUpRobotPos_q[LEFT_ARM_SIDE_SWING] - robotModel[LEFT_ARM_SIDE_SWING].q)/20*i;
		pub_left_arm_side_swing.publish(msg);
		msg.data = ikid_robot_zero_point[LEFT_ARM_ELBOW_FRONT_SWING] + robotModel[LEFT_ARM_ELBOW_FRONT_SWING].q + (FallUpRobotPos_q[LEFT_ARM_ELBOW_FRONT_SWING] - robotModel[LEFT_ARM_ELBOW_FRONT_SWING].q)/20*i;
		pub_left_arm_elbow_front_swing.publish(msg);
		msg.data = ikid_robot_zero_point[RIGHT_ARM_FRONT_SWING] + robotModel[RIGHT_ARM_FRONT_SWING].q + (FallUpRobotPos_q[RIGHT_ARM_FRONT_SWING] - robotModel[RIGHT_ARM_FRONT_SWING].q)/20*i;
		pub_right_arm_front_swing.publish(msg);
		msg.data = ikid_robot_zero_point[RIGHT_ARM_SIDE_SWING] + robotModel[RIGHT_ARM_SIDE_SWING].q + (FallUpRobotPos_q[RIGHT_ARM_SIDE_SWING] - robotModel[RIGHT_ARM_SIDE_SWING].q)/20*i;
		pub_right_arm_side_swing.publish(msg);
		msg.data = ikid_robot_zero_point[RIGHT_ARM_ELBOW_FRONT_SWING] + robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].q + (FallUpRobotPos_q[RIGHT_ARM_ELBOW_FRONT_SWING] - robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].q)/20*i;
		pub_right_arm_elbow_front_swing.publish(msg);
		msg.data = ikid_robot_zero_point[LEFT_HIP_FRONT_SWING] + robotModel[LEFT_HIP_FRONT_SWING].q + (FallUpRobotPos_q[LEFT_HIP_FRONT_SWING] - robotModel[LEFT_HIP_FRONT_SWING].q)/20*i;
		pub_left_hip_front_swing.publish(msg);
		msg.data = ikid_robot_zero_point[LEFT_HIP_SIDE_SWING] + robotModel[LEFT_HIP_SIDE_SWING].q + (FallUpRobotPos_q[LEFT_HIP_SIDE_SWING] - robotModel[LEFT_HIP_SIDE_SWING].q)/20*i;
		pub_left_hip_side_swing.publish(msg);
		msg.data = ikid_robot_zero_point[LEFT_HIP_ROTATION] + robotModel[LEFT_HIP_ROTATION].q + (FallUpRobotPos_q[LEFT_HIP_ROTATION] - robotModel[LEFT_HIP_ROTATION].q)/20*i;
		pub_left_hip_rotation.publish(msg);
		msg.data = ikid_robot_zero_point[RIGHT_HIP_FRONT_SWING] + robotModel[RIGHT_HIP_FRONT_SWING].q + (FallUpRobotPos_q[RIGHT_HIP_FRONT_SWING] - robotModel[RIGHT_HIP_FRONT_SWING].q)/20*i;
		pub_right_hip_front_swing.publish(msg);
		msg.data = ikid_robot_zero_point[RIGHT_HIP_SIDE_SWING] + robotModel[RIGHT_HIP_SIDE_SWING].q + (FallUpRobotPos_q[RIGHT_HIP_SIDE_SWING] - robotModel[RIGHT_HIP_SIDE_SWING].q)/20*i;
		pub_right_hip_side_swing.publish(msg);
		msg.data = ikid_robot_zero_point[RIGHT_HIP_ROTATION] + robotModel[RIGHT_HIP_ROTATION].q + (FallUpRobotPos_q[RIGHT_HIP_ROTATION] - robotModel[RIGHT_HIP_ROTATION].q)/20*i;
		pub_right_hip_rotation.publish(msg);
		msg.data = ikid_robot_zero_point[LEFT_KNEE_FRONT_SWING] + robotModel[LEFT_KNEE_FRONT_SWING].q + (FallUpRobotPos_q[LEFT_KNEE_FRONT_SWING] - robotModel[LEFT_KNEE_FRONT_SWING].q)/20*i;
		pub_left_knee_front_swing.publish(msg);
		msg.data = ikid_robot_zero_point[RIGHT_KNEE_FRONT_SWING] + robotModel[RIGHT_KNEE_FRONT_SWING].q + (FallUpRobotPos_q[RIGHT_KNEE_FRONT_SWING] - robotModel[RIGHT_KNEE_FRONT_SWING].q)/20*i;
		pub_right_knee_front_swing.publish(msg);
		msg.data = ikid_robot_zero_point[LEFT_ANKLE_FRONT_SWING] + robotModel[LEFT_ANKLE_FRONT_SWING].q + (FallUpRobotPos_q[LEFT_ANKLE_FRONT_SWING] - robotModel[LEFT_ANKLE_FRONT_SWING].q)/20*i;
		pub_left_ankle_front_swing.publish(msg);
		msg.data = ikid_robot_zero_point[LEFT_ANKLE_SIDE_SWING] + robotModel[LEFT_ANKLE_SIDE_SWING].q + (FallUpRobotPos_q[LEFT_ANKLE_SIDE_SWING] - robotModel[LEFT_ANKLE_SIDE_SWING].q)/20*i;
		pub_left_ankle_side_swing.publish(msg);
		msg.data = ikid_robot_zero_point[RIGHT_ANKLE_FRONT_SWING] + robotModel[RIGHT_ANKLE_FRONT_SWING].q + (FallUpRobotPos_q[RIGHT_ANKLE_FRONT_SWING] - robotModel[RIGHT_ANKLE_FRONT_SWING].q)/20*i;
		pub_right_ankle_front_swing.publish(msg);
		msg.data = ikid_robot_zero_point[RIGHT_ANKLE_SIDE_SWING] + robotModel[RIGHT_ANKLE_SIDE_SWING].q + (FallUpRobotPos_q[RIGHT_ANKLE_SIDE_SWING] - robotModel[RIGHT_ANKLE_SIDE_SWING].q)/20*i;
		pub_right_ankle_side_swing.publish(msg);
		ikidPubRate.sleep();
	}
#endif
#if CONTROLBOARDPUB
	for(int i = 0; i < 21; i++){
		ikid_motion_control::robot_joint control_board_joint_msg;
		control_board_joint_msg.joint = {
			0,
			0,
			ikid_robot_zero_point[FRONT_NECK_SWING] + robotModel[FRONT_NECK_SWING].q + (FallUpRobotPos_q[FRONT_NECK_SWING] - robotModel[FRONT_NECK_SWING].q)/20*i,
			ikid_robot_zero_point[NECK_ROTATION] + robotModel[NECK_ROTATION].q + (FallUpRobotPos_q[NECK_ROTATION] - robotModel[NECK_ROTATION].q)/20*i,
			ikid_robot_zero_point[RIGHT_ARM_FRONT_SWING] + robotModel[RIGHT_ARM_FRONT_SWING].q + (FallUpRobotPos_q[RIGHT_ARM_FRONT_SWING] - robotModel[RIGHT_ARM_FRONT_SWING].q)/20*i,
			ikid_robot_zero_point[RIGHT_ARM_SIDE_SWING] + robotModel[RIGHT_ARM_SIDE_SWING].q + (FallUpRobotPos_q[RIGHT_ARM_SIDE_SWING] - robotModel[RIGHT_ARM_SIDE_SWING].q)/20*i,
			ikid_robot_zero_point[RIGHT_ARM_ELBOW_FRONT_SWING] + robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].q + (FallUpRobotPos_q[RIGHT_ARM_ELBOW_FRONT_SWING] - robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].q)/20*i,
			0,
			ikid_robot_zero_point[LEFT_ARM_FRONT_SWING] + robotModel[LEFT_ARM_FRONT_SWING].q + (FallUpRobotPos_q[LEFT_ARM_FRONT_SWING] - robotModel[LEFT_ARM_FRONT_SWING].q)/20*i,
			ikid_robot_zero_point[LEFT_ARM_SIDE_SWING] + robotModel[LEFT_ARM_SIDE_SWING].q + (FallUpRobotPos_q[LEFT_ARM_SIDE_SWING] - robotModel[LEFT_ARM_SIDE_SWING].q)/20*i,
			ikid_robot_zero_point[LEFT_ARM_ELBOW_FRONT_SWING] + robotModel[LEFT_ARM_ELBOW_FRONT_SWING].q + (FallUpRobotPos_q[LEFT_ARM_ELBOW_FRONT_SWING] - robotModel[LEFT_ARM_ELBOW_FRONT_SWING].q)/20*i,
			0,
			ikid_robot_zero_point[RIGHT_HIP_FRONT_SWING] + robotModel[RIGHT_HIP_FRONT_SWING].q + (FallUpRobotPos_q[RIGHT_HIP_FRONT_SWING] - robotModel[RIGHT_HIP_FRONT_SWING].q)/20*i,
			ikid_robot_zero_point[RIGHT_HIP_SIDE_SWING] + robotModel[RIGHT_HIP_SIDE_SWING].q + (FallUpRobotPos_q[RIGHT_HIP_SIDE_SWING] - robotModel[RIGHT_HIP_SIDE_SWING].q)/20*i,
			ikid_robot_zero_point[RIGHT_HIP_ROTATION] + robotModel[RIGHT_HIP_ROTATION].q + (FallUpRobotPos_q[RIGHT_HIP_ROTATION] - robotModel[RIGHT_HIP_ROTATION].q)/20*i,
			ikid_robot_zero_point[RIGHT_KNEE_FRONT_SWING] + robotModel[RIGHT_KNEE_FRONT_SWING].q + (FallUpRobotPos_q[RIGHT_KNEE_FRONT_SWING] - robotModel[RIGHT_KNEE_FRONT_SWING].q)/20*i,
			ikid_robot_zero_point[RIGHT_ANKLE_FRONT_SWING] + robotModel[RIGHT_ANKLE_FRONT_SWING].q + (FallUpRobotPos_q[RIGHT_ANKLE_FRONT_SWING] - robotModel[RIGHT_ANKLE_FRONT_SWING].q)/20*i,
			ikid_robot_zero_point[RIGHT_ANKLE_SIDE_SWING] + robotModel[RIGHT_ANKLE_SIDE_SWING].q + (FallUpRobotPos_q[RIGHT_ANKLE_SIDE_SWING] - robotModel[RIGHT_ANKLE_SIDE_SWING].q)/20*i,
			0,
			ikid_robot_zero_point[LEFT_HIP_FRONT_SWING] + robotModel[LEFT_HIP_FRONT_SWING].q + (FallUpRobotPos_q[LEFT_HIP_FRONT_SWING] - robotModel[LEFT_HIP_FRONT_SWING].q)/20*i,
			ikid_robot_zero_point[LEFT_HIP_SIDE_SWING] + robotModel[LEFT_HIP_SIDE_SWING].q + (FallUpRobotPos_q[LEFT_HIP_SIDE_SWING] - robotModel[LEFT_HIP_SIDE_SWING].q)/20*i,
			ikid_robot_zero_point[LEFT_HIP_ROTATION] + robotModel[LEFT_HIP_ROTATION].q + (FallUpRobotPos_q[LEFT_HIP_ROTATION] - robotModel[LEFT_HIP_ROTATION].q)/20*i,
			ikid_robot_zero_point[LEFT_KNEE_FRONT_SWING] + robotModel[LEFT_KNEE_FRONT_SWING].q + (FallUpRobotPos_q[LEFT_KNEE_FRONT_SWING] - robotModel[LEFT_KNEE_FRONT_SWING].q)/20*i,
			ikid_robot_zero_point[LEFT_ANKLE_FRONT_SWING] + robotModel[LEFT_ANKLE_FRONT_SWING].q + (FallUpRobotPos_q[LEFT_ANKLE_FRONT_SWING] - robotModel[LEFT_ANKLE_FRONT_SWING].q)/20*i,
			ikid_robot_zero_point[LEFT_ANKLE_SIDE_SWING] + robotModel[LEFT_ANKLE_SIDE_SWING].q + (FallUpRobotPos_q[LEFT_ANKLE_SIDE_SWING] - robotModel[LEFT_ANKLE_SIDE_SWING].q)/20*i,
			0
		};
		pub_control_board_joint_msg.publish(control_board_joint_msg);
		ros::Duration(0.02).sleep();
	}
	//在执行完特殊步态后，恢复初始q值
	for(int i = 0; i < 26; i++){
		robotModel[i].q = FallUpRobotPos_q[i];
	}
	ikidRobotDynaPosControlBoardPub();
#endif


}

void writeImuData(){
#if WRITEIMUDATA
	FILE* fp = NULL;
	char ch[100];
	char filename[] = "/home/wp/ikid_ws/imu_rpy_data.txt";
	fp = fopen(filename, "a");
	if(fp == NULL)
	{
		exit(0);
	}
	double data_roll,data_pitch,data_yaw;
	ros::param::get("imu_data_roll",data_roll);
	ros::param::get("imu_data_pitch",data_pitch);
	ros::param::get("imu_data_yaw",data_yaw);

	sprintf(ch, "%lf,%lf,%lf\n",data_roll, data_pitch, data_yaw);
	fputs(ch, fp);
	fclose(fp);
#endif
	return;
}

void clearImuDataTxt(){
	FILE* fp;
	char ch[200];
	char filename[] = "/home/wp/ikid_ws/imu_rpy_data.txt";
	fp = fopen(filename, "w");
	if (fp == NULL)
	{
		exit(0);
	}
	fclose(fp);
	return;
}

void writeZmpData(double zmp_data[2][N_preview], double z_d_x, double z_d_y, double z_p_x, double z_p_y,double z_f_x, double z_f_y){
#if WRITEZMPDATA
	FILE* fp = NULL;
	char ch[100];
	char filename[] = "/home/wp/ikid_ws/zmpxydesire_zmpxyfact_data.txt";
	fp = fopen(filename, "a");
	if(fp == NULL)
	{
		exit(0);
	}
	// for(int i = 0; i < N_preview; i++){
	// 	sprintf(ch, "%lf,",zmp_data[0][i]);
	// 	fputs(ch, fp);
	// }
	// for(int i = 0; i < N_preview-1; i++){
	// 	sprintf(ch, "%lf,",zmp_data[1][i]);
	// 	fputs(ch, fp);
	// }
	// sprintf(ch, "%lf\n",zmp_data[1][N_preview-1]);
	// fputs(ch, fp);
	sprintf(ch, "%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",z_d_x, z_d_y,z_f_x,z_f_y,Com[0],Com[1],robot_taoz);
	fputs(ch, fp);
	fclose(fp);
#endif
	return;
}

void clearZmpDataTxt(){
	FILE* fp;
	char ch[200];
	char filename[] = "/home/wp/ikid_ws/zmpxydesire_zmpxyfact_data.txt";
	fp = fopen(filename, "w");
	if (fp == NULL)
	{
		exit(0);
	}
	fclose(fp);
	return;
}

void quinticPolyInterFour(double A[6][4], double B[6][4], double s){

	// 五个点，分四段进行插值
	double t[5] = {0, step_basic_frame/5.0*2*frame_T, step_basic_frame/5.0*3*frame_T, step_basic_frame/5.0*4*frame_T, step_basic_frame*frame_T};
	//double x1[5] = {s/2, 3/10.0*s, 1/10.0*s, -3/10.0*s, -s/2}; // x方向的位置
	double x1[5] = {s/2, 3/10.0*s, 1/10.0*s, -s/2*1.5, -s/2}; // x方向的位置
	double x2[5] = {0, (x1[1]-x1[0])/(t[1]-t[0]), (x1[2]-x1[1])/(t[2]-t[1]), 0.3*(x1[3]-x1[2])/(t[3]-t[2]), 0 }; // x方向的速度
	double x3[5] = {0, (x2[1]-x2[0])/(t[1]-t[0]), (x2[2]-x2[1])/(t[2]-t[1]), (x2[3]-x2[2])/(t[3]-t[2]), 0 }; // x方向的加速度
	double z1[5] = {0, fh, 3/5.0*fh, 2/5.0*fh, 0}; // z方向的位置
	double z2[5] = {0, 0, (z1[2]-z1[1])/(t[2]-t[1]), (z1[3]-z1[2])/(t[3]-t[2]), 0 };  // z方向的速度
	double z3[5] = {0, 0, (z2[2]-z2[1])/(t[2]-t[1]), (z2[3]-z2[2])/(t[3]-t[2]), 0 };  // z方向的加速度

	for(int i = 0; i < 4; i++){
		double T_arg_inv[6][6];
		double matlab_result[36];
		double T_arg[6][6] = {
			{1,  t[i]   , pow(t[i],2)   , pow(t[i],3)      , pow(t[i],4)      , pow(t[i],5)}      ,
			{0,  1      , 2*t[i]        , 3*pow(t[i],2)    , 4*pow(t[i],3)    , 5*pow(t[i],4)}    ,
			{0,  0      , 2             , 6*t[i]           , 12*pow(t[i],2)   , 20*pow(t[i],3)}   ,
			{1,  t[i+1] , pow(t[i+1],2) , pow(t[i+1],3)    , pow(t[i+1],4)    , pow(t[i+1],5)}    ,
			{0,  1      , 2*t[i+1]      , 3*pow(t[i+1],2)  , 4*pow(t[i+1],3)  , 5*pow(t[i+1],4)}  ,
			{0,  0      , 2             , 6*t[i+1]         , 12*pow(t[i+1],2) , 20*pow(t[i+1],3)} 
		};
		matlab_inv((double*)T_arg, matlab_result);
		for (int j = 0; j < 6; j++)
		{
			for (int k = 0; k < 6; k++) {
				T_arg_inv[j][k] = matlab_result[j * 6 + k];
			}
		}
		double temp_x[6] = {x1[i],x2[i],x3[i],x1[i+1],x2[i+1],x3[i+1]};
		double temp_z[6] = {z1[i],z2[i],z3[i],z1[i+1],z2[i+1],z3[i+1]};
		double A_result[6];
		double B_result[6];
		MatrixMultiVector6x1(T_arg_inv, temp_x, A_result);
		MatrixMultiVector6x1(T_arg_inv, temp_z, B_result);
		for(int j = 0; j < 6; j++){
			A[j][i] = A_result[j];
			B[j][i] = B_result[j];
			//printf("%f  ", A[j][i]);
		}
		//printf("\n");
	}
}

void quinticPolyInterTwo(double A[6][4], double B[6][4], double s){

	// 三个点，分两段进行插值
	double t[5] = {0, step_basic_frame/2*frame_T, step_basic_frame*frame_T};
	double x1[5] = {s/2, 0, -s/2}; // x方向的位置
	double x2[5] = {0, (x1[1]-x1[0])/(t[1]-t[0]), 0 }; // x方向的速度
	double x3[5] = {0, (x2[1]-x2[0])/(t[1]-t[0]), 0 }; // x方向的加速度
	double z1[5] = {0, fh, 0}; // z方向的位置
	double z2[5] = {0, 0, 0 };  // z方向的速度
	double z3[5] = {0, 0, 0 };  // z方向的加速度

	for(int i = 0; i < 2; i++){
		double T_arg_inv[6][6];
		double matlab_result[36];
		double T_arg[6][6] = {
			{1,  t[i]   , pow(t[i],2)   , pow(t[i],3)      , pow(t[i],4)      , pow(t[i],5)}      ,
			{0,  1      , 2*t[i]        , 3*pow(t[i],2)    , 4*pow(t[i],3)    , 5*pow(t[i],4)}    ,
			{0,  0      , 2             , 6*t[i]           , 12*pow(t[i],2)   , 20*pow(t[i],3)}   ,
			{1,  t[i+1] , pow(t[i+1],2) , pow(t[i+1],3)    , pow(t[i+1],4)    , pow(t[i+1],5)}    ,
			{0,  1      , 2*t[i+1]      , 3*pow(t[i+1],2)  , 4*pow(t[i+1],3)  , 5*pow(t[i+1],4)}  ,
			{0,  0      , 2             , 6*t[i+1]         , 12*pow(t[i+1],2) , 20*pow(t[i+1],3)} 
		};
		matlab_inv((double*)T_arg, matlab_result);
		for (int j = 0; j < 6; j++)
		{
			for (int k = 0; k < 6; k++) {
				T_arg_inv[j][k] = matlab_result[j * 6 + k];
			}
		}
		double temp_x[6] = {x1[i],x2[i],x3[i],x1[i+1],x2[i+1],x3[i+1]};
		double temp_z[6] = {z1[i],z2[i],z3[i],z1[i+1],z2[i+1],z3[i+1]};
		double A_result[6];
		double B_result[6];
		MatrixMultiVector6x1(T_arg_inv, temp_x, A_result);
		MatrixMultiVector6x1(T_arg_inv, temp_z, B_result);
		for(int j = 0; j < 6; j++){
			A[j][i] = A_result[j];
			B[j][i] = B_result[j];
			//printf("%f  ", A[j][i]);
		}
		//printf("\n");
	}
}

void threeSplineInter(double spline_A[8][2], double s){
	// 边界条件为固定边界

	double t[3] = {0, step_basic_frame/2*frame_T, step_basic_frame*frame_T};
	double x[3] = {s/2, 0, -s/2}; // x方向的位置
	double z[3] = {0, fh, 0}; // z方向的位置

	double A[8][8] = {{1,      t[0],   pow(t[0],2), pow(t[0],3)   ,0,     0    ,   0           , 0             },  
					  {1,      t[1],   pow(t[1],2), pow(t[1],3)   ,0,     0    ,   0           , 0             }, 
		              {0,      0   ,   0          , 0             ,1,     t[1] ,   pow(t[1],2) , pow(t[1],3)   },
		              {0,      0   ,   0          , 0             ,1,     t[2] ,   pow(t[2],2) , pow(t[2],3)   },
		              {0,      1   ,   2*t[1]     , 3*pow(t[1],2) ,0,     -1   ,   -2*t[1]     , -3*pow(t[1],2)},
		              {0,      0   ,   2          , 6*t[1]        ,0,     0    ,   -2          , -6*t[1]       },    
		              {0,      1   ,   2*t[0]     , 3*pow(t[0],2) ,0,     0    ,   0           , 0             },
		              {0,      0   ,   0          , 0             ,0,     1    ,   2*t[2]      , 3*pow(t[2],2) }};
	double temp_x[8] = {x[0], x[1], x[1], x[2], 0, 0 ,0 ,0};
	double temp_z[8] = {z[0], z[1], z[1], z[2], 0, 0 ,0 ,0};
	double A_inv[8][8];
	double matlab_result[64];
	matlab_inv_8((double*)A,matlab_result);
	for (int j = 0; j < 8; j++)
		{
			for (int k = 0; k < 8; k++) {
				A_inv[j][k] = matlab_result[j * 8 + k];
			}
		}
	double result_x[8];
	double result_z[8];
	MatrixMultiVector8x1(A_inv, temp_x, result_x);
	MatrixMultiVector8x1(A_inv, temp_z, result_z);
	for(int j = 0; j < 8; j++){
		spline_A[j][0] = result_x[j];
		spline_A[j][1] = result_z[j];
	}
	
}






