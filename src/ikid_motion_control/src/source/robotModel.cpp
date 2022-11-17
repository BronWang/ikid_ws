#include "ikid_motion_control/robotModel.h"
#include <math.h>
#include "ikid_motion_control/matlab_inv.h"
#include <string.h>
#include <stdio.h>


robotLink robotModel[26];   // 机器人整体模型数组
static double eye[3][3] = { {1,0,0},{0,1,0},{0,0,1} };

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
double	c_h = 0.3;
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
double support_ZMP[3] = { 0,-sy / 2,0 };
// 质心牵引向量PC_MAIN_BODY
double PC_MAIN_BODY[3] = { 0 };
// 质心世界坐标
double Com[3] = { 0 };
// 实际全身质心与ZMP生成的质心位置误差
double error_Com_ZmpCom[3] = { 0 };
double sum_error_Com_ZmpCom = 0;

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
double zc = c_h;
double dt = frame_T;
double gravity = 9.8;
double state_space_A[3][3] = { {1, dt, dt * dt / 2},
								{0, 1, dt},
								{0, 0,  1}};
double state_space_B[3] = { powf(dt,3) / 6, dt * dt / 2,dt };
double state_space_C[3] = { 1,0,-zc / gravity };
double state_space_Com[2][3] = {0};
double sum_e[2] = { 0 };
const int N_preview = 2 * step_basic_frame + 2 * ds_frame;
// 计算结果
double ks = 509.270683139212;
double kx[3] = { 9901.19795970318,	2023.99459263373,	56.6120346271035 };
double zmp_weight_f[2 * N_preview] = {
	- 667.714646977504, - 786.196342686028, - 817.907469422699, - 781.373266261508, - 709.573090048174, - 628.350099533318, 
	- 551.900303351159, - 485.399832740281, - 429.038650260766, - 381.101682308915, - 339.661194810992, - 303.223257369275, 
	- 270.805393500508, - 241.799723354576, - 215.812183616704, - 192.548493248281, - 171.753755856202, - 153.189375234005, 
	- 136.629074276295, - 121.861533914600, - 108.693379646419, - 96.9504456233559, - 86.4773283501663, - 77.1359073254730, 
	- 68.8034821621016, - 61.3709325913295, - 54.7410800813827, - 48.8272873423575, - 43.5522676666375, - 38.8470602170006, 
	- 34.6501333742970, - 30.9065897292540, - 27.5674559555676, - 24.5890468486326, - 21.9323958939230, - 19.5627460505250, 
	- 17.4490949758736, - 15.5637892411450, - 13.8821624281784, - 12.3822124068436, - 11.0443135396040, - 9.85096000768734, 
	- 8.78653687233856, - 7.83711586203139, - 6.99027321022830, - 6.23492716192785, - 5.56119302619990, - 4.96025388131022, 
	- 4.42424524308183, - 3.94615218909936, - 3.51971759381751, - 3.13936027470157, - 2.80010197903805, - 2.49750225663259, 
	- 2.22760036672922, - 1.98686345946763, - 1.77214035424074, - 1.58062031049487, - 1.40979625179083, - 1.25743196216925
};

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

void robotStart()
{
	robotModelInit(robotModel);
#if WRITETXT
	clearTxt();
#endif
	robotModel[MAIN_BODY].p[0] = 0;
	robotModel[MAIN_BODY].p[1] = 0;
	robotModel[MAIN_BODY].p[2] = 0.3;
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
	robotModel[FRONT_NECK_SWING].dq -= robotModel[FRONT_NECK_SWING].q;
	robotModel[NECK_ROTATION].dq -= robotModel[NECK_ROTATION].q;
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
	robotModel[LEFT_ARM_FRONT_SWING].dq -= robotModel[LEFT_ARM_FRONT_SWING].q;
	robotModel[LEFT_ARM_SIDE_SWING].dq -= robotModel[LEFT_ARM_SIDE_SWING].q;
	robotModel[LEFT_ARM_ELBOW_FRONT_SWING].dq -= robotModel[LEFT_ARM_ELBOW_FRONT_SWING].q;
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
	robotModel[RIGHT_ARM_FRONT_SWING].dq -= robotModel[RIGHT_ARM_FRONT_SWING].q;
	robotModel[RIGHT_ARM_SIDE_SWING].dq -= robotModel[RIGHT_ARM_SIDE_SWING].q;
	robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].dq -= robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].q;
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
	robotModel[LEFT_HIP_FRONT_SWING].dq -= robotModel[LEFT_HIP_FRONT_SWING].q;
	robotModel[LEFT_HIP_SIDE_SWING].dq -= robotModel[LEFT_HIP_SIDE_SWING].q;
	robotModel[LEFT_HIP_ROTATION].dq -= robotModel[LEFT_HIP_ROTATION].q;
	robotModel[LEFT_KNEE_FRONT_SWING].dq -= robotModel[LEFT_KNEE_FRONT_SWING].q;
	robotModel[LEFT_ANKLE_FRONT_SWING].dq -= robotModel[LEFT_ANKLE_FRONT_SWING].q;
	robotModel[LEFT_ANKLE_SIDE_SWING].dq -= robotModel[LEFT_ANKLE_SIDE_SWING].q;
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
	robotModel[RIGHT_HIP_FRONT_SWING].dq -= robotModel[RIGHT_HIP_FRONT_SWING].q;
	robotModel[RIGHT_HIP_SIDE_SWING].dq -= robotModel[RIGHT_HIP_SIDE_SWING].q;
	robotModel[RIGHT_HIP_ROTATION].dq -= robotModel[RIGHT_HIP_ROTATION].q;
	robotModel[RIGHT_KNEE_FRONT_SWING].dq -= robotModel[RIGHT_KNEE_FRONT_SWING].q;
	robotModel[RIGHT_ANKLE_FRONT_SWING].dq -= robotModel[RIGHT_ANKLE_FRONT_SWING].q;
	robotModel[RIGHT_ANKLE_SIDE_SWING].dq -= robotModel[RIGHT_ANKLE_SIDE_SWING].q;
	ForwardVelocity(MAIN_BODY);
}

void clearTxt()
{
	FILE* fp;
	char ch[200];
	char filename[] = "/home/wp/ikid_ws/MOS2018.txt";
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
	char filename[] = "/home/wp/ikid_ws/MOS2018.txt";
	fp = fopen(filename, "a");
	if(fp == NULL)
	{
		exit(0);
	}
	for (int i = 0; i < PART_NUMBER; i++)
	{
		sprintf(ch, "%d,%s,%d,%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
			robotModel[i].linkID, robotModel[i].name, robotModel[i].sister, robotModel[i].child, robotModel[i].mother,
			robotModel[i].p[0], robotModel[i].p[1], robotModel[i].p[2],
			robotModel[i].R[0][0], robotModel[i].R[0][1], robotModel[i].R[0][2],
			robotModel[i].R[1][0], robotModel[i].R[1][1], robotModel[i].R[1][2],
			robotModel[i].R[2][0], robotModel[i].R[2][1], robotModel[i].R[2][2], robotModel[i].q);
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
			// SIN曲线
			double world_p[3];
			double local_p[3];
			double x = CP_norm + (-(i + 1) * 2 * CP_norm / step_basic_frame);
			double y = fh * cos(PI / (2 * CP_norm) * x);
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
			waistPosition_com(0,0,theta,i);


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


#if WRITETXT
			writeTxt();
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
			double x = CP_norm + -(i + 1) * 2 * CP_norm / step_basic_frame;
			double y = fh * cos(PI / (2 * CP_norm) * x);
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
			waistPosition_com(0,0,theta,i);


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


#if WRITETXT
			writeTxt();
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
			double x = CP_norm + (-(i + 1) * 2 * CP_norm / step_basic_frame);
			double y = fh * cos(PI / (2 * CP_norm) * x);
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
			waistPosition_com(0, 0, theta + theta_frame/2,i);


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


#if WRITETXT
			writeTxt();
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
			double x = CP_norm + -(i + 1) * 2 * CP_norm / step_basic_frame;
			double y = fh * cos(PI / (2 * CP_norm) * x);
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
			waistPosition_com(0, 0, theta + theta_frame / 2,i);


			robotModel[RIGHT_ANKLE_SIDE_SWING].p[0] = basic_right_foot[i][0] - temp[0];
			robotModel[RIGHT_ANKLE_SIDE_SWING].p[1] = basic_right_foot[i][1] - temp[1];
			robotModel[RIGHT_ANKLE_SIDE_SWING].p[2] = basic_right_foot[i][2] - temp[2];
			inverseKinmatics_rightFoot(0, 0, theta + theta_frame);

			rpy2rot(0, 0, theta, R);
			MatrixMultiVector3x1(R, robotModel[LEFT_FOOT].b, temp);
			robotModel[LEFT_ANKLE_SIDE_SWING].p[0] = solid_left_foot[0] - temp[0];
			robotModel[LEFT_ANKLE_SIDE_SWING].p[1] = solid_left_foot[1] - temp[1];
			robotModel[LEFT_ANKLE_SIDE_SWING].p[2] = solid_left_foot[2] - temp[2];
			inverseKinmatics_leftFoot(0, 0, theta);


#if WRITETXT
			writeTxt();
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
			double x = CP_norm + (-(i + 1) * 2 * CP_norm / step_basic_frame);
			double y = fh * cos(PI / (2 * CP_norm) * x);
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
			waistPosition_com(0, 0, theta + delta / 2+theta_frame / 2,i);


			robotModel[LEFT_ANKLE_SIDE_SWING].p[0] = basic_left_foot[i][0] - temp[0];
			robotModel[LEFT_ANKLE_SIDE_SWING].p[1] = basic_left_foot[i][1] - temp[1];
			robotModel[LEFT_ANKLE_SIDE_SWING].p[2] = basic_left_foot[i][2] - temp[2];
			inverseKinmatics_leftFoot(0, 0, theta + theta_frame);

			rpy2rot(0, 0, theta + delta, R);
			MatrixMultiVector3x1(R, robotModel[RIGHT_FOOT].b, temp);
			robotModel[RIGHT_ANKLE_SIDE_SWING].p[0] = solid_right_foot[0] - temp[0];
			robotModel[RIGHT_ANKLE_SIDE_SWING].p[1] = solid_right_foot[1] - temp[1];
			robotModel[RIGHT_ANKLE_SIDE_SWING].p[2] = solid_right_foot[2] - temp[2];
			inverseKinmatics_rightFoot(0, 0, theta + delta);


#if WRITETXT
			writeTxt();
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
			double x = CP_norm + -(i + 1) * 2 * CP_norm / step_basic_frame;
			double y = fh * cos(PI / (2 * CP_norm) * x);
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
			waistPosition_com(0, 0, theta + delta / 2 + theta_frame / 2,i);


			robotModel[RIGHT_ANKLE_SIDE_SWING].p[0] = basic_right_foot[i][0] - temp[0];
			robotModel[RIGHT_ANKLE_SIDE_SWING].p[1] = basic_right_foot[i][1] - temp[1];
			robotModel[RIGHT_ANKLE_SIDE_SWING].p[2] = basic_right_foot[i][2] - temp[2];
			inverseKinmatics_rightFoot(0, 0, theta + theta_frame);

			rpy2rot(0, 0, theta + delta, R);
			MatrixMultiVector3x1(R, robotModel[LEFT_FOOT].b, temp);
			robotModel[LEFT_ANKLE_SIDE_SWING].p[0] = solid_left_foot[0] - temp[0];
			robotModel[LEFT_ANKLE_SIDE_SWING].p[1] = solid_left_foot[1] - temp[1];
			robotModel[LEFT_ANKLE_SIDE_SWING].p[2] = solid_left_foot[2] - temp[2];
			inverseKinmatics_leftFoot(0, 0, theta + delta);

#if WRITETXT
			writeTxt();
#endif
		}
	}
	theta = theta + delta;
	dFootSupportPhase(theta, theta, theta);
	changeFoot();
	
}

void CalcTrajectory_Com(int current_frame_count) {
	double zmp_preview[2][N_preview];
	bool temp_isLeft = isLeft;
	double temp_pn[2] = { 0 };
	if (temp_isLeft) {
		temp_isLeft = false;
	}
	else
	{
		temp_isLeft = true;
	}
	temp_pn[0] = pn[0] + cos(theta) * sx + (-sin(theta) * sy * (-1) * pow(-1, temp_isLeft));
	temp_pn[1] = pn[1] + sin(theta) * sx + (cos(theta) * sy * (-1) * pow(-1, temp_isLeft));
	
	for (int i = 0; i < N_preview; i++)
	{
		if (!isDsPhase) {
			if (step_basic_frame - current_frame_count >= i) {
				zmp_preview[0][i] = support_ZMP[0];
				zmp_preview[1][i] = support_ZMP[1];
			}
			else if (i - (step_basic_frame - current_frame_count) <= ds_frame) {
				zmp_preview[0][i] = support_ZMP[0] + (i - (step_basic_frame - current_frame_count)) * (pn[0] - support_ZMP[0]) / ds_frame;
				zmp_preview[1][i] = support_ZMP[1] + (i - (step_basic_frame - current_frame_count)) * (pn[1] - support_ZMP[1]) / ds_frame;
			}
			else if (i - (step_basic_frame - current_frame_count) - ds_frame <= step_basic_frame) {
				zmp_preview[0][i] = pn[0];
				zmp_preview[1][i] = pn[1];
			}
			else if (i - (step_basic_frame - current_frame_count) - ds_frame - step_basic_frame <= ds_frame) {
				zmp_preview[0][i] = pn[0] + (i - (step_basic_frame - current_frame_count) - ds_frame - step_basic_frame) * (temp_pn[0] - pn[0]) / ds_frame;
				zmp_preview[1][i] = pn[1] + (i - (step_basic_frame - current_frame_count) - ds_frame - step_basic_frame) * (temp_pn[1] - pn[1]) / ds_frame;
			}
			else
			{
				zmp_preview[0][i] = temp_pn[0];
				zmp_preview[1][i] = temp_pn[1];
			}
		}
		else
		{
			if (ds_frame - current_frame_count >= i) {
				zmp_preview[0][i] = support_ZMP[0] + (current_frame_count + i) * (pn[0] - support_ZMP[0]) / ds_frame;
				zmp_preview[1][i] = support_ZMP[1] + (current_frame_count + i) * (pn[1] - support_ZMP[1]) / ds_frame;
			}
			else if (i - (ds_frame - current_frame_count) <= step_basic_frame) {
				zmp_preview[0][i] = pn[0];
				zmp_preview[1][i] = pn[1];
			}
			else if (i - (ds_frame - current_frame_count)- step_basic_frame  <= ds_frame) {
				zmp_preview[0][i] = pn[0] + (i - (step_basic_frame - current_frame_count) - step_basic_frame) * (temp_pn[0] - pn[0]) / ds_frame;
				zmp_preview[1][i] = pn[1] + (i - (step_basic_frame - current_frame_count) - step_basic_frame) * (temp_pn[1] - pn[1]) / ds_frame;
			}
			else
			{
				zmp_preview[0][i] = temp_pn[0];
				zmp_preview[1][i] = temp_pn[1];
			}
		}
	}
	
	double zmp_x = state_space_C[0] * state_space_Com[0][0] + state_space_C[1] * state_space_Com[0][1] + state_space_C[2] * state_space_Com[0][2];
	double zmp_y = state_space_C[0] * state_space_Com[1][0] + state_space_C[1] * state_space_Com[1][1] + state_space_C[2] * state_space_Com[1][2];
	sum_e[0] = sum_e[0] + zmp_x - zmp_preview[0][0];
	sum_e[1] = sum_e[1] + zmp_y - zmp_preview[1][0];
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
	Com[0] = state_space_Com[0][0];
	Com[1] = state_space_Com[1][0];
}

void dFootSupportPhase(double theta_mainbody, double theta_left, double theta_right)
{
	isDsPhase = true;
	double solid_left_foot[3] = { robotModel[LEFT_FOOT].p[0],robotModel[LEFT_FOOT].p[1],robotModel[LEFT_FOOT].p[2] };
	double solid_right_foot[3] = { robotModel[RIGHT_FOOT].p[0],robotModel[RIGHT_FOOT].p[1],robotModel[RIGHT_FOOT].p[2] };
	for (int i = 0; i < ds_frame; i++)
	{
		waistPosition_com(0, 0, theta_mainbody, i);
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
		#if WRITETXT
		writeTxt();
		#endif
	}
}

/*/void test()
{
	robotModelInit(robotModel);
#if DEBUG
	printf("初始化完毕。\n");
#endif
	//二叉树先序遍历
	clearTxt();
	forwardKinematics(MAIN_BODY);

	robotModel[LEFT_HAND].p[0] = 0.17;
	robotModel[LEFT_HAND].p[1] = 0.16;
	robotModel[LEFT_HAND].p[2] = 0.35;
	inverseKinmatics_leftHand();
	forwardKinematics(MAIN_BODY);
	writeTxt();
#if DEBUG
	printf("robotmodel LEFT_ARM_FRONT_SWING q = %f\n", robotModel[LEFT_ARM_FRONT_SWING].q / PI * 180);
	printf("robotmodel LEFT_ARM_SIDE_SWING q = %f\n", robotModel[LEFT_ARM_SIDE_SWING].q / PI * 180);
	printf("robotmodel LEFT_ARM_ELBOW_FRONT_SWING q = %f\n", robotModel[LEFT_ARM_ELBOW_FRONT_SWING].q / PI * 180);
#endif

	robotModel[RIGHT_HAND].p[0] = 0.17;
	robotModel[RIGHT_HAND].p[1] = -0.16;
	robotModel[RIGHT_HAND].p[2] = 0.35;
	inverseKinmatics_rightHand();
	forwardKinematics(MAIN_BODY);
	writeTxt();
#if DEBUG
	printf("robotmodel RIGHT_ARM_FRONT_SWING q = %f\n", robotModel[RIGHT_ARM_FRONT_SWING].q / PI * 180);
	printf("robotmodel RIGHT_ARM_SIDE_SWING q = %f\n", robotModel[RIGHT_ARM_SIDE_SWING].q / PI * 180);
	printf("robotmodel RIGHT_ARM_ELBOW_FRONT_SWING q = %f\n", robotModel[RIGHT_ARM_ELBOW_FRONT_SWING].q / PI * 180);
#endif

	robotModel[LEFT_ANKLE_SIDE_SWING].p[0] = 0.1;
	robotModel[LEFT_ANKLE_SIDE_SWING].p[1] = 0.1;
	robotModel[LEFT_ANKLE_SIDE_SWING].p[2] = 0.11;
	inverseKinmatics_leftFoot(0,0,0);
	forwardKinematics(MAIN_BODY);
	writeTxt();
#if DEBUG
	printf("robotmodel LEFT_HIP_FRONT_SWING q = %f\n", robotModel[LEFT_HIP_FRONT_SWING].q / PI * 180);
	printf("robotmodel LEFT_HIP_SIDE_SWING q = %f\n", robotModel[LEFT_HIP_SIDE_SWING].q / PI * 180);
	printf("robotmodel LEFT_HIP_ROTATION q = %f\n", robotModel[LEFT_HIP_ROTATION].q / PI * 180);
	printf("robotmodel LEFT_KNEE_FRONT_SWING q = %f\n", robotModel[LEFT_KNEE_FRONT_SWING].q / PI * 180);
	printf("robotmodel LEFT_ANKLE_FRONT_SWING q = %f\n", robotModel[LEFT_ANKLE_FRONT_SWING].q / PI * 180);
	printf("robotmodel LEFT_ANKLE_SIDE_SWING q = %f\n", robotModel[LEFT_ANKLE_SIDE_SWING].q / PI * 180);
#endif

	robotModel[RIGHT_ANKLE_SIDE_SWING].p[0] = 0.1;
	robotModel[RIGHT_ANKLE_SIDE_SWING].p[1] = -0.1;
	robotModel[RIGHT_ANKLE_SIDE_SWING].p[2] = 0.11;
	inverseKinmatics_rightFoot(0,0,0);
	forwardKinematics(MAIN_BODY);
	writeTxt();
#if DEBUG
	printf("robotmodel RIGHT_HIP_FRONT_SWING q = %f\n", robotModel[RIGHT_HIP_FRONT_SWING].q / PI * 180);
	printf("robotmodel RIGHT_HIP_SIDE_SWING q = %f\n", robotModel[RIGHT_HIP_SIDE_SWING].q / PI * 180);
	printf("robotmodel RIGHT_HIP_ROTATION q = %f\n", robotModel[RIGHT_HIP_ROTATION].q / PI * 180);
	printf("robotmodel RIGHT_KNEE_FRONT_SWING q = %f\n", robotModel[RIGHT_KNEE_FRONT_SWING].q / PI * 180);
	printf("robotmodel RIGHT_ANKLE_FRONT_SWING q = %f\n", robotModel[RIGHT_ANKLE_FRONT_SWING].q / PI * 180);
	printf("robotmodel RIGHT_ANKLE_SIDE_SWING q = %f\n", robotModel[RIGHT_ANKLE_SIDE_SWING].q / PI * 180);
#endif
}*/

/*void test2()
{
	// 起步和前进一步
	robotModelInit(robotModel);
#if DEBUG
	printf("初始化完毕。\n");
#endif
	//二叉树先序遍历
	clearTxt();
	forwardKinematics(MAIN_BODY);
	robotModel[MAIN_BODY].p[0] = 0;
	robotModel[MAIN_BODY].p[1] = 0;
	robotModel[MAIN_BODY].p[2] = 0.3;
	for (int i = 0; i < step_start_frame; i++)
	{
		robotModel[LEFT_HAND].p[0] = start_left_hand[i][0];
		robotModel[LEFT_HAND].p[1] = start_left_hand[i][1];
		robotModel[LEFT_HAND].p[2] = start_left_hand[i][2];
		inverseKinmatics_leftHand();

		robotModel[RIGHT_HAND].p[0] = start_right_hand[i][0];
		robotModel[RIGHT_HAND].p[1] = start_right_hand[i][1];
		robotModel[RIGHT_HAND].p[2] = start_right_hand[i][2];
		inverseKinmatics_rightHand();

		double R[3][3];
		rpy2rot(start_left_foot_angle[i][0], start_left_foot_angle[i][1], start_left_foot_angle[i][2], R);
		double temp[3];
		MatrixMultiVector3x1(R, robotModel[LEFT_FOOT].b, temp);
		robotModel[LEFT_ANKLE_SIDE_SWING].p[0] = start_left_foot[i][0] - temp[0];
		robotModel[LEFT_ANKLE_SIDE_SWING].p[1] = start_left_foot[i][1] - temp[1];
		robotModel[LEFT_ANKLE_SIDE_SWING].p[2] = start_left_foot[i][2] - temp[2];
		inverseKinmatics_leftFoot(start_left_foot_angle[i][0], start_left_foot_angle[i][1], start_left_foot_angle[i][2]);

		rpy2rot(start_right_foot_angle[i][0],start_right_foot_angle[i][1], start_right_foot_angle[i][2] , R);
		MatrixMultiVector3x1(R, robotModel[RIGHT_FOOT].b, temp);
		robotModel[RIGHT_ANKLE_SIDE_SWING].p[0] = start_right_foot[i][0] - temp[0];
		robotModel[RIGHT_ANKLE_SIDE_SWING].p[1] = start_right_foot[i][1] - temp[1];
		robotModel[RIGHT_ANKLE_SIDE_SWING].p[2] = start_right_foot[i][2] - temp[2];
		inverseKinmatics_rightFoot(start_right_foot_angle[i][0], start_right_foot_angle[i][1], start_right_foot_angle[i][2]);
		writeTxt();
	}

	for (int i = 0; i < step_basic_frame; i++)
	{
		robotModel[LEFT_HAND].p[0] = basic_left_hand[i][0];
		robotModel[LEFT_HAND].p[1] = basic_left_hand[i][1];
		robotModel[LEFT_HAND].p[2] = basic_left_hand[i][2];
		inverseKinmatics_leftHand();

		robotModel[RIGHT_HAND].p[0] = basic_right_hand[i][0];
		robotModel[RIGHT_HAND].p[1] = basic_right_hand[i][1];
		robotModel[RIGHT_HAND].p[2] = basic_right_hand[i][2];
		inverseKinmatics_rightHand();

		double R[3][3];
		rpy2rot(basic_left_foot_angle[i][0], basic_left_foot_angle[i][1], basic_left_foot_angle[i][2], R);
		double temp[3];
		MatrixMultiVector3x1(R, robotModel[LEFT_FOOT].b, temp);
		robotModel[LEFT_ANKLE_SIDE_SWING].p[0] = basic_left_foot[i][0] - temp[0];
		robotModel[LEFT_ANKLE_SIDE_SWING].p[1] = basic_left_foot[i][1] - temp[1];
		robotModel[LEFT_ANKLE_SIDE_SWING].p[2] = basic_left_foot[i][2] - temp[2];
		inverseKinmatics_leftFoot(basic_left_foot_angle[i][0], basic_left_foot_angle[i][1], basic_left_foot_angle[i][2]);

		rpy2rot(basic_right_foot_angle[i][0], basic_right_foot_angle[i][1], basic_right_foot_angle[i][2], R);
		MatrixMultiVector3x1(R, robotModel[RIGHT_FOOT].b, temp);
		robotModel[RIGHT_ANKLE_SIDE_SWING].p[0] = basic_right_foot[i][0] - temp[0];
		robotModel[RIGHT_ANKLE_SIDE_SWING].p[1] = basic_right_foot[i][1] - temp[1];
		robotModel[RIGHT_ANKLE_SIDE_SWING].p[2] = basic_right_foot[i][2] - temp[2];
		inverseKinmatics_rightFoot(basic_right_foot_angle[i][0], basic_right_foot_angle[i][1], basic_right_foot_angle[i][2]);
		writeTxt();
	}
}*/

/*void test3() {
	// 起步和前进一步加减速加停止
	robotModelInit(robotModel);
#if DEBUG
	printf("初始化完毕。\n");
#endif
	//二叉树先序遍历
	clearTxt();
	forwardKinematics(MAIN_BODY);
	robotModel[MAIN_BODY].p[0] = 0;
	robotModel[MAIN_BODY].p[1] = 0;
	robotModel[MAIN_BODY].p[2] = 0.3;
	for (int i = 0; i < step_start_frame; i++)
	{
		robotModel[LEFT_HAND].p[0] = start_left_hand[i][0];
		robotModel[LEFT_HAND].p[1] = start_left_hand[i][1];
		robotModel[LEFT_HAND].p[2] = start_left_hand[i][2];
		inverseKinmatics_leftHand();

		robotModel[RIGHT_HAND].p[0] = start_right_hand[i][0];
		robotModel[RIGHT_HAND].p[1] = start_right_hand[i][1];
		robotModel[RIGHT_HAND].p[2] = start_right_hand[i][2];
		inverseKinmatics_rightHand();

		double R[3][3];
		rpy2rot(start_left_foot_angle[i][0], start_left_foot_angle[i][1], start_left_foot_angle[i][2], R);
		double temp[3];
		MatrixMultiVector3x1(R, robotModel[LEFT_FOOT].b, temp);
		robotModel[LEFT_ANKLE_SIDE_SWING].p[0] = start_left_foot[i][0] - temp[0];
		robotModel[LEFT_ANKLE_SIDE_SWING].p[1] = start_left_foot[i][1] - temp[1];
		robotModel[LEFT_ANKLE_SIDE_SWING].p[2] = start_left_foot[i][2] - temp[2];
		inverseKinmatics_leftFoot(start_left_foot_angle[i][0], start_left_foot_angle[i][1], start_left_foot_angle[i][2]);

		rpy2rot(start_right_foot_angle[i][0], start_right_foot_angle[i][1], start_right_foot_angle[i][2], R);
		MatrixMultiVector3x1(R, robotModel[RIGHT_FOOT].b, temp);
		robotModel[RIGHT_ANKLE_SIDE_SWING].p[0] = start_right_foot[i][0] - temp[0];
		robotModel[RIGHT_ANKLE_SIDE_SWING].p[1] = start_right_foot[i][1] - temp[1];
		robotModel[RIGHT_ANKLE_SIDE_SWING].p[2] = start_right_foot[i][2] - temp[2];
		inverseKinmatics_rightFoot(start_right_foot_angle[i][0], start_right_foot_angle[i][1], start_right_foot_angle[i][2]);
		writeTxt();
	}

	for (int i = 0; i < step_basic_frame; i++)
	{
		robotModel[LEFT_HAND].p[0] = basic_left_hand[i][0];
		robotModel[LEFT_HAND].p[1] = basic_left_hand[i][1];
		robotModel[LEFT_HAND].p[2] = basic_left_hand[i][2];
		inverseKinmatics_leftHand();

		robotModel[RIGHT_HAND].p[0] = basic_right_hand[i][0];
		robotModel[RIGHT_HAND].p[1] = basic_right_hand[i][1];
		robotModel[RIGHT_HAND].p[2] = basic_right_hand[i][2];
		inverseKinmatics_rightHand();

		double R[3][3];
		rpy2rot(basic_left_foot_angle[i][0], basic_left_foot_angle[i][1], basic_left_foot_angle[i][2], R);
		double temp[3];
		MatrixMultiVector3x1(R, robotModel[LEFT_FOOT].b, temp);
		robotModel[LEFT_ANKLE_SIDE_SWING].p[0] = basic_left_foot[i][0] - temp[0];
		robotModel[LEFT_ANKLE_SIDE_SWING].p[1] = basic_left_foot[i][1] - temp[1];
		robotModel[LEFT_ANKLE_SIDE_SWING].p[2] = basic_left_foot[i][2] - temp[2];
		inverseKinmatics_leftFoot(basic_left_foot_angle[i][0], basic_left_foot_angle[i][1], basic_left_foot_angle[i][2]);

		rpy2rot(basic_right_foot_angle[i][0], basic_right_foot_angle[i][1], basic_right_foot_angle[i][2], R);
		MatrixMultiVector3x1(R, robotModel[RIGHT_FOOT].b, temp);
		robotModel[RIGHT_ANKLE_SIDE_SWING].p[0] = basic_right_foot[i][0] - temp[0];
		robotModel[RIGHT_ANKLE_SIDE_SWING].p[1] = basic_right_foot[i][1] - temp[1];
		robotModel[RIGHT_ANKLE_SIDE_SWING].p[2] = basic_right_foot[i][2] - temp[2];
		inverseKinmatics_rightFoot(basic_right_foot_angle[i][0], basic_right_foot_angle[i][1], basic_right_foot_angle[i][2]);
		writeTxt();
	}

	for (int i = 0; i < step_basic_frame; i++)
	{
		robotModel[LEFT_HAND].p[0] = basic_left_hand[i][0];
		robotModel[LEFT_HAND].p[1] = basic_left_hand[i][1];
		robotModel[LEFT_HAND].p[2] = basic_left_hand[i][2];
		inverseKinmatics_leftHand();

		robotModel[RIGHT_HAND].p[0] = basic_right_hand[i][0];
		robotModel[RIGHT_HAND].p[1] = basic_right_hand[i][1];
		robotModel[RIGHT_HAND].p[2] = basic_right_hand[i][2];
		inverseKinmatics_rightHand();

		double R[3][3];
		rpy2rot(basic_left_foot_angle[i][0], basic_left_foot_angle[i][1], basic_left_foot_angle[i][2], R);
		double temp[3];
		MatrixMultiVector3x1(R, robotModel[LEFT_FOOT].b, temp);
		robotModel[LEFT_ANKLE_SIDE_SWING].p[0] = basic_left_foot[i][0] * cos(PI / 6) - basic_left_foot[i][1] * sin(PI / 6) - temp[0];
		robotModel[LEFT_ANKLE_SIDE_SWING].p[1] = basic_left_foot[i][0] * sin(PI / 6) + basic_left_foot[i][1] * cos(PI / 6) - temp[1];
		robotModel[LEFT_ANKLE_SIDE_SWING].p[2] = basic_left_foot[i][2] - temp[2];
		inverseKinmatics_leftFoot(basic_left_foot_angle[i][0], basic_left_foot_angle[i][1], basic_left_foot_angle[i][2]);

		rpy2rot(basic_right_foot_angle[i][0], basic_right_foot_angle[i][1], basic_right_foot_angle[i][2], R);
		MatrixMultiVector3x1(R, robotModel[RIGHT_FOOT].b, temp);
		robotModel[RIGHT_ANKLE_SIDE_SWING].p[0] = basic_right_foot[i][0] * cos(PI / 6) - basic_right_foot[i][1] * sin(PI / 6) - temp[0];
		robotModel[RIGHT_ANKLE_SIDE_SWING].p[1] = basic_right_foot[i][0] * sin(PI / 6) + basic_right_foot[i][1] * cos(PI / 6) - temp[1];
		robotModel[RIGHT_ANKLE_SIDE_SWING].p[2] = basic_right_foot[i][2] - temp[2];
		inverseKinmatics_rightFoot(basic_right_foot_angle[i][0], basic_right_foot_angle[i][1], basic_right_foot_angle[i][2]);
		writeTxt();
	}

	for (int i = 0; i < step_slow_frame; i++)
	{
		robotModel[LEFT_HAND].p[0] = slow_left_hand[i][0];
		robotModel[LEFT_HAND].p[1] = slow_left_hand[i][1];
		robotModel[LEFT_HAND].p[2] = slow_left_hand[i][2];
		inverseKinmatics_leftHand();

		robotModel[RIGHT_HAND].p[0] = slow_right_hand[i][0];
		robotModel[RIGHT_HAND].p[1] = slow_right_hand[i][1];
		robotModel[RIGHT_HAND].p[2] = slow_right_hand[i][2];
		inverseKinmatics_rightHand();

		double R[3][3];
		rpy2rot(basic_left_foot_angle[i][0], basic_left_foot_angle[i][1], basic_left_foot_angle[i][2], R);
		double temp[3];
		MatrixMultiVector3x1(R, robotModel[LEFT_FOOT].b, temp);
		robotModel[LEFT_ANKLE_SIDE_SWING].p[0] = slow_left_foot[i][0] - temp[0];
		robotModel[LEFT_ANKLE_SIDE_SWING].p[1] = slow_left_foot[i][1] - temp[1];
		robotModel[LEFT_ANKLE_SIDE_SWING].p[2] = slow_left_foot[i][2] - temp[2];
		inverseKinmatics_leftFoot(basic_left_foot_angle[i][0], basic_left_foot_angle[i][1], basic_left_foot_angle[i][2]);

		rpy2rot(basic_right_foot_angle[i][0], basic_right_foot_angle[i][1], basic_right_foot_angle[i][2], R);
		MatrixMultiVector3x1(R, robotModel[RIGHT_FOOT].b, temp);
		robotModel[RIGHT_ANKLE_SIDE_SWING].p[0] = slow_right_foot[i][0] - temp[0];
		robotModel[RIGHT_ANKLE_SIDE_SWING].p[1] = slow_right_foot[i][1] - temp[1];
		robotModel[RIGHT_ANKLE_SIDE_SWING].p[2] = slow_right_foot[i][2] - temp[2];
		inverseKinmatics_rightFoot(basic_right_foot_angle[i][0], basic_right_foot_angle[i][1], basic_right_foot_angle[i][2]);
		writeTxt();
	}

	for (int i = 0; i < step_stop_frame; i++)
	{
		robotModel[LEFT_HAND].p[0] = stop_left_hand[i][0];
		robotModel[LEFT_HAND].p[1] = stop_left_hand[i][1];
		robotModel[LEFT_HAND].p[2] = stop_left_hand[i][2];
		inverseKinmatics_leftHand();

		robotModel[RIGHT_HAND].p[0] = stop_right_hand[i][0];
		robotModel[RIGHT_HAND].p[1] = stop_right_hand[i][1];
		robotModel[RIGHT_HAND].p[2] = stop_right_hand[i][2];
		inverseKinmatics_rightHand();

		double R[3][3];
		rpy2rot(stop_left_foot_angle[i][0], stop_left_foot_angle[i][1], stop_left_foot_angle[i][2], R);
		double temp[3];
		MatrixMultiVector3x1(R, robotModel[LEFT_FOOT].b, temp);
		robotModel[LEFT_ANKLE_SIDE_SWING].p[0] = stop_left_foot[i][0] - temp[0];
		robotModel[LEFT_ANKLE_SIDE_SWING].p[1] = stop_left_foot[i][1] - temp[1];
		robotModel[LEFT_ANKLE_SIDE_SWING].p[2] = stop_left_foot[i][2] - temp[2];
		inverseKinmatics_leftFoot(stop_left_foot_angle[i][0], stop_left_foot_angle[i][1], stop_left_foot_angle[i][2]);
		
		rpy2rot(stop_right_foot_angle[i][0], stop_right_foot_angle[i][1], stop_right_foot_angle[i][2], R);
		MatrixMultiVector3x1(R, robotModel[RIGHT_FOOT].b, temp);
		robotModel[RIGHT_ANKLE_SIDE_SWING].p[0] = stop_right_foot[i][0] - temp[0];
		robotModel[RIGHT_ANKLE_SIDE_SWING].p[1] = stop_right_foot[i][1] - temp[1];
		robotModel[RIGHT_ANKLE_SIDE_SWING].p[2] = stop_right_foot[i][2] - temp[2];
		inverseKinmatics_rightFoot(stop_right_foot_angle[i][0], stop_right_foot_angle[i][1], stop_right_foot_angle[i][2]);
		writeTxt();
	}
}*/





