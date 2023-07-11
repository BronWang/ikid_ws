#ifndef ROBOTMODEL
#define ROBOTMODEL

#include <ros/ros.h>
#define DEBUG 0
#define WRITETXT 1
#define WRITEIMUDATA 1
#define WRITEZMPDATA 1
#define ROSPUB 1    // 是否向Gazebo中的关节控制器发送计算出的关节位置信息
#define CONTROLBOARDPUB 0    // 是否向物理控制板发送计算出的关节位置信息
#define SWING_ARM 0
#define PID_AMEND  0  // 是否对机器人的姿态进行PID修正
#define PART_NUMBER 26
#define NONE_JOINT  255
#define PI  3.1415926
// 机器人关节组件,相关数据单位均以基本单位为基础
typedef struct robotLink {
	unsigned int linkID;    // 自身的编号 linkId
	const char *name;          // 连杆名称 name
	unsigned int sister;    // 姐妹连杆的编号 sister
	unsigned int child;     // 子连杆的编号 child
	unsigned int mother;    // 母连杆的编号 mother
	double p[3];                  // 在世界坐标系中的位置 p
	double R[3][3];               // 在世界坐标系中的姿态 R
	double v[3];               // 在世界坐标系中的速度 v
	double w[3];				// 在世界坐标系中的角速度 w
	double q;					// 关节角 q
	double dq;					// 关节速度 dq
	double ddq;				// 关节角加速度 ddq
	double a[3];				// 关节轴矢量（相对于母连杆） a
	double b[3];				// 相对位置（相对于母连杆） b
	double m;			// 质量 m
	double c[3];				// 质心位置（在连杆局部坐标系中） c
	double I[3][3];				// 惯性张量（在连杆局部坐标系中） I
}robotLink;					
				
enum {
	MAIN_BODY = 0,
	HEAD,
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
	LEFT_FOOT
};
// 20个关节，一个主体，两个手端，两个脚端，一个头部，共26个部件
// id 0 主体
// id 1 头部
// id 2 颈前摆          初始位置2048    上2048-2844下
// id 3 颈旋转			初始位置2048    顺1024-3096逆
// id 4 右大臂前摆	    初始位置2048    后1024-3096前
// id 5 右大臂侧摆	    初始位置1024    左1024-3096右
// id 6 右臂肘前摆	    初始位置2048    后2048-3593前
// id 7 右手端			
// id 8 左大臂前摆	    初始位置2048    前1024-3096后
// id 9 左大臂侧摆	    初始位置3096    左1024-3096右
// id 10 左臂肘前摆	    初始位置2048    前747-2048后
// id 11 左手端		    
// id 12 右髋前摆		初始位置2048    后1536-2560前
// id 13 右髋侧摆		初始位置2048    右1712-2165左
// id 14 右髋旋转		初始位置2048    顺1712-2280逆
// id 15 右膝前摆		初始位置2048    后1024-3096前
// id 16 右踝前摆		初始位置2048    上1376-2851下
// id 17 右踝侧摆		初始位置2048    右1536-2560左
// id 18 右脚端		    
// id 19 左髋前摆		初始位置2048    前1536-2560后
// id 20 左髋侧摆		初始位置2048    右1712-2165左
// id 21 左髋旋转		初始位置2048    顺1712-2280逆
// id 22 左膝前摆		初始位置2048    前1024-3096后
// id 23 左踝前摆		初始位置2048    下1376-2851上
// id 24 左踝侧摆		初始位置2048    右1536-2560左
// id 25 左脚端	
void ikidRobotDynaPosPubInit(ros::NodeHandle& n_);	    
void ikidRobotDynaPosPub(); // 仿真环境
void ikidRobotDynaPosPubSpecialGait(); // 仿真环境
void ikidRobotDynaPosControlBoardPub(); // 物理环境
void ikidRobotDynaPosControlBoardPubSpecialGait();
void readIkidRobotZeroPoint(int id);
void robotModelInit(robotLink*); // 已测试
void initRobotPos(); 
void initRobotPosSpecialGait();
void robotStart(ros::NodeHandle& n_); // 已测试
void robotStartSpecialGait(ros::NodeHandle& n_);
void MatrixSquare3x3(double a[3][3], double a_square[3][3]); // 已测试
void MatrixMultiMatrix3x3(double a[3][3], double b[3][3], double result[3][3]); // 已测试
void MatrixMultiVector3x1(double a[3][3], double b[3], double result[3]); // 已测试
void VectorAddVector3x1(double a[3], double b[3], double result[3]); // 已测试
void invMatrix3x3(double matrix[3][3], double result[3][3]);// 已测试
void MatrixMultiVector6x1(double a[6][6], double b[6], double result[6]);
void MatrixMultiVector8x1(double a[8][8], double b[8], double result[8]);
void matrix_inverse_LU(double a[6][6] ,double a_inverse[6][6]);// 可用但不建议用，直接使用matlab_inv
void LU_decomposition(double arr[6][6], double W_n[6][6]); // 可用但不建议用，直接使用matlab_inv
bool sign(double value); 
double norm(double a[], int m, int n);
void rodrigues(double* a, double q, double R[3][3]); // 已测试
void rot2omega(double a[3][3], double omega[3]); // 已测试
void forwardKinematics(unsigned int linkID); // 已测试
double totalMass(unsigned int linkID);
void Ryaw(double q, double result[3][3]); // 已测试
void Rroll(double q, double result[3][3]);// 已测试
void Rpitch(double q, double result[3][3]);// 已测试
void rpy2rot(double r, double p, double y, double result[3][3]);// 已测试，角度均为弧度表示
void cross3x1(double a[3], double b[3], double result[3]);  // 已测试
void ForwardVelocity(unsigned int linkID);
void CalcP(unsigned int linkID, double P[3]);
void R_T3x3(double R[3][3], double R_T[3][3]);
void CalcL(unsigned int linkID, double L[3]);
void Calc_mc(unsigned int linkID, double mc[3]); // 已测试
void Calc_com(double com[3]);// 已测试
void Calc_ZMP(double fact_zmp[3], double *taoz);
void changeFoot();
void angleLimit();
void waistPosition_com(double r, double p, double y, int current_frame_count);
void inverseKinmatics_head(); //已测试
void inverseKinmatics_leftHand(); //已测试，这里结合了解析法和数值法迭代，可以调整迭代次数提高精度
void inverseKinmatics_rightHand();//已测试，这里结合了解析法和数值法迭代，可以调整迭代次数提高精度
void inverseKinmatics_leftFoot(double r, double p, double y); //已测试，这里结合了解析法和数值法迭代，可以调整迭代次数提高精度,实际需要给定的是踝关节的坐标和位姿，其可以通过足部的坐标和位姿算出
void inverseKinmatics_rightFoot(double r, double p, double y);//已测试，这里结合了解析法和数值法迭代，可以调整迭代次数提高精度,实际需要给定的是踝关节的坐标和位姿，其可以通过足部的坐标和位姿算出
void clearTxt(); // 只是方便自己输出数据在matlab可视化用
void writeTxt(); // 只是方便自己输出数据在matlab可视化用
void startTrajPlan(); // 为了让起步更稳定
void trajPlan(); // 已测试
void anglePlan(double delta); // 已测试
void CalcTrajectory_Com(int current_frame_count);
void dFootSupportPhase(double theta_mainbody, double theta_left, double theta_right);
void imuGesturePidControl(double &delta_roll, double &delta_pitch, double &delta_yaw);
void specialGaitExec(int id);
void judgeFall();
void FallUpInitPos(); //机器人跌倒起立后把腰部的高度调节到和初始一样
void writeImuData();
void clearImuDataTxt();
void writeZmpData(double zmp_data[2][85],double z_d_x, double z_d_y, double z_p_x,double z_p_y,double z_f_x, double z_f_y);
void clearZmpDataTxt();
void quinticPolyInterFour(double A[6][4], double B[6][4], double s);
void quinticPolyInterTwo(double A[6][4], double B[6][4], double s);
void threeSplineInter(double spline_A[8][2], double s);

#endif // !ROBOTMODEL

