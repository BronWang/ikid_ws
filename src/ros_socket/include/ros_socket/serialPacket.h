#ifndef SERIALPACKET_H
#define SERIALPACKET_H
#include "def.h"


/**************** Data structure ************************/
#define MAX_PARAM_NUM		256			// Maximum parameter number

////////////// Instruction packet ////////////////////////
typedef struct
{
    BYTE	id;							// Dynamixel ID
    BYTE	length;						// Packet length = parameter's number + 1(ID) + 1(Instruction)
    BYTE	instruction;				// Instruction code
    BYTE	parameter[MAX_PARAM_NUM];	// Parameters
}DSP_INST_PACKET;

////////////////// Status packet /////////////////////////
typedef struct
{
    BYTE	id;							// Dynamixel ID
    BYTE	length;						// Packet length = parameter's number + 1(ID) + 1(infomation)
    BYTE	infomation;						// Error code
    BYTE	parameter[MAX_PARAM_NUM];	// Parameters
}DSP_STATUS_PACKET;

////////////////// Feedback packet /////////////////////////
struct RealAngle
{
    SHORT incline[3];
    SHORT omega[3];
};

struct RigidOffset
{
    SHORT x;
    SHORT y;
    SHORT z;
};

struct RigidPose
{
    SHORT alpha;
    SHORT beta;
    SHORT theta;
};

struct RigidBody
{
    struct RigidOffset offset;
    struct RigidPose pose;
};

struct FeedbackStatePiece
{
    USHORT isLeft;
    struct RigidBody hip;
    struct RealAngle sensors;
};

struct SensorsRaw
{
    SHORT gyro[3];
    SHORT accel[3];
    SHORT mag[3];
};

typedef struct
{
	float accel_float[3];
	float gyro_float[3];
	float compass_float[3];
	float orientation[3];
	float temperature;
}icm20948_data_t;

/****************** Define Macro ************************/

///////////////// Instruction code ///////////////////////
// 舵机命令
#define INST_CONNECTION_VALID           0x01
#define INST_SINGLE_ACTION				0x02
#define INST_MULTIPLE_ACTION			0x03
#define INST_PROPERTY_SETUP				0x04
#define INST_BULK_DOWNLOAD				0x05
#define INST_RESET						0x06
#define INST_STATE_FEEDBACK				0x07
#define INST_TORQUE_ON					0x08
#define INST_TORQUE_OFF					0x09
#define INST_INITIAL_STATE				0x0a
// 新添舵机命令
#define INST_GET_DXL_ID				    0x19
#define INST_SET_DXL_ID				    0x1a
#define INST_GET_SINGLE_DXL_ANGLE_SPEED		0x1b
#define INST_GET_MULTIPLE_DXL_ANGLE_SPEED		0x1c
#define INST_SET_SINGLE_DXL_ANGLE_SPEED		0x1d
#define INST_SET_MULTIPLE_DXL_ANGLE_SPEED		0x1e
// 下载命令
#define INST_GAIT_MEMORY_START			0x0b
#define INST_ADD_SINGLE_GAIT			0x0c
#define INST_GAIT_COMMAND				0x0d
#define INST_FLASH_PROGRAM				0x0e
// 传感器信息反馈命令
#define INST_START_INCLINOMETER_FEEDBACK	0x0f
#define INST_STOP_INCLINOMETER_FEEDBACK		0x10
#define INST_INCLINOMETER_REQUIRED			0x11
#define INST_STOP_GAIT_EXECUTING			0x12
#define INST_GAIT_DIRECTION                 0x13
#define INST_GAIT_DIRECTION_EXT				0x14
#define INST_ADD_DATA_PATCH					0x15
#define INST_STATE_SWAP						0x16
#define INST_CALIBRATE_AHRS					0x17
#define INST_GAIT_STABLIZATION_VISUALIZE	0x18
// 反馈信息传输命令
#define INFO_GAIT_EXECUTED				0x71
#define INFO_ICM20948_FEEDBACK	     	0x72
#define INFO_GAIT_EXECUTING				0x73
#define INFO_GAIT_FEEDBACK              0x74
// 指令掩码
#define PACKET_TYPE_MASK 				0x80

#define LOOP_EXECUTE					0xff
#define STOP_EXECUTE					0x00
#define ID_DSP 							0xfe




#endif // SERIALPACKET_H
