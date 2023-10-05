#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <sys/stat.h>
#include <fstream>
#include "ikid_motion_control/robotModel.h"
#include "ikid_motion_control/cmd_walk.h"
#include <std_msgs/Int16.h>

extern robotLink robotModel[26];
extern double FallUpRobotPos_q[26];

const int CORNER_BALL_ONE = 5;
const int CORNER_BALL_TWO = 6;


void specialGaitExec_corner_ball(int id){
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
									robotModel[j].q = zero_point[j]/180*M_PI+ FallUpRobotPos_q[j] +(gait_frame_data[j]-(zero_point[j]/180*M_PI+ FallUpRobotPos_q[j]))*i/temp_frame_rate;
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
						}
						count_frame++;
                	}

				}
				for(int i = 0; i < 26; i++){
					robotModel[i].q -= zero_point[i]/180*M_PI;
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

void specialGaitExec_corner_ball_1(int id){
	
	specialGaitExec_corner_ball(id);
	ros::param::set("stop_special_gait_flag", true);
}

void specialGaitExec_corner_ball_2(int id){
	
	specialGaitExec_corner_ball(id);
	FallUpInitPos();
	ros::param::set("stop_special_gait_flag", true);
}

void doSpecialGaitMsg(const std_msgs::Int16::ConstPtr& id_msg){

    specialGaitExec_corner_ball_2(CORNER_BALL_TWO);
    ros::Duration(2).sleep();
    specialGaitExec_corner_ball_1(CORNER_BALL_ONE);
}

int main(int argc, char *argv[])
{
    //执行 ros 节点初始化
    ros::init(argc,argv,"technical_challenges_node");
    //创建 ros 节点句柄
    ros::NodeHandle n;

    ROS_INFO("IkID-ROBOT NB! Let's challenges it!");
    robotStart(n);
    ros::Duration(3).sleep();



    ros::Subscriber specialGaitSuber = n.subscribe<std_msgs::Int16>("/special_gait",1,doSpecialGaitMsg);
    ros::param::set("stop_special_gait_flag", true);
    
    specialGaitExec_corner_ball_1(CORNER_BALL_ONE);

    ros::Rate rate(80);
    while (ros::ok())
    {
        rate.sleep();
        ros::spinOnce();
    }
    
    
    return 0;
}
