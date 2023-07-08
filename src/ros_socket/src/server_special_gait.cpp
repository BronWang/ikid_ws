#include<stdio.h>
#include<stdlib.h>
#include<cstring>
#include<sys/socket.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include<unistd.h>
#include <pthread.h>
#include<iostream>
#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "ros_socket/robotModel.h"
#include "ros_socket/cmd_walk.h"

#define MAXWORD 1024

using namespace std;

struct s_info{
    struct sockaddr_in cliaddr;
    int connfd;
    
};

extern robotLink robotModel[26];
double step_len = 0.1;
double step_wid = 0.0528 * 2;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "server_port");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;
    robotStart(n);
    string local_ip_addr;
    ros::param::get("/pid_amend/local_ip_addr",local_ip_addr);
    cout << local_ip_addr << endl;
    struct sockaddr_in servaddr, cliaddr;
    socklen_t cliaddr_len;
    int listenfd, connfd;
    char buf[MAXWORD];
    char str[MAXWORD];
    int i = 0;
    pthread_t tid_read,tid_write;
    struct s_info ts[100]; // 设置可接收的客户端上限

    //1.创建一个socket
    listenfd = socket(AF_INET, SOCK_STREAM, 0);
    int option;
    int optlen = sizeof(option);
    option = true;
    setsockopt(listenfd,SOL_SOCKET,SO_REUSEADDR,(void*)&option,optlen);
    if (listenfd == -1)
    {
        cout << "socket 创建失败： "<< endl;
        exit(1);
    }
    bzero(&servaddr, sizeof(servaddr));
    //2.准备通讯地址（必须是服务器的）是本机的IP
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(1110);//将一个无符号短整型的主机数值转换为网络字节顺序，即大尾顺序(big-endian)
    servaddr.sin_addr.s_addr = inet_addr(local_ip_addr.data());//net_addr方法可以转化字符串，主要用来将一个十进制的数转化为二进制的数，用途多于ipv4的IP转化。

    //3.bind()绑定
    //参数一：0的返回值（listenfd）
    //参数二：(struct sockaddr*)&addr 前面结构体，即地址
    //参数三: addr结构体的长度
    int res = bind(listenfd,(struct sockaddr*)&servaddr,sizeof(servaddr));
    if (res == -1)
    {
        cout << "bind创建失败: " << endl;
        exit(-1);
    }
    cout << "bind ok 等待客户端的连接" << endl;
    //4.监听客户端listen()函数
    //参数二：进程上限，一般小于30
    listen(listenfd,30);
    //5.等待客户端的连接accept()，返回用于交互的socket描述符
    socklen_t len = sizeof(cliaddr);
    ros::Rate loop_rate(50);
    
    // 需要等待上传文件的路径
    string txt_src = "";
    
    //保存关节数据
    FILE* fp = NULL;
	char ch[500];
	char filename[] = "/home/wp/ikid_ws/specialgait_data.txt";
    fp = fopen(filename, "w");
	if (fp == NULL)
	{
		exit(0);
	}
	fclose(fp);
   ros::param::get("/pid_amend/walk_length",step_len);
   ros::param::get("/pid_amend/walk_width",step_wid);
   ros_socket::cmd_walk walk_msg;
   ros::Publisher pub_walk = n.advertise<ros_socket::cmd_walk>("/cmd_walk", 5);
   while (ros::ok()) 
   {
        cout << "正在监听网络连接...\n" << endl;
        int connfd = accept(listenfd,(struct sockaddr*)&cliaddr,&len);
        if (connfd == -1)
        {
            cout << "accept错误\n" << endl;
            exit(-1);
        }
        cout << "网络已连接\n" << endl;
        //6.使用第5步返回socket描述符，进行读写通信。
        int n,i;
        char buf[MAXWORD];
        char str[INET_ADDRSTRLEN];
        while (ros::ok())
        {
            
            n = read(connfd, buf, sizeof(buf));
            if(n!=0 && n!=-1){
                // 接收网络通信消息，并处理后发送到仿真环境中的机器人
                char* token;
                const char spl_chara[2] = "\n";
                token = strtok(buf,spl_chara);
                string token_str = token;
                // if(token_str == "special_gait_data"){
                //     for (int i = 1; i <= 25; i++)
                //     {
                //         token = strtok(NULL, ",");
                //         double robot_q = atof(token);
                //         robotModel[i].q = robot_q;
                //         //cout << "[out] " << robot_q << endl;
                //     }
                //     #if ROSPUB
                //     ikidRobotDynaPosPubSpecialGait();
                //     #endif
                //     #if CONTROLBOARDPUB
                //     ikidRobotDynaPosControlBoardPubSpecialGait();
                //     ros::Duration(0.02).sleep();
                //     #endif
                //     //保存关节数据
                //     fp = fopen(filename, "a");
                //     if(fp == NULL)
                //     {
                //         exit(0);
                //     }
                //     sprintf(ch, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
                //     robotModel[0].q, robotModel[1].q,robotModel[2].q,robotModel[3].q,
                //     robotModel[4].q, robotModel[5].q,robotModel[6].q,robotModel[7].q,
                //     robotModel[8].q, robotModel[9].q,robotModel[10].q,robotModel[11].q,
                //     robotModel[12].q, robotModel[13].q,robotModel[14].q,robotModel[15].q,
                //     robotModel[16].q, robotModel[17].q,robotModel[18].q,robotModel[19].q,
                //     robotModel[20].q, robotModel[21].q,robotModel[22].q,robotModel[23].q,
                //     robotModel[24].q, robotModel[25].q);
                //     fputs(ch, fp);
                //     fclose(fp);
                // }
                if(token_str == "start_write_gait_txt"){
                    txt_src = "/home/wp/ikid_ws/specialGaitFile/";
                    token = strtok(NULL, spl_chara);
                    string temp_str = token;
                    txt_src = txt_src + temp_str+".txt";
                    fp = fopen(txt_src.data(), "w");
                    if(fp == NULL)
                    {
                        exit(0);
                    }
                    fclose(fp);
                    char message[] = "success start_write_gait_txt";
                    write(connfd, message, sizeof(message));
                }
                if(token_str == "gait_txt_data"){

                    token = strtok(NULL, spl_chara);
                    string txt_data = "";
                    //保存txt文件数据
                    string temp_str = token;
                    txt_data = temp_str + "\n";
                    fp = fopen(txt_src.data(), "a");
                    if(fp == NULL)
                    {
                        exit(0);
                    }
                    fputs(txt_data.data(), fp);
	                fclose(fp);
                    char message[] = "success gait_txt_data";
                    write(connfd, message, sizeof(message));
                }
                if(token_str == "cmd_walk_start_walk"){
                    readIkidRobotZeroPoint(0);
                    for (int i = 1; i <= 25; i++)
                    {
                        robotModel[i].q = 0;
                    }
                    initRobotPos();
                }
                if(token_str == "cmd_walk_forward"){
                    walk_msg.stop_walk = false;
                    walk_msg.sx = step_len;
                    walk_msg.sy = step_wid;
                    walk_msg.var_theta = 0;
                    walk_msg.walk_with_ball = false;
                    pub_walk.publish(walk_msg);
                }
                if(token_str == "cmd_walk_left"){
                    walk_msg.stop_walk = false;
                    walk_msg.sx = step_len;
                    walk_msg.sy = step_wid;
                    walk_msg.var_theta = 25/180.0*M_PI;
                    walk_msg.walk_with_ball = false;
                    pub_walk.publish(walk_msg);
                }
                if(token_str == "cmd_walk_right"){
                    walk_msg.stop_walk = false;
                    walk_msg.sx = step_len;
                    walk_msg.sy = step_wid;
                    walk_msg.var_theta = -25/180.0*M_PI;
                    walk_msg.walk_with_ball = false;
                    pub_walk.publish(walk_msg);
                }
                if(token_str == "cmd_walk_stop"){
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
                            pub_walk.publish(walk_msg);
                        } 
                    }
                }
            }else{
                printf("The client ip: %s, port: %d\n has beens closed!\n", 
                    inet_ntop(AF_INET, &cliaddr.sin_addr, str, sizeof(str)), ntohs(cliaddr.sin_port));
                break;
            }
            
        }
        close(ts->connfd);

        ros::spinOnce(); 
        loop_rate.sleep();
     }
        
    //7.关闭sockfd
    close(listenfd);
    return 0;
}


