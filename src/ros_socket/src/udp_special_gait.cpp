#include<stdio.h>
#include<stdlib.h>
#include<cstring>
#include<sys/socket.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include<unistd.h>
#include<iostream>
#include <ros/ros.h>
#include "ros_socket/robotModel.h"


#define MAXWORD 1024

using namespace std;

struct s_info{
    struct sockaddr_in cliaddr;
    int connfd;
    
};

extern robotLink robotModel[26];


int main(int argc, char** argv)
{
    ros::init(argc, argv, "udp_special_gait_node");
    ros::NodeHandle n;
    robotStart(n);
    string local_ip_addr;
    ros::param::get("/pid_amend/local_ip_addr",local_ip_addr);
    cout << local_ip_addr << endl;
    struct sockaddr_in servaddr, cliaddr;
    socklen_t cliaddr_len;
    char buf[MAXWORD];
    char str[MAXWORD];

    struct s_info ts[100]; // 设置可接收的客户端上限

    //1.创建一个socket
    int serv_socket = socket(AF_INET, SOCK_DGRAM, 0);
    int option;
    int optlen = sizeof(option);
    option = true;
    setsockopt(serv_socket,SOL_SOCKET,SO_REUSEADDR,(void*)&option,optlen);
    bzero(&servaddr, sizeof(servaddr));
    //2.准备通讯地址（必须是服务器的）是本机的IP
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(1111);//将一个无符号短整型的主机数值转换为网络字节顺序，即大尾顺序(big-endian)
    servaddr.sin_addr.s_addr = inet_addr(local_ip_addr.data());//net_addr方法可以转化字符串，主要用来将一个十进制的数转化为二进制的数，用途多于ipv4的IP转化。
    bind(serv_socket,(struct sockaddr*)&servaddr,sizeof(servaddr));
    if (serv_socket == -1)
    {
        cout << "udp socket 创建失败： "<< endl;
        exit(1);
    }
    cout << "udp socket 创建成功\n" << endl;

    socklen_t len = sizeof(cliaddr);
    ros::Rate loop_rate(100);
    
   while (ros::ok()) 
   {
        
        //6.使用第5步返回socket描述符，进行读写通信。
        int n,i;
        char buf[MAXWORD];
        char str[INET_ADDRSTRLEN];
        while (ros::ok())
        {
            
            n = recvfrom(serv_socket,buf,MAXWORD, 0,(struct sockaddr*)&cliaddr, &cliaddr_len);
            cout << n << endl;
            if(n!=0 && n!=-1){
                // 接收网络通信消息，并处理后发送到仿真环境中的机器人
                char* token;
                const char spl_chara[2] = "\n";
                token = strtok(buf,spl_chara);
                string token_str = token;
                if(token_str == "special_gait_data"){
                    for (int i = 1; i <= 25; i++)
                    {
                        token = strtok(NULL, ",");
                        double robot_q = atof(token);
                        robotModel[i].q = robot_q;
                        cout << "[out] " << robot_q << endl;
                    }
                    #if ROSPUB
                    ikidRobotDynaPosPubSpecialGait();
                    #endif
                    #if CONTROLBOARDPUB
                    ikidRobotDynaPosControlBoardPubSpecialGait();
                    ros::Duration(0.02).sleep();
                    #endif
                    //保存关节数据
                    // fp = fopen(filename, "a");
                    // if(fp == NULL)
                    // {
                    //     exit(0);
                    // }
                    // sprintf(ch, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
                    // robotModel[0].q, robotModel[1].q,robotModel[2].q,robotModel[3].q,
                    // robotModel[4].q, robotModel[5].q,robotModel[6].q,robotModel[7].q,
                    // robotModel[8].q, robotModel[9].q,robotModel[10].q,robotModel[11].q,
                    // robotModel[12].q, robotModel[13].q,robotModel[14].q,robotModel[15].q,
                    // robotModel[16].q, robotModel[17].q,robotModel[18].q,robotModel[19].q,
                    // robotModel[20].q, robotModel[21].q,robotModel[22].q,robotModel[23].q,
                    // robotModel[24].q, robotModel[25].q);
                    // fputs(ch, fp);
                    // fclose(fp);
                }
                
            }
            
        }

        ros::spinOnce(); 
        loop_rate.sleep();
     }
        
    //7.关闭sockfd
    close(serv_socket);
    return 0;
}


