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

#define MAXWORD 1024

using namespace std;

struct s_info{
    struct sockaddr_in cliaddr;
    int connfd;
    
};

extern robotLink robotModel[26];

int main(int argc, char** argv)
{
    ros::init(argc, argv, "server_port");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;
    robotStart(n);
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
    if (listenfd == -1)
    {
        cout << "socket 创建失败： "<< endl;
        exit(1);
    }
    bzero(&servaddr, sizeof(servaddr));
    //2.准备通讯地址（必须是服务器的）192.168.186.130是本机的IP
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(1200);//将一个无符号短整型的主机数值转换为网络字节顺序，即大尾顺序(big-endian)
    servaddr.sin_addr.s_addr = inet_addr("192.168.186.130");//net_addr方法可以转化字符串，主要用来将一个十进制的数转化为二进制的数，用途多于ipv4的IP转化。

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
            if(n!=0){
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
                    ikidRobotDynaPosPub();
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


