#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<sys/socket.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include<unistd.h>
#include<iostream>
#include <ros/ros.h>

using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "client_node");

    ros::NodeHandle n;

    int ss = socket(AF_INET, SOCK_STREAM, 0);

    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(1024);
    addr.sin_addr.s_addr = inet_addr("192.168.186.130");
    int len = sizeof(sockaddr_in);

    if(connect(ss,(sockaddr *)&addr, len) == -1){
        cout<< "connect error" << endl;
        return 0;
    }

    int ret = 0;
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        char buffer[1024] = {'\0'};
        ret = recv(ss, buffer, sizeof(buffer), 0);
        cout<< "recv_data: " << buffer << endl;
        ret = send(ss, "I am Client!", strlen("I am Client!"), 0);
        ros::spinOnce();
        loop_rate.sleep();
    }
    close(ss);
    return 0;
    

}