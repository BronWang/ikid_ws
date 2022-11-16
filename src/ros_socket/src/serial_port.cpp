#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>

int main(int argc, char** argv){
    ros::init(argc,argv, "serial_port");
    ros::NodeHandle n;
    serial::Serial sp;
    serial::Timeout to = serial::Timeout::simpleTimeout(10);

    // 设置串口名称
    sp.setPort("/dev/ttyUSB0");
    sp.setBaudrate(115200);
    sp.setTimeout(to);
    try
    {
        sp.open();
    }
    catch(const serial::IOException& e)
    {
        ROS_ERROR_STREAM("unable to connect ttyUSB0");
        return -1;
    }
    if(sp.isOpen()){
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened");
    }else{
        return -1;
    }

    ros::Rate loop_rate(1);
    uint8_t buffer[1024];

    uint8_t send_data[7] = {0xDD, 1,0xA5,0x03,0x00,0xFF,0xFD};
    while (ros::ok())
    {
        size_t n = sp.available();
        size_t ret=1;
        try{
            ret = sp.write(send_data,sizeof(send_data));
        }catch(const serial::IOException& e)
        {
            ROS_ERROR_STREAM("unable to send");
            //return -1;
        }
        
        ROS_INFO_STREAM(ret);
        if(n != 0 ){
            n = sp.read(buffer,n);
            for(int i = 0; i<n; i++){
                std::cout<<"接收到的数据：";
                std::cout<< std::hex<< (buffer[i]&0xff)<<" ";
            }
            std::cout<<std::endl;
        }
        loop_rate.sleep();
    }
    sp.close();
    return 0;
    
    
}
