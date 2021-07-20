#include "ros/ros.h"
#include "CAN_driver/Motor_Control.h"
#include <sstream>
#include <stdio.h>               //标准输入输出定义          
#include <stdlib.h>              //标准函数库定义
#include <unistd.h>              //Unix 标准函数定义
#include <sys/types.h>  
#include <sys/stat.h>   
#include "string.h" 
#include <fcntl.h>               //文件控制定义
#include <termios.h>             //PPSIX 终端控制定义
#include <errno.h>               //错误号定义
#include <sys/ioctl.h>  

int Input_Key=0; 


/***************************************************
*函数名：GetInput()
*输入：void
*功能：获取键盘输入
****************************************************/
char GetInput()
{   
    
    fd_set rfds;              //fd_set 为long型数组，其每个元,素都能和打开的文件句柄建立联系
    struct timeval tv;
    char c = 0;
    FD_ZERO(&rfds);           //将　rfds数组清零
    FD_SET(0, &rfds);         //将rfds的第0位置为１，这样fd=1的文件描述符就添加到了rfds中//最初　rfds为00000000,添加后变为10000000
    tv.tv_sec = 0;
    tv.tv_usec = 10;          //设置等待超时时间
    if (select(1, &rfds, NULL, NULL, &tv) > 0)          //检测键盘是否有输入 //由内核根据io状态修改rfds的内容，来判断执行了select的进程哪个句柄可读
    {
        c = getchar();
        return c;
    }   
    return 0;                  //没有数据返回n
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "Motor_Control");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<CAN_driver::Motor_Control>("Motor_Control_mssage", 10);
  ros::Rate loop_rate(100);
 
  CAN_driver::Motor_Control Motor_Control_msg;               
  Motor_Control_msg.Control_Mode_flag = 2;   //默认手动控制
  Motor_Control_msg.Angle_Value = 0;        //输入角度
  Motor_Control_msg.Acc_Value = 0;        
  Motor_Control_msg.CAN_Start = 0; 
  Motor_Control_msg.CAN_Stop = 0;
  while (ros::ok())
  {
     system("stty -icanon"); 
     Input_Key=GetInput();    //左52 右54
     //printf (" Input_Key=%d\n",Input_Key);
        if(Input_Key)
            {
                switch (Input_Key)   //K=107,O=111
                {
                    case 107:  
                        printf (" Input_Key=%d\n",Input_Key);
                        printf(">>Stop CAN1\n");
                        Motor_Control_msg.CAN_Start = 0; 
                        Motor_Control_msg.CAN_Stop = 1;               
                        break;
                    case 111: 
                        printf (" Input_Key=%d\n",Input_Key); 
                        printf(">>Send Data\n"); 
                        Motor_Control_msg.CAN_Start = 1; 
                        Motor_Control_msg.CAN_Stop = 0;    
                        break;                       
                    default:  
                        break; 
                }

                if(Input_Key == 97) //A +
                {
                   Motor_Control_msg.Angle_Value = Motor_Control_msg.Angle_Value+10;
                }
                else if(Input_Key == 100) //D -
                {
                   Motor_Control_msg.Angle_Value = Motor_Control_msg.Angle_Value-10;
                }
                /*-100 0 40*/
                /*-5m/s2 0 o-2m/s2*/
                if(Input_Key == 119) //W +
                {
                   Motor_Control_msg.Acc_Value = Motor_Control_msg.Acc_Value+1;
                }
                else if(Input_Key == 115) //S -
                {
                   Motor_Control_msg.Acc_Value = Motor_Control_msg.Acc_Value-1;
                }
            } 
            
         
            pub.publish(Motor_Control_msg);
            ROS_INFO("I send: [%d] [%d] [%d] [%d] [%d] [%d]", Input_Key,Motor_Control_msg.Control_Mode_flag , Motor_Control_msg.Angle_Value,Motor_Control_msg.Acc_Value,Motor_Control_msg.CAN_Start ,Motor_Control_msg.CAN_Stop);
            ros::spinOnce();
            loop_rate.sleep();
  }
  return 0;
}