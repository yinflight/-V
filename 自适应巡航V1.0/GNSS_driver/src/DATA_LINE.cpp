/***********************************************************
*Author:DashBear
*Function:利用   RVIZ绘制路径信息图
*Node: 
*Pub_msg:      
*Sub_msg:    
*Reference: 
*Attention:  
*Version: V1.0
*Revision date: 2021/05/13           
*************************************************************/


#include "ros/ros.h"
#include "GNSS_driver/GNSS_CAN.h"
#include "GNSS_driver/controlcan.h"
//#include "mems/nav.h"
#include "std_msgs/UInt8.h"
#include <ros/ros.h>             //ros通用头文件
#include <std_msgs/String.h>     //消息类型头文件
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
#include <string.h>
#include "std_msgs/Float32.h"
#include <fstream>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"


visualization_msgs::Marker gnss_points;     //GNSS实际点
visualization_msgs::Marker gnss_line;   //实际路径
visualization_msgs::Marker line_list;    //预测路径
nav_msgs::OccupancyGrid Grid_Map;        //栅格地图
ros::Subscriber gnss_message_sub;        //数据订阅
ros::Publisher draw_pub;                 //rviz数据发布
int i =0;
int Input_Key=0; 


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

/*void DataLine_Callback(const GNSS_driver::GNSS_CAN::ConstPtr& gnss_message)
{
    gnss_points.header.frame_id = gnss_line.header.frame_id = line_list.header.frame_id = "/map";    //frame帧头

    gnss_points.header.stamp = gnss_line.header.stamp = line_list.header.stamp = ros::Time::now();      //帧时间
    gnss_points.ns = "定位实际点";
    gnss_line.ns = "定位实际路线";
    line_list.ns = "points_and_lines";      //标记的命名空间
    gnss_points.action = gnss_line.action = line_list.action = visualization_msgs::Marker::ADD;          //0 = 增加, 1 = (已弃用), 2 = 删除,  3 = 删除所有   Marker.h里有
    //points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;


    //四元数设置  暂时不用
    gnss_points.pose.orientation.x = gnss_points.pose.orientation.y = gnss_points.pose.orientation.z  = gnss_points.pose.orientation.w = 0;
    gnss_line.pose.orientation.x = gnss_line.pose.orientation.y = gnss_line.pose.orientation.z = gnss_line.pose.orientation.z = 0;
    line_list.pose.orientation.x = line_list.pose.orientation.y = line_list.pose.orientation.z = line_list.pose.orientation.z = 0;
    //命名控件的唯一ID
    gnss_points.id = 0;    
    gnss_line.id = 1;
    line_list.id = 2;

    //生命周期
    gnss_points.lifetime = ros::Duration();    
    gnss_line.lifetime = ros::Duration();
    line_list.lifetime = ros::Duration();
    //标记类型设置
    gnss_points.type = visualization_msgs::Marker::POINTS;
    gnss_line.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    //标记比例 X->线宽  y->线高  z->？
    gnss_points.scale.x = 0.1;
    gnss_points.scale.y = 0;
    gnss_points.scale.z = 0;
    gnss_line.scale.x = 0.1;
    line_list.scale.x = 0.1;

    //颜色设置   .a是透明度 0完全透明 1不透明  其余为rgb 范围 0-1
    // GNSS点颜色
    gnss_points.color.g = 1.0;
    gnss_points.color.a = 1.0;
    // 实际路径颜色
    gnss_line.color.b = 1.0;
    gnss_line.color.a = 1.0;
    // 预测路径颜色
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    gnss_points.pose.position.x = 0 ;
    gnss_points.pose.position.y = 0;

    gnss_line.pose.position.x = 0 ;
    gnss_line.pose.position.y = 0;



     geometry_msgs::Point p;
      p.x =i++;
      p.y =0;
      p.z =0;
    gnss_points.points.push_back(p);
    gnss_line.points.push_back(p);

    ROS_INFO("Callback 1 triggered");
    draw_pub.publish(gnss_points);
    draw_pub.publish(gnss_line);
}*/

void DataLine_Callback(const GNSS_driver::GNSS_CAN::ConstPtr& gnss_message)
{

}


/***************************************************
*函数名：main(int argc, char **argv)
*输入：void
*返回值：void
*功能：main 
****************************************************/
int main(int argc, char **argv)
{   
    ros::init(argc, argv, "DATA_LINE");
    ros::NodeHandle nh;
    ros::Rate r(30); 
    gnss_message_sub = nh.subscribe("nav_msg", 1000, DataLine_Callback);  //   订阅经纬度数据
//    draw_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);    
    draw_pub = nh.advertise<nav_msgs::OccupancyGrid>("/gridMap", 10);    // nav_msgs::OccupancyGrid   发表栅格地图数据   
    nav_msgs::OccupancyGrid Grid_Map;

    ROS_INFO("Callback 1 triggered");

    Grid_Map.header.frame_id="grid";         //帧头
    Grid_Map.header.stamp = ros::Time::now(); 
    Grid_Map.info.resolution = 0.5;         // float32
    Grid_Map.info.width      = 30;           // uint32
    Grid_Map.info.height     = 30;           // uint32
    int p[Grid_Map.info.width*Grid_Map.info.height] = {-1};   // [0,100]   -1
    ROS_INFO("%d,%d",p[0],p[1]);
   // p[0] = 100;
   // p[1] = 100;
    std::vector<signed char> a(p, p+900);  //  p+400  p - p+900全传送给a
    Grid_Map.data = a;   //a*/

    //ros::spin();
    while (ros::ok()) //循环读取数据
    {
/*    i++;
    p[i] = 100;
    std::vector<signed char> a(p, p+400);
    Grid_Map.data = a;   //a*/
        system("stty -icanon"); 
        Input_Key=GetInput();
        if(Input_Key!=0)
            printf (" Input_Key=%d\n",Input_Key);
 //       if(Input_Key)
        draw_pub.publish(Grid_Map);
       //ROS_INFO("死循环")
  /*  gnss_message_sub = nh.subscribe("gnss_message", 1000, DataLine_Callback);
    draw_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);*/

    }  
    return 0;


}