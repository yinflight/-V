// 1.包含头文件
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <stdio.h>                         //标准输入输出定义          
#include <stdlib.h>                        //标准函数库定义



void odomPose(const geometry_msgs::Twist::ConstPtr& pose){
    //  5-1.创建 TF 广播器
    static tf2_ros::TransformBroadcaster broadcaster;
    //  5-2.创建 广播的数据(通过 pose 设置)
    geometry_msgs::TransformStamped tfs;
    //  |----头设置
    tfs.header.frame_id = "world";
    tfs.header.stamp = ros::Time::now();

    //  |----坐标系 ID
    tfs.child_frame_id = "odom";

    //  |----坐标系相对信息设置
    tfs.transform.translation.x = pose->linear.x ;
    tfs.transform.translation.y = pose->linear.y ;
    tfs.transform.translation.z = 0.0; // 二维实现，pose 中没有z，z 是 0
    //  |--------- 四元数设置
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,pose->linear.z/57.3 );
    tfs.transform.rotation.x = qtn.getX();
    tfs.transform.rotation.y = qtn.getY();
    tfs.transform.rotation.z = qtn.getZ();
    tfs.transform.rotation.w = qtn.getW();
    //printf("pose->linear.x %f\n" , pose->linear.x);
    
    //  5-3.广播器发布数据
    broadcaster.sendTransform(tfs);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ROS 节点
    ros::init(argc,argv,"dynamic_tf_pub");
    // 3.创建 ROS 句柄
    ros::NodeHandle nh;
    // 4.创建订阅对象
    ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("car_pose",100,odomPose);
    // 5.回调函数处理订阅到的数据(实现TF广播)
    //  
    ros::Rate loop_rate(100);      
    // 6.spin
    ros::spin();
    return 0;
}
