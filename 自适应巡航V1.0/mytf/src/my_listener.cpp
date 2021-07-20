//1.包含头文件
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Point.h"



void obstacle_callback(const geometry_msgs::Point::ConstPtr& pose)
{
    
     // 4.创建 TF 订阅对象
    tf2_ros::Buffer buffer; 
    tf2_ros::TransformListener listener(buffer);
    //ros::Time timeout(0.1);
    ros::Rate rate(2.0);

    try{
        rate.sleep();
        geometry_msgs::TransformStamped tfs = buffer.lookupTransform("world","velodyne",ros::Time(0));
        /*
        ROS_INFO("velodyne 相对于 world 的坐标关系:父坐标系ID=%s",tfs.header.frame_id.c_str());
        ROS_INFO("velodyne 相对于 world 的坐标关系:子坐标系ID=%s",tfs.child_frame_id.c_str());
        ROS_INFO("velodyne 相对于 world 的坐标关系:x=%.2f,y=%.2f,z=%.2f",
        tfs.transform.translation.x,
        tfs.transform.translation.y,
        tfs.transform.translation.z
                );
        */

        // 坐标点解析
        geometry_msgs::PointStamped ps;
        ps.header.frame_id = "velodyne";
        ps.header.stamp = ros::Time();
        ps.point.x = pose->x;
        ps.point.y = pose->y;
        ps.point.z = pose->z;

        geometry_msgs::PointStamped psAtWorld;
        psAtWorld = buffer.transform(ps,"world");
        ROS_INFO("在 World 中的坐标:x=%.2f,y=%.2f,z=%.2f",
                psAtWorld.point.x,
                psAtWorld.point.y,
                psAtWorld.point.z
                );
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("lookupTransform base2laser error,%s",ex.what());
        exit(1);
    }
    
}



int main(int argc, char *argv[])
{   setlocale(LC_ALL,"");
    // 2.初始化 ros 节点
    ros::init(argc,argv,"sub_frames");
    // 3.创建 ros 句柄
    ros::NodeHandle nh;
   

    ros::Subscriber sub = nh.subscribe<geometry_msgs::Point>("coordinate",10,obstacle_callback);

    //ros::Rate loop_rate(100);

    ros::spin();
    return 0;
   
}
