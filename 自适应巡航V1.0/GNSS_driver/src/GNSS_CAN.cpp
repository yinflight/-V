/***********************************************************
*Author:DashBear
*Function:接收并处理   华测导航410终端的各类数据
*Node:GNSS_CAN
*Pub_msg:nav_msg     
*Sub_msg:    
*Reference:华测导航410数据手册、王月阳师兄关于羲朗定位的程序、尹鹏飞关于CAN通信的程序
*Attention: 本程序中CAN通信数据采用Motorola线序，即高字节LSB在前,低字节MSB在后
*           本程序中Conver_Data_4byte(),Conver_Data_2byte()中的形参均为高字节在前，低字节在后
*           本程序使用的是华测410 CAN1.0版本;请注意CAN2.0版本与此程序略有出入
*Version: V1.6   
*Revision date: 2021/05/17
*************************************************************/


#include "ros/ros.h"
#include "GNSS_driver/GNSS_CAN.h"
#include "GNSS_driver/controlcan.h"
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
#define ABS(x) ((x)>=0?(x):-(x))    //取绝对值
#define Pi 3.14159265358979323846264338328
#define Scalefactor_8 0.00000001    //比例因子 1e^-8   防止多0或少0导致的数据出错
#define Scalefactor_7 0.0000001     //比例因子 1e^-7   防止多0或少0导致的数据出错
#define Scalefactor_6 0.000001      //比例因子 1e^-6   防止多0或少0导致的数据出错
#define Scalefactor_5 0.00001       //比例因子 1e^-5   防止多0或少0导致的数据出错
#define Scalefactor_4 0.0001        //比例因子 1e^-4   防止多0或少0导致的数据出错
#define Scalefactor_3 0.001         //比例因子 1e^-3   防止多0或少0导致的数据出错
#define Scalefactor_2 0.01          //比例因子 1e^-2   防止多0或少0导致的数据出错
#define Ture 1
#define False 0

using namespace std;

ros::Publisher  pub;//发布topic为pub的消息

//CAN匹配
string ProductSn[50];
VCI_BOARD_INFO pInfo [50];
string CAN_ID = "21A10000CC3";
int CAN_NUM;

//GNSS->CAN.ID配置      CGI410数据手册13页
int TIME_ID = 0X0320;               //时间ID
int IMUANG_ID  = TIME_ID + 0X0001;   //IMU角速度
int IMUACC_ID = TIME_ID + 0X0002;    //IMU加速度
int IMSSTATE_ID = TIME_ID + 0X0003;    //INS定位状态
int LOCATION_ID = TIME_ID + 0X0004;      //定位经纬度
int GROHIG_ID = TIME_ID + 0X0005;      //大地高度
int SIGMA_ID  = TIME_ID + 0X0006;      //位置西格玛
int GROVEL_ID = TIME_ID + 0X0007;      //大地坐标系速度
int GROVELSIG_ID = TIME_ID + 0X0008;      //大地坐标系速度西格玛
int ACCEL_ID = TIME_ID + 0X0009;         //车辆坐标系加速度
int ATTANG_ID = TIME_ID + 0X000A;        //姿态角
int ATTANGSIG_ID = TIME_ID + 0X000B;      //姿态角西格玛
int ANGVEL_ID = TIME_ID + 0X000C;         //车辆坐标系角速度
//int LONITUDE_ID = TIME_ID + 0X000D;         //  CAN2.0   定位经度
//int LATITUDE_ID = TIME_ID + 0X000E;         //  CAN2.0   定位纬度

std::ofstream outFile;    //读写文件操作


//GNSS->Data     CGI410数据手册13页
int  Week_Time = 0;       //星期数  1980-1-6至今 
float Gps_Time = 0.0;    //秒数   本周日0：00：00至现在
float Ang_Rate_RawX = 0.0;     //IMU-> X轴角速度
float Ang_Rate_RawY = 0.0;     //IMU-> Y轴角速度
float Ang_Rate_RawZ = 0.0;     //IMU-> Z轴角速度
float Accel_RawX = 0.0;    //IMU-> X轴加速度
float Accel_RawY = 0.0;    //IMU-> Y轴加速度
float Accel_RawZ = 0.0;    //IMU-> Z轴加速度
int System_State = 0;     //系统状态
int Gps_Num = 0;    //主天线卫星数
int Satellite_State =0;      //卫星状态
int Gps_Num2 = 0;   //辅天线卫星数
float Gps_Age = 0.0;   //差分延时
int Gps_Num_Sats =0;    //主天线搜星数
int Gps_Num_Sats2 =0;   //辅天线搜星数
double Pos_Lat = 0.0;    //纬度
double Pos_Lon = 0.0;    //经度
double Pos_Alt = 0.0;    //高度
float Pos_Sigma_East =0.0;     //东向西格玛
float Pos_Sigma_North =0.0;    //北向西格玛
float Pos_Sigma_Up =0.0;       //天向西格玛
float Vel_East =0.0;    //东向速度
float Vel_North =0.0;   //北向速度
float Vel_Up =0.0;      //天向速度
float Vel_Vehicle =0.0;     //车辆速度
float Vel_Sigma_East =0.0;      //东向速度西格玛
float Vel_Sigma_North =0.0;     //北向速度西格玛
float Vel_Sigma_Up =0.0;        //天向速度西格玛
float Vel_Sigma_Vehicle =0.0;   //车辆速度西格玛
float Accel_X = 0.0;    //车辆坐标系-> X轴加速度
float Accel_Y = 0.0;    //车辆坐标系-> Y轴加速度
float Accel_Z = 0.0;    //车辆坐标系-> Z轴加速度
float Yaw_Angle = 0.0;      //航向角
float Pitch_Angle = 0.0;    //俯仰角
float Roll_Angle = 0.0;     //横滚角
float Yaw_Angle_Sigma =0.0;     //航向角西格玛
float Pitch_Angle_Sigma =0.0;     //俯仰角西格玛
float Roll_Angle_Sigma =0.0;     //横滚角西格玛
float Ang_Rate_X = 0.0;     //车辆坐标系-> X轴角速度
float Ang_Rate_Y = 0.0;     //车辆坐标系-> Y轴角速度
float Ang_Rate_Z = 0.0;     //车辆坐标系-> Z轴角速度
//处理后的数据
bool Data_Init = False;     //保存原点
static double lattitude_Zero = 13080793.0;  //处理后平面坐标的原点
static double longitude_Zero = 4715278.6503;  //处理后平面坐标的原点
static double lattitude_X = 0.0;    //平面坐标经度
static double longitude_Y = 0.0;    //平面坐标纬度


//数据更新信号
bool Position_Signal = False;   //位置更新信号
bool Velocity_Signal = False;   //前进速度更新信号
bool Yaw_Signal = False;        //偏航角更新信号
bool Acc_Signal = False;        //前进加速度更新信号
bool Ang_Vel_Signal = False;    //前角速度更新信号

int Input_Key=0; 

GNSS_driver::GNSS_CAN nav_msg; 
ros::Publisher navpub;

/****************************************************
*函数名：save_point()
*输入：
*返回值：
*功能：固定格式保存经纬度平面值
****************************************************/
void save_point()
{
    ofstream out("/home/robot/Robot/Smart_robot_ws/src/GNSS_driver/save_point_data/rightdoubleliner.txt",std::ios::app);
    //out << setprecision(12) << lattitude_X << ',' << longitude_Y << ',' << volety <<'\n';  //',' << Angle 
    out << setprecision(12) << lattitude_X << ',' << longitude_Y <<'\n';
    out.close();
    //ROS_INFO("save");
}

/***************************************************
*函数名：GetInput()
*输入：void
*返回值：
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
 /***************************************************
*函数名：Conver_Data_4byte(BYTE First,BYTE Second,BYTE Third,BYTE Forth,int Symbol)
*输入：四个字节型变量->First、Second、Third、Forth;一个符号转换标志位->Symbol  1为带符号转换，0为不带符号转换
*返回值：转换后的值
*功能：转换CAN收到的4字节十六进制为(有/无符号)十进制数
****************************************************/
long double Conver_Data_4byte(BYTE First,BYTE Second,BYTE Third,BYTE Forth,int Symbol)
{
    if(Symbol == 1)
    {
        long Conver_Data_4 = 0.0;
        Conver_Data_4 = (long)(Conver_Data_4 ^ First);          //将First赋给Conver_Data_4的低8位
        Conver_Data_4 = (long)(Conver_Data_4 << 8);             //Conver_Data_4左移8位
        Conver_Data_4 = (long)(Conver_Data_4 ^ Second);         //将Second赋给Conver_Data_4的低8位
        Conver_Data_4 = (long)(Conver_Data_4 << 8);             //Conver_Data_4左移8位
        Conver_Data_4 = (long)(Conver_Data_4 ^ Third);          //将Third赋给Conver_Data_4的低8位
        Conver_Data_4 = (long)(Conver_Data_4 << 8);             //Conver_Data_4左移8位
        Conver_Data_4 = (long)(Conver_Data_4 ^ Forth);          //将Forth赋给Conver_Data_4的低8位*/
        Conver_Data_4 = (long double)Conver_Data_4;
        //Conver_Data_4 = (long)(First << 24 | Second << 16 | Third << 8 | Forth);
        //ROS_INFO("11==%ld",Conver_Data_4);
        return Conver_Data_4;
    }
    else if(Symbol == 0)
    {
        long Conver_Data_4 = 0;
        Conver_Data_4 = ((long) First)*16777215+((long)Second)*655355+((long)Third)*255+(long)Forth;   //由于数据溢出原因，此处特殊
        //ROS_INFO("%ld",Conver_Data_4);
        return Conver_Data_4;
    }
    else
        return 0;
}


/***************************************************
*函数名：Conver_Data_2byte(BYTE First,BYTE Second,int Symbol)
*输入：两个字节型变量->First、Second;一个符号转换标志位->Symbol  1为带符号转换，0为不带符号转换
*返回值：转换后的值
*功能：转换CAN收到的2字节十六进制为(有/无符号)十进制数
****************************************************/
long Conver_Data_2byte(BYTE First,BYTE Second,int Symbol)
{
    if(Symbol == 1)
    {
        short Conver_Data_2 = 0;
        Conver_Data_2 = (short)(Conver_Data_2 ^ First);          //将First赋给Conver_Data_4的低8位
        Conver_Data_2 = (short)(Conver_Data_2 << 8);             //Conver_Data_4左移8位
        Conver_Data_2 = (short)(Conver_Data_2 ^ Second);         //将Second赋给Conver_Data_4的低8位

        //ROS_INFO("%d",Conver_Data_2);
        return Conver_Data_2;             //*/
    }
    else if(Symbol == 0)
    {
      /*  unsigned long Conver_Data_2 = 0;
        Conver_Data_2 = (unsigned long)(Conver_Data_2 ^ First);          //将First赋给Conver_Data_4的低8位
        Conver_Data_2 = (unsigned long)(Conver_Data_2 << 8);             //Conver_Data_4左移8位
        Conver_Data_2 = (unsigned long)(Conver_Data_2 ^ Second);         //将Second赋给Conver_Data_4的低8位
        //ROS_INFO("2222--%d",Conver_Data_2);*/
        unsigned long Conver_Data_2 = 0.0;
        //Conver_Data_2 = ((int)First*256)+((int)Second);
        Conver_Data_2 = ((((int)First)*256)+((int)Second));
        return Conver_Data_2;
    }
    else
        return 0;
}

/***************************************************
*函数名：GNSS_Data_Conver(double longitude,double lattitude)
*输入：GNSS读出来的经度和纬度
*返回值：转换后的的经度和纬度
*功能：球面坐标转为平面坐标   此程序无需懂，会用即可    一般转换 采取此程序
****************************************************/
void GNSS_Data_Conver(double longitude,double lattitude) 
{   
   double a,b,L,B,e1,e2,BO,L0,N,K,X,Y;
   double p,k,m;
    a = 6378137.0000;//地球椭球体长半球，单位m
    b = 6356752.3142;//地球椭球体短半轴，单位m
    //将角度转换为弧度
    L = longitude* Pi/180;//经度
    B = lattitude* Pi/180;//纬度
    e1 = 0.0818191909289062;//第一偏心率
    e2 = 0.0820944380368541;//第二偏心率
    BO = 0;//投影基准纬度
    L0 = 0;//media坐标原点的经度
    N = a/sqrt(1-e1*e1*sin(B)*sin(B));//卯酉圈曲率半径
    //计算平面直角坐标，X:水平直角坐标 单位m，Y:纵向直角坐标 单位m
    K = N*cos(BO);
    lattitude_X = K*(L - L0);//水平直角坐标，单位m
    longitude_Y = K*log(tan(Pi/4+B/2)*(pow(((1-e1*sin(B))/(1+e1*sin(B))),e1/2)));//纵向直角坐标，单位为m
  /*  if(Data_Init==False)  //保存原点
    {
        lattitude_Zero = lattitude_X; 
        longitude_Zero = longitude_Y;
        Data_Init = Ture;
    }    // */
    
  //   lattitude_X = lattitude_X - lattitude_Zero;     //平面坐标减去原点坐标
  //   longitude_Y = longitude_Y - longitude_Zero;     //平面坐标减去原点坐标
        
    
}

/***************************************************
*功能：定义CAN数据发送结构体
*功能：定义发布、接收话题结构体 
****************************************************/
class sub_send
{
    public: 
        sub_send(ros::NodeHandle &nh);
        ~sub_send();
        unsigned char Verify_Conversion(unsigned char Conver_Data[]);
        void InitCAN();
        void Start_CAN();
        void Close_CAN();
        void Send_Data(unsigned char BYTE0,unsigned char BYTE1,unsigned char BYTE2,unsigned char BYTE3,unsigned char BYTE4,unsigned char BYTE5,unsigned char BYTE6);
        
    private:
        VCI_CAN_OBJ rec[1000];                                                             //CAN接收缓存
 //       VCI_CAN_OBJ sendbuf[1];                                                           //CAN发送MCU一帧
 //       VCI_CAN_OBJ Motorsendbuf[1];                                                      //CAN发送转向电机一帧
 //       unsigned char SendData[8]={0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00};              //MCU默认手动驾驶
 //       unsigned char MotorSendData[8]={0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00};         //转向电机默认手动驾驶     
        //unsigned char SendData[8]={0x10,0x00,0x00,0x00,0x00,0x55,0x00,0x45};              //初始化发送的数据（标定


        
       
        void Receive_Data();
        void timerCallback(const ros::TimerEvent&);

        //void MotorControl_callback(const control::GNSS_CAN::ConstPtr& GNSS_CAN_msg);
};

sub_send::sub_send(ros::NodeHandle &nh)
{
    InitCAN();    //初始化can

    
    
    navpub = nh.advertise<GNSS_driver::GNSS_CAN>("gnss_message", 1000);
    
 //   Receive_Data(); 
 //   ros::Timer timer = nh.createTimer(ros::Duration(0.1), sub_send::timerCallback);
 //   Motor_Pub = nh.advertise<control::Motor_Feedback>("Motor_Feedback_mssage", 10);  
  //  ros::spin();
}

sub_send::~sub_send(){}


/*void sub_send::MotorControl_callback(const control::GNSS_CAN::ConstPtr& GNSS_CAN_msg )
{
    Receive_Data();
    ROS_INFO("在线");
}
*/
/***************************************************
*函数名：Send_Data()
*输入：void（待修改）
*功能：发送CAN数据
****************************************************/
/*void sub_send::Send_Data(unsigned char BYTE0,unsigned char BYTE1,unsigned char BYTE2,unsigned char BYTE3,unsigned char BYTE4,unsigned char BYTE5,unsigned char BYTE6)
{
    

	    if(VCI_Transmit(VCI_USBCAN2, 0, 0, sendbuf, 1) == 1)    //  设备类型号、设备索引号 0->com1,1->com2、第几路CAN、要发送的数据帧数组的首指针、要发送的数据帧数组的长度;返回实际长度
	    {
    
		}
        else
	    {
		    ROS_INFO (" CAN sendbuf error");
		}
    
}*/



/***************************************************
*函数名：Receive_Data()
*输入：void
*功能：接收CAN数据
****************************************************/

void Receive_Data()   //sub_send::
{
    unsigned int reclen=0,j=0;
    std::stringstream ss;

    VCI_CAN_OBJ rec[1000];    
    //ROS_INFO("k"); 
    reclen=VCI_Receive(VCI_USBCAN2,CAN_NUM,0,rec,1000,0);       //接收长度读取
    //ROS_INFO("kee%d",reclen); 
    setlocale(LC_CTYPE, "zh_CN.utf8");   //这条语句让ROS_INFO能显示中文
    //ROS_INFO("我在等待数据");
    if(reclen>0)//如果有数据，进行数据处理并显示。  if((reclen=VCI_Receive(VCI_USBCAN2,0,1,rec,200,0))>0)
    {
        //ROS_INFO("1111111111111k"); 
        
        for(j=0;j<reclen;j++)
        {
            if(rec[j].ID==TIME_ID)   //时间帧       调好了        
            {
              /*  ROS_INFO("收到时间帧 ID==%x: [%x] [%x] [%x] [%x] [%x] [%x] [%x] [%x]", 
                        rec[j].ID,rec[j].Data[0], rec[j].Data[1], rec[j].Data[2],
                        rec[j].Data[3],rec[j].Data[4],rec[j].Data[5], rec[j].Data[6], 
                        rec[j].Data[7]);  //原始数据显示*/
               //具体移几位参考羲朗数据手册42页
              // Nav_Longitude = rec[j].Data[3] << 24 + rec[j].Data[2] << 16 + rec[j].Data[1] << 8 + rec[j].Data[0]; //导航经度
              // Nav_Latitude  = rec[j].Data[7] << 24 + rec[j].Data[6] << 16 + rec[j].Data[5] << 8 + rec[j].Data[4]; //导航纬度

               Week_Time = (int) (Conver_Data_2byte(rec[j].Data[0],rec[j].Data[1],0));       //周数
               Gps_Time = (float)((Conver_Data_4byte(rec[j].Data[2],rec[j].Data[3],rec[j].Data[4],rec[j].Data[5],0))*Scalefactor_4);     //秒数

        //    ROS_INFO("得到数据:周数[%d],秒数[%f]", Week_Time,Gps_Time);  //初次处理数据显示
            }
            else if(rec[j].ID==IMUANG_ID)     //IMU角速度帧
            {
              /*  ROS_INFO("收到角速度帧 ID==%x: [%x] [%x] [%x] [%x] [%x] [%x] [%x] [%x]", 
                        rec[j].ID,rec[j].Data[0], rec[j].Data[1], rec[j].Data[2],
                        rec[j].Data[3],rec[j].Data[4],rec[j].Data[5], rec[j].Data[6], 
                        rec[j].Data[7]);  //原始数据显示*/
            /*   Ang_Rate_RawX = (float)((short)(rec[j].Data[0] << 12) + (short)(rec[j].Data[1] << 8) + (short)((rec[j].Data[2] & 0b11110000) >> 4  ))*Scalefactor_4;  //IMU-> X轴角速度
               Ang_Rate_RawY = (float)((short)(rec[j].Data[4] << 16) + (short)(rec[j].Data[3] << 8) + (short)((rec[j].Data[2] & 0b00001111)       ))*Scalefactor_4; //IMU-> Y轴角速度
               Ang_Rate_RawZ = (float)((short)(rec[j].Data[5]      ) + (short)(rec[j].Data[6] << 8) + (short)((rec[j].Data[7] & 0b11110000) << 12 ))*Scalefactor_4;  //IMU-> Z轴角速度*/
                Ang_Rate_RawX = (float)((short)(rec[j].Data[0] << 12) + (short)(rec[j].Data[1] << 4) + (short)((rec[j].Data[2] & 0b11110000) >> 4 ))*Scalefactor_4;  //IMU-> X轴角速度
                Ang_Rate_RawY = (float)((short)(rec[j].Data[4]      ) + (short)(rec[j].Data[3] << 8) + (short)((rec[j].Data[2] & 0b00001111) << 16))*Scalefactor_4;  //IMU-> Y轴角速度
                Ang_Rate_RawZ = (float)((short)(rec[j].Data[5] << 12) + (short)(rec[j].Data[6] << 4) + (short)((rec[j].Data[7] & 0b11110000) >> 4 ))*Scalefactor_4;  //IMU-> Z轴角速度
                //ROS_INFO("得到数据:IMU-X角[%f],IMU-Y角[%f],IMU-Z角[%f]", Ang_Rate_RawX,Ang_Rate_RawY,Ang_Rate_RawZ);  //初次处理数据显示
            }
            else if(rec[j].ID==IMUACC_ID)     //IMU加速度帧
            {
           //    Accel_RawX = (float)(Conver_Data_4byte(0x00,((rec[j].Data[2]&0xF0)>>4),rec[j].Data[1],rec[j].Data[0],1))*Scalefactor_4;  //IMU-> X轴加速度
           //    Accel_RawY = (float)(Conver_Data_4byte(0x00,(rec[j].Data[2]&0x0F),rec[j].Data[3],rec[j].Data[4],1))*Scalefactor_4;     //IMU-> Y轴加速度
           //    Accel_RawZ = (float)(Conver_Data_4byte(0x00,rec[j].Data[5],rec[j].Data[6],((rec[j].Data[7]&0xF0)>>4),1))*Scalefactor_4;  //IMU-> Z轴加速度
               Accel_RawX = (float)((short)(rec[j].Data[0] << 12) + (short)(rec[j].Data[1] << 4) + (short)((rec[j].Data[2] & 0b11110000) >> 4 ))*Scalefactor_4;   //IMU-> X轴加速度
               Accel_RawY = (float)((short)(rec[j].Data[4]      ) + (short)(rec[j].Data[3] << 8) + (short)((rec[j].Data[2] & 0b00001111) << 16))*Scalefactor_4; //IMU-> Y轴加速度
               Accel_RawZ = (float)((short)(rec[j].Data[5] << 12) + (short)(rec[j].Data[6] << 4) + (short)((rec[j].Data[7] & 0b11110000) >> 4 ))*Scalefactor_4;  //IMU-> Z轴加速度
              // ROS_INFO("得到数据:IMU-X加[%f],IMU-Y加[%f],IMU-Z加[%f]", Accel_RawX,Accel_RawY,Accel_RawZ);  //初次处理数据显示
            }
            else if(rec[j].ID==IMSSTATE_ID)          //INS定位状态帧   调好了
            {
            /*    ROS_INFO("收到载体系速度帧 ID==%x: [%x] [%x] [%x] [%x] [%x] [%x] [%x] [%x]", 
                        rec[j].ID,rec[j].Data[0], rec[j].Data[1], rec[j].Data[2],
                        rec[j].Data[3],rec[j].Data[4],rec[j].Data[5], rec[j].Data[6], 
                        rec[j].Data[7]);  //原始数据显示*/
                System_State = (int)rec[j].Data[0];    //系统状态
                Gps_Num = (int)rec[j].Data[1];    //主天线卫星数
                Satellite_State = (int)rec[j].Data[2];    //卫星状态
                Gps_Num2 = (int)rec[j].Data[3];    //辅天线卫星数
                Gps_Age = (float)((Conver_Data_2byte(rec[j].Data[4],rec[j].Data[5],0))*Scalefactor_2);    //差分延时
                Gps_Num_Sats = (int)rec[j].Data[6];     //主天线搜星数
                Gps_Num_Sats2 = (int)rec[j].Data[7];     //辅天线搜星数
           /*     ROS_INFO("得到数据:系统状态[%d],主卫星[%d],卫星状态[%d],辅卫星[%d],差分延时[%f],主搜星[%d],辅搜星[%d]", 
                System_State,Gps_Num,Satellite_State,Gps_Num2,Gps_Age,Gps_Num_Sats,Gps_Num_Sats2);  //初次处理数据显示 */

            }
            else if(rec[j].ID==LOCATION_ID)     //定位经纬度帧           调好了
            {
                Pos_Lat = (long double)((Conver_Data_4byte(rec[j].Data[0],rec[j].Data[1],rec[j].Data[2],rec[j].Data[3],1))*Scalefactor_7);    //纬度
                Pos_Lon = (long double)((Conver_Data_4byte(rec[j].Data[4],rec[j].Data[5],rec[j].Data[6],rec[j].Data[7],1))*Scalefactor_7);    //经度
                
               ROS_INFO("得到数据:经度[%.7f],纬度[%.7f]", Pos_Lon,Pos_Lat);  //初次处理数据显示
                GNSS_Data_Conver(Pos_Lon,Pos_Lat);    //球面坐标转为平面坐标
                Position_Signal = Ture;
            }
            else if(rec[j].ID==GROHIG_ID)    //大地高度帧                 调好了
            {
                   /*                  ROS_INFO("收到角速度帧 ID==%x: [%x] [%x] [%x] [%x] [%x] [%x] [%x] [%x]", 
                        rec[j].ID,rec[j].Data[0], rec[j].Data[1], rec[j].Data[2],
                        rec[j].Data[3],rec[j].Data[4],rec[j].Data[5], rec[j].Data[6], 
                        rec[j].Data[7]);  //原始数据显示*/  // 有
              Pos_Alt = (float)(Conver_Data_4byte(rec[j].Data[0],rec[j].Data[1],rec[j].Data[2],rec[j].Data[3],1))*Scalefactor_3;    //高度

           //    ROS_INFO("得到数据:高度[%f]", Pos_Alt);  //初次处理数据显示
             //    ROS_INFO("得到数据:右角速度[%f],前角速度[%f],上角速度[%f],上加速度[%f]", X_Aixs_Ang_Vel,Y_Aixs_Ang_Vel,Z_Aixs_Ang_Vel,Z_Aixs_Accelerate);  //初次处理数据显示
             //    Ang_Vel_Signal = Ture;    //前角速度更新信号
            }
            else if(rec[j].ID==SIGMA_ID)         //位置西格玛帧      调好了?
            {
               Pos_Sigma_East  = (float)(Conver_Data_2byte(rec[j].Data[0],rec[j].Data[1],0))*Scalefactor_2;       //东向西格玛
               Pos_Sigma_North = (float)(Conver_Data_2byte(rec[j].Data[2],rec[j].Data[3],0))*Scalefactor_2;     //北向西格玛
               Pos_Sigma_Up    = (float)(Conver_Data_2byte(rec[j].Data[4],rec[j].Data[5],0))*Scalefactor_2;         //天向西格玛
           //    ROS_INFO("得到数据:东向Sig[%f],北向Sig[%f],天向Sig[%f]", Pos_Sigma_East,Pos_Sigma_North,Pos_Sigma_Up);  //初次处理数据显示
            }
            else if(rec[j].ID==GROVEL_ID)    //大地坐标系速度帧          调好了
            {
                Vel_East  = (float)(Conver_Data_2byte(rec[j].Data[0],rec[j].Data[1],1))*Scalefactor_2;       //东向速度
                Vel_North = (float)(Conver_Data_2byte(rec[j].Data[2],rec[j].Data[3],1))*Scalefactor_2;       //北向速度
                Vel_Up    = (float)(Conver_Data_2byte(rec[j].Data[4],rec[j].Data[5],1))*Scalefactor_2;       //天向速度
                Vel_Vehicle = (float)(Conver_Data_2byte(rec[j].Data[6],rec[j].Data[7],1))*Scalefactor_2;       //车辆速度
            //    ROS_INFO("得到数据:东向速度[%f],北向速度[%f],天向速度[%f],车辆速度[%f]", Vel_East,Vel_North,Vel_Up,Vel_Vehicle*3.6);  //初次处理数据显示
            }
            else if(rec[j].ID==GROVELSIG_ID)    //大地坐标系速度西格玛帧    调好了
            {
                Vel_Sigma_East  = (float)(Conver_Data_2byte(rec[j].Data[0],rec[j].Data[1],0))*Scalefactor_2;       //东向速度西格玛
                Vel_Sigma_North = (float)(Conver_Data_2byte(rec[j].Data[2],rec[j].Data[3],0))*Scalefactor_2;       //北向速度西格玛
                Vel_Sigma_Up    = (float)(Conver_Data_2byte(rec[j].Data[4],rec[j].Data[5],0))*Scalefactor_2;       //天向速度西格玛
                Vel_Sigma_Vehicle = (float)(Conver_Data_2byte(rec[j].Data[6],rec[j].Data[7],0))*Scalefactor_2;       //车辆速度西格玛
            //    ROS_INFO("得到数据:东速Sig[%f],北速Sig[%f],天速Sig[%f],车速Sig[%f]", Vel_Sigma_East,Vel_Sigma_North,Vel_Sigma_Up,Vel_Sigma_Vehicle);  //初次处理数据显示
            }
            else if(rec[j].ID==ACCEL_ID)         //车辆坐标系加速度帧
            {
               // Accel_X = (float)(Conver_Data_4byte(0x00,((rec[j].Data[2]&0xF0)>>4),rec[j].Data[1],rec[j].Data[0],1))*Scalefactor_4;  //车辆系-> X轴加速度
                //Accel_X = (float)(Conver_Data_2_5byte(rec[j].Data[0],rec[j].Data[1],rec[j].Data[2],1)*Scalefactor_4);
                Accel_X = (float)((short)(rec[j].Data[0] << 12) + (short)(rec[j].Data[1] << 4) + (short)((rec[j].Data[2] & 0b11110000) >> 4 ))*Scalefactor_4;  //车辆系-> X轴加速度
                Accel_Y = (float)((short)(rec[j].Data[4]      ) + (short)(rec[j].Data[3] << 8) + (short)((rec[j].Data[2] & 0b00001111) << 16))*Scalefactor_4;  //车辆系-> Y轴加速度
                Accel_Z = (float)((short)(rec[j].Data[5] << 12) + (short)(rec[j].Data[6] << 4) + (short)((rec[j].Data[7] & 0b11110000) >> 4 ))*Scalefactor_4;  //车辆系-> Z轴加速度
               // ROS_INFO("得到数据:车X加[%f],车Y加[%f],车Z加[%f]", Accel_X,Accel_Y,Accel_Z);  //初次处理数据显示
            }
            else if(rec[j].ID==ATTANG_ID)       //姿态角帧        调好了
            {
         /*     ROS_INFO("收到姿态角帧 ID==%x: [%x] [%x] [%x] [%x] [%x] [%x] [%x] [%x]", 
                        rec[j].ID,rec[j].Data[0], rec[j].Data[1], rec[j].Data[2],
                        rec[j].Data[3],rec[j].Data[4],rec[j].Data[5], rec[j].Data[6], 
                        rec[j].Data[7]);  //原始数据显示*/ 
                Yaw_Angle   = (float) ((Conver_Data_2byte(rec[j].Data[0],rec[j].Data[1],0))*Scalefactor_2);       //航向角
                Pitch_Angle = (float) ((Conver_Data_2byte(rec[j].Data[2],rec[j].Data[3],1))*Scalefactor_2);       //俯仰角
                Roll_Angle  = (float) ((Conver_Data_2byte(rec[j].Data[4],rec[j].Data[5],1))*Scalefactor_2);       //横滚角
             //   ROS_INFO("得到数据:航向角[%f],俯仰角[%f],横滚角[%f]", Yaw_Angle,Pitch_Angle,Roll_Angle);  //初次处理数据显示
            }
            else if(rec[j].ID==ATTANGSIG_ID)        //姿态角西格玛帧     调好了
            {
                Yaw_Angle_Sigma   = (float)(Conver_Data_2byte(rec[j].Data[0],rec[j].Data[1],0))*Scalefactor_2;       //航向角西格玛
                Pitch_Angle_Sigma = (float)(Conver_Data_2byte(rec[j].Data[2],rec[j].Data[3],0))*Scalefactor_2;       //俯仰角西格玛
                Roll_Angle_Sigma  = (float)(Conver_Data_2byte(rec[j].Data[4],rec[j].Data[5],0))*Scalefactor_2;       //横滚角西格玛
            //    ROS_INFO("得到数据:航向角Sig[%f],俯仰角Sig[%f],横滚角Sig[%f]", Yaw_Angle_Sigma,Pitch_Angle_Sigma,Roll_Angle_Sigma);  //初次处理数据显示
            }
            else if(rec[j].ID==ANGVEL_ID)           //车辆坐标系角速度
            {
               //GNSS_Longitude = rec[j].Data[0] << 24 + rec[j].Data[1] << 16 + rec[j].Data[2] << 8 + rec[j].Data[3];       //GNSS经度
               //GNSS_Latitude =  rec[j].Data[4] << 24 + rec[j].Data[5] << 16 + rec[j].Data[6] << 8 + rec[j].Data[7];        //GNSS纬度
                Ang_Rate_X = (float)((short)(rec[j].Data[0] << 12) + (short)(rec[j].Data[1] << 4) + (short)((rec[j].Data[2] & 0b11110000) >> 4 ))*Scalefactor_4;  //车辆坐标系-> X轴角速度
                Ang_Rate_Y = (float)((short)(rec[j].Data[4]      ) + (short)(rec[j].Data[3] << 8) + (short)((rec[j].Data[2] & 0b00001111) << 16))*Scalefactor_4;  //车辆坐标系-> Y轴角速度
                Ang_Rate_Z = (float)((short)(rec[j].Data[5] << 12) + (short)(rec[j].Data[6] << 4) + (short)((rec[j].Data[7] & 0b11110000) >> 4 ))*Scalefactor_4;  //车辆坐标系-> Z轴角速度
           //     ROS_INFO("得到数据:车X角[%f],车Y角[%f],车Z角[%f]", Ang_Rate_X,Ang_Rate_Y,Ang_Rate_Z);  //初次处理数据显示
            
            }
            else
            {
              ROS_INFO("出现未知CAN.ID号:%x",rec[j].ID);
            }
            
        }
    }

}




/***************************************************
*函数名：InitCAN()
*输入：void
*功能：初始化CAN总线
****************************************************/
void sub_send::InitCAN()
{
    
    Start_CAN();                                        //打开设备
    VCI_INIT_CONFIG config;                             //初始化CAN的配置
    config.AccCode=0;                                   //验收码
    config.AccMask=0xffffffff;                          //屏蔽码
    config.Filter=1;                                    //接收所有帧                                   
    config.Timing0=0x00;                                //波特率500 Kbps
    config.Timing1=0x1C;
    config.Mode=0;                                      //正常模式




    if(VCI_InitCAN(VCI_USBCAN2,CAN_NUM,0,&config)!=1)
    {
        printf(">>Init CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);
    }
    if(VCI_StartCAN(VCI_USBCAN2,CAN_NUM,0)!=1)
    {
        printf(">>Start CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);
    }

}





/***************************************************
*函数名：Close_CAN()
*输入：void
*功能：关闭CAN总线
****************************************************/
void sub_send::Close_CAN()
{
    VCI_CloseDevice(VCI_USBCAN2,CAN_NUM);
    printf(">>Close CAN1 Device\n");
}




/***************************************************
*函数名：Start_CAN()
*输入：void
*功能：打开CAN总线
****************************************************/
void sub_send::Start_CAN()
{
    printf(">>Start CAN2 Device\n");
    if(VCI_OpenDevice(VCI_USBCAN2,CAN_NUM,0)==1)              //打开设备
    {
        printf(">>open deivce2 success!\n");             //打开设备成功
    }     
    else
    {   
        printf(">>open deivce2 error!\n");
        exit(1);
    }
    
}





/***************************************************
*函数名：sub_send::Verify_Conversion(unsigned char Conver_Data[8])
*输入：Conver_Data[8]
*返回值：异或校验位
*功能：Verify_Conversion
****************************************************/
unsigned char sub_send::Verify_Conversion(unsigned char Conver_Data[])
{
    int count=0;
    unsigned char Verify_Value = 0x00;
    Verify_Value=Conver_Data[0]^Conver_Data[1]^Conver_Data[2]^Conver_Data[3]^Conver_Data[4]^Conver_Data[5]^Conver_Data[6];
    return Verify_Value;
}


void timerCallback(const ros::TimerEvent&)   //此函数为读取GNSS信号函数
{
  //ROS_INFO("Callback 1 triggered");
  //Receive_Data();
 /*       gnss_message.Longitude = 1;
      gnss_message.Latitude = 1; 
      pub.publish(gnss_message);
        ROS_INFO("Callback 1 triggered");*/
}


/***************************************************
*函数名：main(int argc, char **argv)
*输入：void
*返回值：void
*功能：main
****************************************************/
int main(int argc, char **argv)
{   
    ros::init(argc, argv, "GNSS_CAN");
    ros::NodeHandle nh;
    //ROS_INFO("死111循环");
    //ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);// 0.01秒中断收数据

/***************************************************
*功能：此段程序为通过CAN硬件号自动分配序列号
*注意:在用到CAN序列号的地方都换成CAN_NUM了
****************************************************/
    int num=VCI_FindUsbDevice2(pInfo);
    string strtemp,str;

    for(int i=0;i<num;i++)
    {
        str="";
        for(int j=0;j<11;j++)
        {
            if(pInfo [i].str_Serial_Num [j] != ' ')   //序列号不为空才放进去
                {
                    str+=pInfo [i].str_Serial_Num [j];
                    ROS_INFO("CAN_ID==%c",pInfo [i].str_Serial_Num [j]);
                }
        }
        ROS_INFO("CAN_ID==%s",str);
        if(str == CAN_ID)
        {
            CAN_NUM = i;
            ROS_INFO("CAN_NUM==%d",CAN_NUM);
        }
    }
/**************************************************/
    sub_send core(nh);     
    
    ros::Rate loop_rate(100);
    //Receive_Data();
    while (ros::ok()) //循环读取数据
    {
       //setlocale(LC_CTYPE, "zh_CN.utf8");   //这条语句让ROS_INFO能显示中文
       //ROS_INFO("死循环");
        system("stty -icanon"); 
        Input_Key=GetInput();
        if(Input_Key!=0)
            printf (" Input_Key=%d\n",Input_Key);
       Receive_Data();
      if(Position_Signal == Ture)    //位置信号更新
       {
        nav_msg.latitude = lattitude_X;
        nav_msg.longitude = longitude_Y;
        nav_msg.course_angle = Yaw_Angle;
        //pub.publish(nav_msg);
        navpub.publish(nav_msg);
        ROS_INFO("lattitude_X=%lf",lattitude_X);
        ROS_INFO("lattitude_X=%lf",longitude_Y);
        ROS_INFO("lattitude_X=%f",Yaw_Angle);
        //save_point();
        Position_Signal = False;
       }

       if(Input_Key==115||Input_Key==83)   //按下大小写s 把当前位置点和航向角保存下来
       {
            /*nav_msg.map_longitude = lattitude_X; 
            nav_msg.map_latitude = longitude_Y;
            nav_msg.map_course_angle = Yaw_Angle;
            navpub.publish(nav_msg);*/
            lattitude_X = (float)(lattitude_X - lattitude_Zero);     //减去零点
            longitude_Y = (float)(longitude_Y - longitude_Zero);     //减去零点

            //ofstream out("/home/dashbear/GNSS_PROJECT/src/GNSS_driver/gnss_data2",std::ios::app);

            ofstream out("/home/robot/Robot/Smart_robot_ws/src/GNSS_driver/global_planner_data/gnss_data",std::ios::app);
            //out << setprecision(12) << lattitude_X << ',' << longitude_Y << ',' << volety <<'\n';  //',' << Angle 
            out << setprecision(12) << lattitude_X << ',' << longitude_Y << ',' << Yaw_Angle <<'\n';
            out.close();
            ROS_INFO("save");
       }

    }  
    return 0;


}










