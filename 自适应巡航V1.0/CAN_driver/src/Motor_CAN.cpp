/***********************************************************
*Author:YINPENGFEI
*Function:控制
*Node:Motor_CAN
*Pub_msg:Motor_Feedback     
*Sub_msg:Motor_Control      
*************************************************************/

#include "cmath"
#include "ros/ros.h"
#include "CAN_driver/controlcan.h"
#include "CAN_driver/Motor_Control.h"
#include "CAN_driver/Motor_Feedback.h"
#include "std_msgs/UInt8.h"
#include <std_msgs/String.h>               
#include <sstream>
#include <stdio.h>                         //标准输入输出定义          
#include <stdlib.h>                        //标准函数库定义
#include <unistd.h>                        //Unix 标准函数定义  
#include "string.h" 
#include "std_msgs/Float32.h"


using namespace std;


#define HandBrake_Unlock 0;                //拉起手刹
#define HandBrake_Lock 1;                  //放下手刹



//CAN匹配
string ProductSn[50];
VCI_BOARD_INFO pInfo [50];
string CAN_ID = "21A10000CC5";
int CAN_NUM_2;

//string ProductSn[50];
//VCI_BOARD_INFO pInfo [50];
int num=VCI_FindUsbDevice2(pInfo);
string strtemp,str;


int Keep_Time = 50;
int Hkeep_Time = 20;
int Gkeep_Time = 200;

unsigned int Gear_R = 2;
unsigned int Gear_N = 3;
unsigned int Gear_D = 4;

unsigned int Light_Close_Flag = 0;         //关闭 
unsigned int Light_Left_Flag = 1;          //左转标识
unsigned int Light_Right_Flag = 2;         //右转标识
unsigned int Light_Warning_Flag = 3;       //双闪
unsigned int Light_Keep_Flag = 0;          //灯保持标识

unsigned int GearEND_SendFlag = 0;         //挂档发送标识
unsigned int GearENR_SendFlag = 0;         //挂档发送标识
unsigned int Manual_Flag = 0;              //手动驾驶标识
unsigned int Auto_Flag = 1;                //自动驾驶标识
unsigned int GearEN_Flag = 2;              //自动驾驶 档位使能 
unsigned int GasBrakeEn_Flag = 3;          //进入自动驾驶模式 纵向使能
unsigned int GearGasBrakeEn_Flag = 4;      //自动驾驶、档位、纵向使能
/***************************************************
*功能：定义CAN数据发送结构体
*功能：定义发布、接收话题结构体 
****************************************************/
class sub_send
{
    public: 
        sub_send(ros::NodeHandle &nh);
        ~sub_send();
        unsigned char Verify_Conversion(unsigned char Conver_Data[]);                     //奇偶校验转化
        void AEB_TarAcc_Conversion(unsigned int C_AEB_Tar);                               //自动制动加速度物理值转化到总线值 
        void TargetAcc_Conversion(unsigned int C_Acc);                                    //纵向加速度物理值转化到总线值
        void Target_Brake_Conversion(unsigned int C_BrakeAcc);                            //纵向减速度物理值转化到总线值
        void Angle_Conversion(int C_Angle);                                               //角度物理值转化到总线值
        void Velocity_Conversion(unsigned int C_Velocity);                                //角速度物理值转化到总线值

        void InitCAN();                                                                   //CAN初始化
        void Start_CAN();                                                                 //打开CAN
        void Close_CAN();                                                                 //关闭CAN （每次都要关闭）
        void Receive_Data();                                                              //接收的数据
        
        void Drive_Mode_Select(int Auto, int Gear_en, int AEB_en, int Acc_en, int Esp_en);            //模式使能
        unsigned char Gear_Select(int Gear_Mode);
        void HandBrake_Select(int HandBrake_Mode, int Gear_Choose);
        void Forward_Run(int F_Acc);                                                      //向前走
        void Rear_Run(int Re_Acc);                                                              //倒车
        void Steer_Turn( int S_Angle);                                            //转向
        void BcmControl2(unsigned int TurnLight);                                      //给ID0x10发送转向灯指令
        void Send_Data_ID02(unsigned int S_Acc ,unsigned int S_Direct_Flag);        //给ID0X11发送驾驶先决条件、档位、加速度、加速度方向
        void Send_Data_ID03(int Send_Angle );                                                         //给ID0x12发送方向盘角度、角速度、使能
        void Send_Data();                                                                 //将三个ID数据的Buf发送到CAN

    private:
        VCI_CAN_OBJ rec[200];                                                             //CAN接收缓存
        VCI_CAN_OBJ sendbuf[3];                                                           //CAN发送三帧 三个ID
        VCI_CAN_OBJ sendbuf_mems[1]; 
        unsigned char SendData_ID01[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};         //初始化SendData_ID01[]
        unsigned char SendData_ID02[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};         //初始化SendData_ID02[]
        unsigned char SendData_ID03[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};         //初始化SendData_ID03[]     
        int i;
        int j;
//Send
        /**************************************ID：0x10************************************/
        /*车门控制 Byte2*/
        unsigned char Door_Unlock = 0x0C;                            //解锁 
        unsigned char Door_Lock = 0x08;                              //上锁 
        unsigned char Door_Keep = 0x00;                              //保持
        /*转向灯控制 Byte3*/
        unsigned char Byte3 = 0x00;                                  //初始化Byte3  
        unsigned char Light_Left = 0x01;                             //左转灯
        unsigned char Light_Right = 0x02;                            //右转灯
        unsigned char Light_Warning = 0x03;                          //报警灯
        unsigned char Light_Close = 0x00;                            //灯关闭
        unsigned int Light_Flag = 0;                                 //灯复位标志
        
        
        /***********************************ID:0x11***************************************/
        /*Byte0*/
        unsigned int Auto_Speed_Limit = 0;                           //车辆限速
        unsigned char Spd_Limit = 0x00;                              //车辆限速  20km/h
        /*Byte1*/
        unsigned int AEB_Acc = 0;                                    //AEB减速度
        unsigned char AEB_TarAcce_H = 0x00;                          //减速度高字节  AEB_en 0x90         0.1
        /*Byte2*/
        unsigned char AEB_TarAcce_L = 0x00;                          //减速度低字节  0x15
        /*Byte3*/
        unsigned int Exp_Acc = 0;                                    //期望加速度
        unsigned char TargetAcc = 0x00;                              //加速度 0.2 0x04
        /*Byte4*/
        unsigned int Exp_BrakeAcc = 0;                               //期望减速度
        unsigned char Target_Brake = 0x00;                           //制动控制      0x0a 
        /*Byte5*/
        unsigned int GEPB_Flag = 0;                                  //档位标识  R2 N3 D4
        unsigned int Gear_Flag = 0;
        unsigned int Gear_Data = 0x00;
        unsigned char Target_GEPB = 0x00;                            //档位控制
        unsigned char Target_GEPB_R = 0x02;                          //档位R控制
        unsigned char Target_GEPB_N = 0x03;                          //档位N控制
        unsigned char Target_GEPB_D = 0x04;                          //档位D控制
        unsigned int  Target_ReGEPB_Flag = 0;                        //挂档复位标识 =0 复位
        unsigned char HandBrake_Data = 0x00;
        unsigned char Unlock_Handbrake = 0x20;                       //实施手刹
        unsigned char Lock_Handbrake = 0x10;                         //解除手刹
        unsigned int HandBrake_Flag = 0;
        unsigned int HandBrake_Success_Flag = 0;
        /*Byte6驾驶先决条件*/
        unsigned int DrivingReq_Flag = 0;                            //自动驾驶先决条件标志 
        unsigned char DrivingReq_Mode = 0x00;                        //驾驶模式                 
        unsigned char Driving_Mode_Manual = 0x00;                    //手动                    DrivingReq_Flag = 0
        unsigned char Driving_Mode_Auto = 0x10;                      //自动                    DrivingReq_Flag=1
        unsigned char DrivingReq_GearEN = 0x20;                      //档位使能 自动驾驶         DrivingReq_Flag = 2
        unsigned char DrivingReq_AEBenable = 0x40;                   //AEB使能
        unsigned char DrivingReq_GasBrake_EN = 0x80;                 //进入自动驾驶模式 纵向使能   DrivingReq_Flag = 3
        unsigned char DrivingReq_GearGasBrake_EN = 0xB0;             //自动驾驶、档位、纵向使能    DrivingReq_Flag = 4
        unsigned char DrivingReq_EN = 0xF0;                          //自动驾驶全部使能           DrivingReq_Flag = 5


        /***************************************ID:0x12***********************************/     
        /*Byte0*/
        unsigned int Esp_Velocity = 0;                               //角速度
        unsigned char TargetAng_Speed = 0x00;                        //方向盘角速度
        /*Byte1*/
        unsigned int Esp_Anglge = 0;                                 //角度
        unsigned char TargetAng_H = 0x22;                            //方向盘转角高字节 0x36 0x22 0x37
        /*Byte2*/
        unsigned char TargetAng_L = 0x12;                            //方向盘转角低字节 0x5f 0x12 0xfb
        /*Byte6*/
        unsigned char ESP_EN = 0x40;                                 //转向使能 进入自动驾驶模式有效


        unsigned int CAN_Flag = 0;                                   //CAN状态标识   
 

//Recive 

        /*0x13_FB*/
        unsigned char Ang_H = 0x00;                                  //BYTE0 方向盘转角高字节
        unsigned char Ang_L = 0x00;                                  //BYTE1 方向盘转角低字节
        int R_Angle = 0;                                             //反馈回来的角度
        int Steer_Angle = 0;
        int Steer_Angle_New = 0;
        int Steer_Angle_Old = 0;
        int Steer_Angle_In = 0;
        float Tire_Angle = 0.0;
        float Path_steer = 0.0;
        float Path_acc = 0.0;
        int purepursuit_acc = 0;
        int target_acc = 0;
        int actual_acc = 0;
        int Purepursuit_Angle = 0;
        int Steer_Angle_Delta = 0;
        int Angle_Gain = 8.1;      //10 6 8     //7.5  //5    // 10km/h (9)


        unsigned char ACC_FB = 0x00;                                 //BYTE2 加速度反馈
        float R_Acc = 0.0;                                           //反馈回来的加速度
        
        unsigned char Brake_FB = 0x00;                               //BYTE3 减速度反馈
        float R_Brake = 0.0;                                         //反馈回来的减速度
        
        unsigned char vehspd_H = 0x00;                               //BYTE4 车速高字节
        unsigned char Vehspd_L = 0X00;                               //BYTE5 车速低字节
        float R_Vehspd = 0.0;                                        //反馈回来的车速
        
        unsigned char Gear_FB = 0x00;                                //BYTE6 档位反馈
        unsigned char Gear_D_FB = 0x40;                              //反馈回的D档
        unsigned char Gear_N_FB = 0x30;                              //反馈回的N档
        unsigned char Gear_R_FB = 0x20;                              //反馈回的R档
        unsigned int R_Gear_Flag = 0;                                //反馈回来的档位标志 2R 3N 4D



        /*0x15_FB*/
        unsigned char FL_Speed_H = 0x00;                             //BYTE0 左前轮车速，高位
        unsigned char FL_Speed_L = 0X00;                             //BYTE1 左前轮车速，低位
        float FL_Spd_FB = 0.0;                                       //左前轮速度反馈

        unsigned char FR_Speed_H = 0x00;                             //BYTE2 右前轮车速，高位
        unsigned char FR_Speed_L = 0X00;                             //BYTE3 右前轮车速，低位
        float FR_Spd_FB = 0.0;                                       //右前轮速度反馈

        unsigned char BL_Speed_H = 0x00;                             //BYTE4 左后轮车速，高位
        unsigned char BL_Speed_L = 0x00;                             //BYTE5 左后轮车速，低位
        float BL_Spd_FB = 0.0;                                       //左后轮速度反馈

        unsigned char BR_Speed_H = 0x00;                             //BYTE6 右后轮车速，高位
        unsigned char BR_Speed_L = 0x00;                             //BYTE7 右后轮车速，低位       
        float BR_Spd_FB = 0.0;                                       //右后轮速度反馈


        ros::Subscriber Control_Sub;
        ros::Subscriber Purepursuit_Sub;
        ros::Subscriber Purepursuit_acc_Sub;
        ros::Subscriber AvoidPlan_Sub;
        ros::Subscriber Avoidspeed_Sub;
        ros::Publisher Motor_Pub;
        CAN_driver::Motor_Feedback Feedback_msg;
        void MotorControl_callback(const CAN_driver::Motor_Control::ConstPtr& Control_msg);
        void Purepursuit_callback(const std_msgs::Float32::ConstPtr& purepursuit_msg);
        void PurepursuitAcc_callback(const std_msgs::Float32::ConstPtr& purepursuit_acc_msg);
        void AvoidPlan_callback(const std_msgs::Float32::ConstPtr& avoidpath_steer_msg);
        void AvoidPlanSpeed_callback(const std_msgs::Float32::ConstPtr& avoidpath_speed_msg);
};

sub_send::sub_send(ros::NodeHandle &nh)
{
    InitCAN();
    Control_Sub = nh.subscribe("Motor_Control_mssage",10,&sub_send::MotorControl_callback,this);
    Purepursuit_Sub = nh.subscribe("delta",10,&sub_send::Purepursuit_callback,this);
    Purepursuit_acc_Sub = nh.subscribe("c_a",10,&sub_send::PurepursuitAcc_callback,this);
    //AvoidPlan_Sub = nh.subscribe("Car_Steering",10,&sub_send::AvoidPlan_callback,this);
    //Avoidspeed_Sub = nh.subscribe("Car_Velocity",10,&sub_send::AvoidPlanSpeed_callback,this);
    Motor_Pub = nh.advertise<CAN_driver::Motor_Feedback>("Motor_Feedback_mssage", 10);  
    ros::spin();  //只执行一次
}

sub_send::~sub_send(){}

void sub_send::AvoidPlanSpeed_callback(const std_msgs::Float32::ConstPtr& avoidpath_speed_msg)
{

    Path_acc = avoidpath_speed_msg->data;
    printf("Path_acc %f\n" , Path_acc);
    
}


void sub_send::AvoidPlan_callback(const std_msgs::Float32::ConstPtr& avoidpath_steer_msg)
{

    Path_steer = avoidpath_steer_msg->data;
    printf("Path_steer %f\n" , Path_steer);
    Steer_Angle = (int) (Angle_Gain * Path_steer);
    //printf("R_Vehspd %f\n" , R_Vehspd);
    printf("Steer_Angle %d\n" , Steer_Angle);
    
}


void sub_send::PurepursuitAcc_callback(const std_msgs::Float32::ConstPtr& purepursuit_acc_msg)
{
   purepursuit_acc = (int) purepursuit_acc_msg->data;
   printf("Tire_Angle %f\n" , Tire_Angle);
}


void sub_send::Purepursuit_callback(const std_msgs::Float32::ConstPtr& purepursuit_msg)
{

    Tire_Angle = purepursuit_msg->data;
    printf("Tire_Angle %f\n" , Tire_Angle);
    Steer_Angle = (int) (Angle_Gain * Tire_Angle);
    //printf("R_Vehspd %f\n" , R_Vehspd);
    printf("Steer_Angle %d\n" , Steer_Angle);
    
}




void sub_send::MotorControl_callback(const CAN_driver::Motor_Control::ConstPtr& Control_msg )
{

    //ROS_INFO("I recive msg:  [%x] [%x] [%x] [%x]", Control_msg->Acc_Value,Control_msg->Angle_Value,Control_msg->CAN_Start ,Control_msg->CAN_Stop);
    if(Control_msg->CAN_Start == 1)
    {   
        if(CAN_Flag == 2)
        {   
            Drive_Mode_Select(1, 1, 0, 1, 1);
            
            
            if(HandBrake_Success_Flag == 0)                            //D档
            {
                BcmControl2(Light_Left_Flag);
                HandBrake_Select(2, 3);                                //手刹优先级最高
                for(int k; k<1000 ;k++){}
                HandBrake_Success_Flag = 1;
            }
            else if(HandBrake_Success_Flag == 1)
            {
                BcmControl2(Light_Close);
                HandBrake_Select(0, 4);                                //挂档时手刹必需为空位
            }
            //printf("actual_acc %d" , actual_acc);
            Forward_Run(purepursuit_acc);                            //前向加减速
            //Forward_Run(Control_msg->Acc_Value);                       //前向加减速 
            //Steer_Turn(Control_msg->Angle_Value);           //方向盘 
            //Rear_Run(Control_msg->Acc_Value);                        //倒车
            //printf("target_acc %d" , target_acc);
            Steer_Turn(Steer_Angle_In); 
            //Forward_Run(Path_acc);
            //Steer_Turn(Steer_Angle_In);           //方向盘  
            Send_Data();  
            Receive_Data(); 

            //printf("Steer_Angle %d\n" , Steer_Angle); 

            /*方向盘顺滑处理*/          
            if(Steer_Angle_In < Steer_Angle)
            {
                Steer_Angle_In = Steer_Angle_In + 1;
            }
            else
            {
                Steer_Angle_In = Steer_Angle_In - 1;
            }

            if(Steer_Angle == 0)
            {
                Steer_Angle_In = 0;
            }

                              
        }
        else 
        {        
            CAN_Flag = 1;
        }
    }
    else if(Control_msg->CAN_Stop == 1)
    {
        Drive_Mode_Select(0, 0, 0, 0, 0);
        //Close_CAN();
    } 
    //先等待CAN 10ms
    if(CAN_Flag == 1)
    {
        CAN_Flag = 2;
    } 
}



/***************************************************
*函数名：Send_Data()
*输入：void
*功能：发送sendbuf数据到CAN
*输出：void
****************************************************/
void sub_send::Send_Data()
{
    SendData_ID01[7] = Verify_Conversion(SendData_ID01);
    for (i = 0;i < 8;i++)
	{
	    sendbuf[0].Data[i] = SendData_ID01[i];
      }

    SendData_ID02[7] = Verify_Conversion(SendData_ID02); 
    for (i = 0;i < 8;i++)
    {
        sendbuf[1].Data[i] = SendData_ID02[i];
    }

    SendData_ID03[7] = Verify_Conversion(SendData_ID03);
    for (i = 0;i < 8;i++)
	{
	    sendbuf[2].Data[i] = SendData_ID03[i];
      } 

	if(VCI_Transmit(VCI_USBCAN2, CAN_NUM_2, 0, sendbuf, 3) == 3)
	{
        ROS_INFO("ID:ox10: [%x] [%x] [%x] [%x] [%x] [%x] [%x] [%x]", sendbuf[0].Data[0], sendbuf[0].Data[1], sendbuf[0].Data[2],sendbuf[0].Data[3],sendbuf[0].Data[4],sendbuf[0].Data[5], sendbuf[0].Data[6], sendbuf[0].Data[7]);       
        ROS_INFO("ID:ox11: [%x] [%x] [%x] [%x] [%x] [%x] [%x] [%x]", sendbuf[1].Data[0], sendbuf[1].Data[1], sendbuf[1].Data[2],sendbuf[1].Data[3],sendbuf[1].Data[4],sendbuf[1].Data[5], sendbuf[1].Data[6], sendbuf[1].Data[7]);
        ROS_INFO("ID:ox12: [%x] [%x] [%x] [%x] [%x] [%x] [%x] [%x]", sendbuf[2].Data[0], sendbuf[2].Data[1], sendbuf[2].Data[2],sendbuf[2].Data[3],sendbuf[2].Data[4],sendbuf[2].Data[5], sendbuf[2].Data[6], sendbuf[2].Data[7]);
		}
    else
	{
        //ROS_INFO("VCI_Transmit: [%d] ", Return_Transmit);
		//printf (" CAN sendbuf error");
		}

    /*sendbuf_mems[0].Data[0] = R_Gear_Flag;
    sendbuf_mems[0].Data[1] = R_Vehspd;
    sendbuf_mems[0].Data[2] = 0;
    sendbuf_mems[0].Data[3] = 0;
    sendbuf_mems[0].Data[4] = 0;
    sendbuf_mems[0].Data[5] = 0;
    sendbuf_mems[0].Data[6] = 0;
    sendbuf_mems[0].Data[7] = 0;
    if(VCI_Transmit(VCI_USBCAN2, 0, 1, sendbuf_mems, 1) == 1)
	{
        ROS_INFO("ID:ox01: [%x] [%x] [%x] [%x] [%x] [%x] [%x] [%x]", sendbuf_mems[0].Data[0], sendbuf_mems[0].Data[1], sendbuf_mems[0].Data[2],sendbuf_mems[0].Data[3],sendbuf_mems[0].Data[4],sendbuf_mems[0].Data[5], sendbuf_mems[0].Data[6], sendbuf_mems[0].Data[7]);
		}
    else
	{
        //ROS_INFO("VCI_Transmit: [%d] ", Return_Transmit);
		//printf (" CAN sendbuf error");
		}*/
    
}



/***************************************************
*函数名：Receive_Data()
*输入：void
*功能：接收CAN数据
****************************************************/
void sub_send::Receive_Data()
{
    unsigned int reclen=0,j=0;
    if((reclen=VCI_Receive(VCI_USBCAN2,CAN_NUM_2,0,rec,200,0))>0)//调用接收函数，如果有数据，进行数据处理显示。 //(reclen=VCI_Receive(VCI_USBCAN2,0,0,rec,200,0)
    {
        for(j=0;j<reclen;j++)
        {
            ROS_INFO("I recive: [%x]", rec[j].ID);
           if(rec[j].ID==0x13)
           {
               ROS_INFO("I recive ID==0x13: [%x] [%x] [%x] [%x] [%x] [%x] [%x] [%x]", rec[j].Data[0], rec[j].Data[1], rec[j].Data[2],rec[j].Data[3],rec[j].Data[4],rec[j].Data[5], rec[j].Data[6], rec[j].Data[7]);     
               Ang_H = rec[j].Data[0];
               Ang_L = rec[j].Data[1];
               R_Angle = ( ((int) Ang_H)*256 + ((int) Ang_L) )*0.1 - 870;

               ACC_FB = rec[j].Data[2];
               R_Acc = ACC_FB*0.05;

               Brake_FB = rec[j].Data[3];
               R_Brake = Brake_FB*0.05;

               vehspd_H = rec[j].Data[4];
               Vehspd_L = rec[j].Data[5];                               
               R_Vehspd = (vehspd_H*256 + Vehspd_L)*0.05625;

               if(rec[j].Data[6] == Gear_D_FB)
               {
                    R_Gear_Flag = 4;
               }
               else if(rec[j].Data[6] == Gear_R_FB)
               {
                    R_Gear_Flag = 2;
               }
               else if(rec[j].Data[6] == Gear_N_FB)
               {
                    R_Gear_Flag = 3;
               }
               else
               {
                    R_Gear_Flag = 0;    
               }
               ROS_INFO("I recive ID==0x13 FeedBack Data: [%d] [%f] [%f] [%f] [%d]", R_Angle, R_Acc, R_Brake,R_Vehspd,R_Gear_Flag);
           }
           else if(rec[j].ID==0x15)
           {
               //ROS_INFO("I recive ID==0x15: [%x] [%x] [%x] [%x] [%x] [%x] [%x] [%x]", rec[j].Data[0], rec[j].Data[1], rec[j].Data[2],rec[j].Data[3],rec[j].Data[4],rec[j].Data[5], rec[j].Data[6], rec[j].Data[7]); 
               FL_Speed_H = rec[j].Data[0];
               FL_Speed_L = rec[j].Data[1];
               FR_Speed_H = rec[j].Data[2];
               FR_Speed_L = rec[j].Data[3];
               BL_Speed_H = rec[j].Data[4];
               BL_Speed_L = rec[j].Data[5];
               BR_Speed_H = rec[j].Data[6];
               BR_Speed_L = rec[j].Data[7];
               FL_Spd_FB = (FL_Speed_H*256 + FL_Speed_L)*0.05625;
               FR_Spd_FB = (FR_Speed_H*256 + FR_Speed_L)*0.05625;
               BL_Spd_FB = (BL_Speed_H*256 + BL_Speed_L)*0.05625;
               BR_Spd_FB = (BR_Speed_H*256 + BR_Speed_L)*0.05625;
               //ROS_INFO("I recive ID==0x15 FeedBack Data: [%f] [%f] [%f] [%f]", FL_Spd_FB, FR_Spd_FB, BL_Spd_FB,BR_Spd_FB);
           }
        }
    }
    else
    {
         ROS_INFO("DON'T VCI_Receive");    
    }
    Feedback_msg.Base_Angle = R_Angle;
    Feedback_msg.Base_Acc = R_Acc;
    Feedback_msg.Base_Brake = R_Brake;
    Feedback_msg.Base_Vehspd = R_Vehspd;
    Feedback_msg.Base_Gear_Flag = R_Gear_Flag;
    Feedback_msg.Base_FL_Spd = FL_Spd_FB;
    Feedback_msg.Base_FR_Spd = FR_Spd_FB;
    Feedback_msg.Base_BL_Spd = BL_Spd_FB;
    Feedback_msg.Base_BR_Spd = BR_Spd_FB;
    Motor_Pub.publish(Feedback_msg);
}




/***************************************************
*函数名：Drive_Mode_Select()
*输入：void
*功能：驾驶模式使能
****************************************************/
void sub_send::Drive_Mode_Select(int Auto, int Gear_en, int AEB_en, int Acc_en, int Esp_en)
{
    unsigned char Auto_Data = 0x00;
    unsigned char Gear_en_Data = 0x00;
    unsigned char AEB_en_Data = 0x00;
    unsigned char Acc_en_Data = 0x00;
    unsigned char Esp_en_Data = 0x00;
    if(Auto == 1)
    {
        Auto_Data = Driving_Mode_Auto;
    }
    else
    {
        Auto_Data = 0x00;
    }

    if(Gear_en == 1)
    {
        Gear_en_Data = DrivingReq_GearEN;
    }
    else
    {
        Gear_en_Data = 0x00;
    }

    if(AEB_en == 1)
    {
        AEB_en_Data = DrivingReq_AEBenable;
    }
    else
    {
        AEB_en_Data = 0x00;
    }

    if(Acc_en == 1)
    {
        Acc_en_Data = DrivingReq_GasBrake_EN;
    }
    else
    {
        Acc_en_Data = 0x00;
    }
    /*ID02[6]*/
    SendData_ID02[6] = Auto_Data + Gear_en_Data + AEB_en_Data + Acc_en_Data;                       //驾驶先决条件

    if(Esp_en == 1)
    {
        Esp_en_Data = ESP_EN;
    }
    else
    {
        Esp_en_Data = 0x00;
    }
    /*ID03[6]*/
    SendData_ID03[6] = Esp_en_Data;                               //转向使能
}


/***************************************************
*函数名：Gear_Select()
*输入：void
*功能：档位选择
****************************************************/
unsigned char sub_send::Gear_Select(int Gear_Mode)
{
    
    
    
    if(Gear_Mode == 2)
    {

        if(Gkeep_Time > 5 && Gear_Flag == 0)
        {
            Gkeep_Time--;
            Gear_Data = Target_GEPB_N;
            SendData_ID02[4] = 0x20;                                 //期望纵向减速度，踩一点刹车
        }
        else if(Gkeep_Time <= 5)
        {
            Gkeep_Time = 100;
            Gear_Flag = 1;
            Gear_Data = Target_GEPB_R;
        }  
    }
    else if(Gear_Mode == 3)
    {
        Gear_Data = Target_GEPB_N;
    }
    else if(Gear_Mode == 4)
    {
        //ROS_INFO("Gkeep_Time:  [%d]", Gkeep_Time);
        if(Gkeep_Time > 5  && Gear_Flag == 0)
        {
            
            Gkeep_Time--;
            Gear_Data = Target_GEPB_N;
            SendData_ID02[4] = 0x20;                                 //期望纵向减速度，踩一点刹车
            //ROS_INFO("Gear_Data:  [%x]", Gear_Data);
        }
        else if(Gkeep_Time <= 5)
        {
            Gear_Data = Target_GEPB_D;
            SendData_ID02[4] = 0x20;                                 //期望纵向减速度，踩一点刹车
            //ROS_INFO("Gear_Flag:  [%d]", Gear_Flag);
            //ROS_INFO("Gear_Data:  [%x]", Gear_Data);
            Gkeep_Time = 200;
            Gear_Flag = 1;  
        }

         //ROS_INFO("Gear_Data:  [%x]", Gear_Data);
    }  
    return Gear_Data;
}


void sub_send::HandBrake_Select(int HandBrake_Mode, int Gear_Choose)
{
    
    
    if(HandBrake_Mode == 1)
    {
        
        if(Hkeep_Time > 5 && HandBrake_Flag == 0)
        {
            Hkeep_Time--;
            HandBrake_Data = Unlock_Handbrake;
        }
        else if(Hkeep_Time <= 5 )
        {
            Hkeep_Time = 20;
            HandBrake_Data = 0x00;
            HandBrake_Flag = 1;  
        }    
    }
    else if(HandBrake_Mode == 2)
    {
        
        if(Hkeep_Time > 5 && HandBrake_Flag == 0)
        {
            Hkeep_Time--;
            HandBrake_Data = Lock_Handbrake;
            ROS_INFO("Lock_Handbrake:  [%x]", Lock_Handbrake);
            
        }
        else if(Hkeep_Time <= 5)
        {
            
            Hkeep_Time =20;
            HandBrake_Data = 0x00;  
            HandBrake_Flag = 1; 
        } 
    }
    else
    {
        HandBrake_Data = 0x00; 
    }
    SendData_ID02[5] = HandBrake_Data + Gear_Select(Gear_Choose);                          //档位、手刹
    
}


/***************************************************
*函数名：Send_Data_ID01() 
*输入：void 
*功能：发送CAN数据 ID 0x10 
****************************************************/
void sub_send::BcmControl2(unsigned int TurnLight)
{
    unsigned char TurnLight_Data = 0x00;

    switch (TurnLight)   
    {
        case 0:  
                TurnLight_Data = Light_Close;               
                break;
        case 1: 
                TurnLight_Data = Light_Left;
                break;  
        case 2: 
                TurnLight_Data = Light_Right;   
                break; 
        case 3: 
                TurnLight_Data = Light_Warning;   
                break;                                     
        default:  
                break; 
      }    

    SendData_ID01[3] = TurnLight_Data;                                       
     
}


/***************************************************
*函数名：Send_Data_ID02()
*输入：void
*功能：发送CAN数据 ID 0x11
****************************************************/
void sub_send::Send_Data_ID02(unsigned int S_Acc ,unsigned int S_Direct_Flag)
{
    if(S_Direct_Flag == 1)                           //减速度
    {
        Exp_BrakeAcc = S_Acc;
        Exp_Acc = 0;                                 //加速度清零
        ROS_INFO("Exp_BrakeAcc [%d]",Exp_BrakeAcc);
     }
    else if(S_Direct_Flag == 2)                      //加速度
   {
        Exp_Acc = S_Acc; 
        Exp_BrakeAcc = 0;                            //减速度清零 
        ROS_INFO("Exp_Acc [%d]",Exp_Acc);              
    }
   else
   {
        Exp_Acc = 0;
        Exp_BrakeAcc = 0;
    }      
    TargetAcc_Conversion (Exp_Acc);
    Target_Brake_Conversion (Exp_BrakeAcc);
    SendData_ID02[3] = TargetAcc;                            //期望纵向加速度
    SendData_ID02[4] = Target_Brake;                         //期望纵向减速度
         
}
        



/***************************************************
*函数名：Send_Data_ID03()
*输入：void 
*功能：发送CAN数据 ID 0x12
****************************************************/
void sub_send::Send_Data_ID03( int Send_Angle )
{
    Angle_Conversion (Send_Angle);            
    SendData_ID03[1] = TargetAng_H;                               //角度高
    SendData_ID03[2] = TargetAng_L;                               //角度低
}




/***************************************************
*函数名：Forward_Run()
*输入：int F_Acc ,unsigned int F_Gear
*功能：
****************************************************/
void sub_send::Forward_Run(int F_Acc)
{
    //自动驾驶、档位、纵向使能
    unsigned int Run_Acc = 0;
    unsigned int Brake_Acc = 0;
    unsigned int Direct_Flag = 0;                                                    //方向标
    if(F_Acc <= 0)                                                                   //减速度
    {
        Direct_Flag = 1;                                                             //减速度
        Brake_Acc = abs(F_Acc);                                                             
        Send_Data_ID02(Brake_Acc ,Direct_Flag);                                      //自动驾驶、档位、纵向使能       
    }
    else if(F_Acc > 0)                                                               //加速度
    {  
        Direct_Flag = 2;                                                             //加速度
        Run_Acc = F_Acc;                                                             
        Send_Data_ID02(Run_Acc, Direct_Flag);                                        //自动驾驶、档位、纵向使能         
    }

}



/***************************************************
*函数名：Rear_Run()
*输入：void
*功能：
****************************************************/
void sub_send::Rear_Run(int Re_Acc)                      
{
    unsigned int Run_Acc = 0;
    unsigned int Brake_Acc = 0;
    unsigned int Direct_Flag = 0;                                                    //方向标
    if(Re_Acc <= 0)                                                                   //减速度
    {
        Direct_Flag = 1;                                                             //减速度
        Brake_Acc = abs(Re_Acc);                                                             
        Send_Data_ID02(Brake_Acc ,Direct_Flag);      //自动驾驶、档位、纵向使能       
    }
    else if(Re_Acc > 0)                                                               //加速度
    {  
        Direct_Flag = 2;                                                             //加速度
        Run_Acc = Re_Acc;                                                             
        Send_Data_ID02(Re_Acc, Direct_Flag);        //自动驾驶、档位、纵向使能         
    }
}




/***************************************************
*函数名：Steer_Turn()
*输入：void
*功能：
****************************************************/
void sub_send::Steer_Turn(int S_Angle )
{   

    if(S_Angle >= 400)
    {
        S_Angle = 400;
    }
    else if(S_Angle <= -400)
    {
        S_Angle = -400;
    }
    printf("S_Angle %d",S_Angle);
    Send_Data_ID03(S_Angle);
}




/***************************************************
*函数名：InitCAN()
*输入：void
*功能：初始化CAN总线
*只执行一次
****************************************************/
void sub_send::InitCAN()
{
    
    Start_CAN();                                        //打开设备
    VCI_INIT_CONFIG config;                             //初始化CAN的配置
    config.AccCode=0;                                   //验收码
    config.AccMask=0xffffffff;                          //屏蔽码
    config.Filter=1;                                    //接收所有帧   //1                                
    config.Timing0=0x00;                                //波特率500 Kbps
    config.Timing1=0x1C;
    config.Mode=0;                                      //正常模式

    /*CAN信息帧 ID_1*/
    sendbuf[0].ID = 0x10;                               //标准帧0x10
    sendbuf[0].SendType = 0;                            //发送帧类型 正常发送
    sendbuf[0].RemoteFlag = 0;                          //数据帧
    sendbuf[0].ExternFlag = 0;                          //标准帧
    sendbuf[0].DataLen = 8;                             //数据长度

    /*CAN信息帧 ID_2*/
    sendbuf[1].ID = 0x11;                               //标准帧0x10
    sendbuf[1].SendType = 0;                            //发送帧类型 正常发送
    sendbuf[1].RemoteFlag = 0;                          //数据帧
    sendbuf[1].ExternFlag = 0;                          //标准帧
    sendbuf[1].DataLen = 8;                             //数据长度

    /*CAN信息帧 ID_3*/
    sendbuf[2].ID = 0x12;                               //标准帧0x10
    sendbuf[2].SendType = 0;                            //发送帧类型 正常发送
    sendbuf[2].RemoteFlag = 0;                          //数据帧
    sendbuf[2].ExternFlag = 0;                          //标准帧
    sendbuf[2].DataLen = 8;                             //数据长度 



    /*CAN1*/
    if(VCI_InitCAN(VCI_USBCAN2,CAN_NUM_2,0,&config)!=1)
    {
        printf(">>Init CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2,CAN_NUM_2);
    }
    if(VCI_StartCAN(VCI_USBCAN2,CAN_NUM_2,0)!=1)
    {
        printf(">>Start CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2,CAN_NUM_2);
    }



}





/***************************************************
*函数名：Close_CAN()
*输入：void
*功能：关闭CAN总线
****************************************************/
void sub_send::Close_CAN()
{
    VCI_ResetCAN(VCI_USBCAN2,CAN_NUM_2,0);
    VCI_CloseDevice(VCI_USBCAN2,CAN_NUM_2);
    //printf(">>Close CAN1 Device\n");
}




/***************************************************
*函数名：Start_CAN()
*输入：void
*功能：打开CAN总线
****************************************************/
void sub_send::Start_CAN()
{
    printf(">>Start CAN1 Device\n");
    if(VCI_OpenDevice(VCI_USBCAN2,CAN_NUM_2,0)==1)              //打开设备
    {
        printf(">>open deivce success!\n");             //打开设备成功
    }     
    else
    {   
        printf(">>open deivce error!\n");
        exit(1);
    }
}



/***************************************************
*函数名：AEB_TarAcc_Conversion()
*输入：V_AEB_Tar  (-16 0)
*输出：void（高8位 低8位）
*功能：自动制动H、L转化
****************************************************/
void sub_send::AEB_TarAcc_Conversion(unsigned int C_AEB_Tar)
{
    if(C_AEB_Tar>-5 && C_AEB_Tar<0)
	{
        AEB_TarAcce_H = (unsigned char) (((C_AEB_Tar + 16)*2050) / 256);
		AEB_TarAcce_L = (unsigned char) (((C_AEB_Tar + 16)*2050) % 256);
	  }
    else
    {
        //默认值的选取？？？
        AEB_TarAcce_H = 0X00;
        AEB_TarAcce_L = 0X00;
        //printf(">> AEB_TarAcc Conversion Over \n");  
      }

}



/***************************************************
*函数名：Angle_Conversion()  ##########待改进#########
*输入：C_Angle  (-80 80)     #####加反馈角度、死区######
*输入：左正右负
*输出：高8位 低8位
*功能：角度转化 
****************************************************/
void sub_send::Angle_Conversion(int C_Angle)
{
    unsigned int Delta_Angle = 0;
    Delta_Angle = abs(C_Angle - R_Angle);
    if(Delta_Angle > 80)
    {
        if(C_Angle >= 0)
        {
            C_Angle = R_Angle + 80;
        }
        else 
        {
            C_Angle = R_Angle - 80;
        }
        TargetAng_H = (unsigned char) (((C_Angle + 870)*10) / 256);
        TargetAng_L = (unsigned char) (((C_Angle + 870)*10) % 256);
    }
    else
    {
        if(C_Angle >= -400 && C_Angle <= 400)
	    {
            TargetAng_H = (unsigned char) (((C_Angle + 870)*10) / 256);
            TargetAng_L = (unsigned char) (((C_Angle + 870)*10) % 256);
	    }
        else
        {
            if(C_Angle > 0)
            {
                C_Angle = 400;
            }
            else
            {
                C_Angle = -400;
            }
            TargetAng_H = (unsigned char) (((C_Angle + 870)*10) / 256);
            TargetAng_L = (unsigned char) (((C_Angle + 870)*10) % 256); 
        }
    }

}



/***************************************************
*函数名：Velocity_Conversion()
*输入：C_Velocity  (100度/秒 540度/秒)
*输出：void
*功能：角速度转化
****************************************************/
void sub_send::Velocity_Conversion(unsigned int C_Velocity)
{
    if(C_Velocity>100 && C_Velocity<540)                   //测试
	  {
        TargetAng_Speed = (unsigned char) (C_Velocity/10);
	  }
    else
     {
        TargetAng_Speed = 0X00;                            //设置默认合适
        //printf(">> Velocity Conversion Over \n");  
     }
}




/***************************************************
*函数名：TargetAcc_Conversion()
*输入：C_Acc  [0 m/s2 5 m/s2] -> (0 100)  (0-2)
*输出：void
*功能：前进加速度转化
****************************************************/
void sub_send::TargetAcc_Conversion(unsigned int C_Acc)
{
    if(C_Acc>0 && C_Acc<40)
	  {
        TargetAcc = (unsigned char) (C_Acc);
	  }
    else
     {
        TargetAcc = 0X00;
        //printf(">> TargetAcc Conversion Over \n");  
     }
}




/***************************************************
*函数名：Target_Brake_Conversion()
*输入：C_BrakeAcc [0 m/s2 5 m/s2] -> (0 100)
*输出：void
*功能：前进减速度转化
****************************************************/
void sub_send::Target_Brake_Conversion(unsigned int C_BrakeAcc)
{
    if(C_BrakeAcc>0 && C_BrakeAcc<100)
	  {
        Target_Brake = (unsigned char) (C_BrakeAcc);
	  }
    else
     {
        Target_Brake = 0X00;
        //printf(">> Target_Brake Conversion Over \n");  
     }
}




/***************************************************
*函数名：sub_send::Verify_Conversion(unsigned char Conver_Data[8])
*输入：Conver_Data[]  要异或校验的数据
*返回值：异或校验位
*功能：异或校验
****************************************************/
unsigned char sub_send::Verify_Conversion(unsigned char Conver_Data[])
{
    int count=0;
    unsigned char Verify_Value = 0x00;
    Verify_Value=Conver_Data[0]^Conver_Data[1]^Conver_Data[2]^Conver_Data[3]^Conver_Data[4]^Conver_Data[5]^Conver_Data[6];
    return Verify_Value;
}



/***************************************************
*函数名：main(int argc, char **argv)
*输入：void
*返回值：void
*功能：打开析构函数
****************************************************/
int main(int argc, char **argv)
{   
    ros::init(argc, argv, "Motor_CAN");
    ros::NodeHandle n;
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
            CAN_NUM_2 = i;
            ROS_INFO("CAN_NUM_2==%d",CAN_NUM_2);
        }
    }
    sub_send core(n);     
    ros::Rate loop_rate(100);
    while (ros::ok()) 
    {
      ROS_INFO("CAN_NUM_2==%d",CAN_NUM_2);
    }  
    return 0;

}










