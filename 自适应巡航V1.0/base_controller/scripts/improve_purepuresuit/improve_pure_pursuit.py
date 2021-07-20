#!/usr/bin/python2
# -*- coding: UTF-8 -*-
# coding: utf-8
#!/usr/bin/env python


'''
订阅：局部路径规划  List[] -> path.x path.y
采用MIT算法对局部路径进行跟踪，期望速度<=10km/h

'''


import rospy
import numpy as np
import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
import tf
import matplotlib.pyplot as plt
from local_planner.msg import localPath
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from CAN_driver.msg import Motor_Feedback
from GNSS_driver.msg import GNSS_CAN
import sys



sys.path.append("../../PathPlanning/CubicSpline/")

try:
    import cubic_spline_planner
except:
    raise

THRESH_DIST=0.010
#全局变量
LocalPath_x = 0.0
LocalPath_y = 0.0


sx = 0.0
sy = 0.0
ryaw = 0.0
angle = 0.0
rv = 0.0
zero_cord_x = 0.0
zero_cord_y = 0.0
object_bool = 0
# 参数
k = -0.2  # 前视增益
Lfc =3.0  # [m] 前视距离
Lf = 0.0  # 前视距离
old_Lf = 0.0
bet = 0.1                             #缓冲距离
Lf_optimal = 0.0 #最优前视距离
Kp = 1.5 # 速度比例
dt = 0.1  # [s]
WB = 2.49  # [m] 轴距
lfw = 1.25
K_ld = 2.5 #2.5
const = 0
max_steer = np.radians(30.0)  # [rad] 最大转弯角速
show_animation = True
Lateral_Error = []
GNSS_flag = 0
localPath_flag = 0
target_speed =0.0
recive_global_point_final = 0



class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.x - point_x
        dy = self.y - point_y
        return math.hypot(dx, dy)


class States:

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)



def update(state):
    #global ryaw, sx, sy
 
    try:
        state.x = sx 
        state.yaw = ryaw
        state.y = sy 
    except:
        print("state updata error")
        
    gps_to_map(state)

    #state.yaw = np.radians(state.yaw)
    '''
    while state.yaw > np.pi:
        state.yaw -= 2.0 * np.pi

    while state.yaw < -np.pi:
        state.yaw += 2.0 * np.pi
    '''
    #print("state.yaw:", state.yaw)
    #print("state.yaw:", math.sin(state.yaw))
    # state.v=float(rv)
    # state.v=state.v/3.6   # speed is m/s

    '''
    state.x = state.x + state.v * math.cos(state.yaw) * dt
    print("state.x:",state.x)
    print("state.x:",state.x)
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    '''
    state.v = rv  # state.v + a * dt                                   #更新此时的速度
    state.rear_x = state.x  # - ((L / 2) * math.cos(state.yaw))     #计算当前的后轮位置X
    state.rear_y = state.y  # - ((L / 2) * math.sin(state.yaw))     #计算当前的后轮位置Y

    return state









def proportional_control(target, current):
    #print("*"*50)
    #print("target=",current)
    #print("target - current",target - current)
    a = Kp * (target - current)

    return a


class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state, Lf):

        # 加速搜索最近点，只在第一次执行。
        if self.old_nearest_point_index is None:
            # 搜索最近点的索引号
            dx = [state.x - icx for icx in self.cx]
            dy = [state.y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
        
            #print("state.x",state.x)
            #print("state.y",state.y)
            #print("d",d)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
            #print("*"*50)
            #print("old_nearest_point_index",ind)
        else:
            #car在轨迹上
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                
                if (ind + 1) < len(self.cx) :
                    ind = ind + 1 
                else :
                    ind = ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind
            #print("*"*50)
            #print("ind",ind)

        #Lf = Lfc + 0.1  # 更新前视距离
        # 搜索前视距离对应的目标点的索引号
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # 不会超出目标轨迹点
            ind += 1

        return ind

#主函数重复执行
def pure_pursuit_steer_control(state, trajectory, pind, Lf):
    ind= trajectory.search_target_index(state,Lf)

    if pind >= ind:
        ind = pind
    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1
    #alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw
    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw
    delta = (math.atan2(WB * math.sin(alpha) / (Lf/K_ld), 1.0))/math.pi*180
    #delta = -(math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0))/math.pi*180
    #delta = math.atan2(WB * math.sin(alpha) /(Lf/2 - 0.2*math.cos(alpha)) , 1.0)
    #print("delta=",delta)
    return delta, ind


def plot_arrow(x, y, yaw, length=1.0, width=1.50, fc="r", ec="k"):
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for ix, iy, iyaw in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)






def fit_expect_path(state, trajectory, Lf, pind):
    delta, ind = pure_pursuit_steer_control(state, trajectory, pind, Lf)
    radius = WB / math.tan(math.fabs(np.radians(delta)))
    start_x = state.rear_x
    start_y = state.rear_y
    end_x = trajectory.cx[ind]
    end_y = trajectory.cy[ind]
    start_theta = state.yaw - math.pi/2
    center_x = start_x - radius*math.cos(start_theta)
    center_y = start_y - radius*math.sin(start_theta)
    if (end_x - center_x)/radius >1 or (end_x - center_x)/radius < -1 :
        fx = [start_x, end_x]
        fy = [start_y, end_y]
        gx, gy, gyaw, gk, s, gv = cubic_spline_planner.calc_spline_course(
            fx, fy, ds=2.5)
    else:
        end_theta = math.acos((end_x - center_x) / radius)
        middle_x = center_x + radius*math.cos((end_theta + start_theta)/2)
        middle_y = center_y + radius*math.sin((end_theta + start_theta)/2)
        fx = [start_x, middle_x, end_x]
        fy = [start_y, middle_y, end_y]
        gx, gy, gyaw, gk, s, gv = cubic_spline_planner.calc_spline_course(
            fx, fy, ds=100000) #ds=100000 mm
    return gx, gy, gyaw, gk, s, gv




def fit_geometric_curve(state, trajectory, Lf):
    """
    end_point 等价于 Goldposition
    """

    ind = trajectory.search_target_index(state, Lf)
    start_x = state.rear_x
    start_y = state.rear_y
    end_x = trajectory.cx[ind]
    end_y = trajectory.cy[ind]
    fx = [start_x, end_x]
    fy = [start_y, end_y]
    gx, gy, gyaw, gk, s, gv=cubic_spline_planner.calc_spline_course(
        fx, fy, ds=0.1)

    return gx, gy, gyaw, gk, s, gv, ind
    
    
    
    
    
    
def get_lateral_dist(tx,ty,curr_posx,curr_posy):
   dist=[]
   for x in range(0,len(tx)-1):
       dist.append(np.hypot((float(curr_posx)-tx[x]),(float(curr_posy)-ty[x])))
   lat_dist=min(dist)
   st=dist.index(min(dist))
   theta1=math.atan2((ty[st]-ty[st-1]),(tx[st]-tx[st-1]))
   theta2=math.atan2((curr_posy-ty[st-1]),(curr_posx-tx[st-1]))
   if lat_dist<THRESH_DIST:
       lat_dist=0
       curr_posx=tx[st]
       curr_posy=ty[st]
   if theta2<theta1:
       lat_dist=-lat_dist
   # print(lat_dist)
   return st, lat_dist, curr_posx, curr_posy   
    
    
    
    
    
    
    
    
    
    
    
    
    
def gps_to_map(state):
    
    state.x = sx 
    state.y = sy - 1.0
    state.yaw =90 - state.yaw
    #print("state.yaw",state.yaw)
    #print("zero_cord_x:", zero_cord_x)
    #print("zero_cord_y:", zero_cord_y)
    #print("state.x:", state.x)
    #print("state.y:", state.y)
    #print("state.yaw:", state.yaw)
    state.yaw = (state.yaw * np.pi) / 180

#######################################
#函数：RVcallback(data)
#功能：订阅当前车速
#data的数据类型与Subscriber接收的Topic对应的消息类型一致
#######################################
def RVcallback(Motor_Feedback):
    global rv
    rv = Motor_Feedback.Base_Vehspd
    #print("*"*50)
    #print("rv:",rv)
    #rospy.loginfo('I heard: %s', data.data)
    
    

    

#######################################
# 函数：callback(msg)
# 功能：订阅组合惯导纬度、经度、航向角
# data的数据类型与Subscriber接收的Topic对应的消息类型一致
#######################################
def nav_callback(gnss_msg):
    
        
    zero_cord_x = 13080701.0116 
    zero_cord_y = 4714996.4051
    global ryaw, sx, sy ,GNSS_flag
    sx = gnss_msg.latitude - zero_cord_x
    sy = gnss_msg.longitude - zero_cord_y
    ryaw = gnss_msg.course_angle
    GNSS_flag = 1

    

 
    


    #sx.strip()
    

    #state = update(state)
    # return yaw,sx,sy


def avoid_object_callback(obj_data):
    global object_bool
    object_bool = int (obj_data.data)
    #print("*"*50)
    #print("object_bool",object_bool)





def localPath_callback(pathMsg):
    global LocalPath_x, LocalPath_y, localPath_flag
    
    LocalPath_x = pathMsg.path_x
    LocalPath_y = pathMsg.path_y
    localPath_flag = 1
   
    
    #print("position_x",LocalPath_x)
    #print("position_y",LocalPath_y)
    #print("*"*50)






def main1():
    
    
    blank = []                                                                      #buffer
    white = []                                                                      #buffer
    yellow = []                                                                     #buffer
    GPS_x = []                                                                         #所采集预描点的x
    GPS_y = []                                                                         #所采集预描点的x
 #读取预描点
    nums, ber = np.loadtxt("/home/robot/Robot/Smart_robot_ws/src/GNSS_driver/save_point_data/rightdoubleliner.txt", dtype=str, delimiter=',', unpack=True)
    for i in range(len(nums)):
        if not nums[i] in blank:                                                #去除重复点
                #blank.append(nums[i])
            yellow.append(float(nums[i]))
            white.append(float(ber[i]))
    bx = yellow[0]                                                              #起始点坐标
    by = white[0]
    for i in range(len(yellow)):
        dx = yellow[i] - bx
        dy = white[i] - by
        dis = math.sqrt(dx ** 2 + dy ** 2) 
        if dis > bet:                                                           #选取大于设定的距离的点
            GPS_x.append(yellow[i])                                                #使cx，cy中点均满足要求
            GPS_y.append(white[i])
            bx = yellow[i]
            by = white[i]   
    GPS_x = np.array(GPS_x)                                                           #将列表转换成数组
    GPS_y = np.array(GPS_y)
    #print("cx:",cx)
    #print("cy:",cy)
    
    global zero_cord_x,zero_cord_y
    zero_cord_x = GPS_x[0]
    zero_cord_y = GPS_y[0]
    
    
    GPS_x = GPS_x - zero_cord_x
    GPS_y = GPS_y - zero_cord_y
    plt.plot(GPS_x,GPS_y, "-r", label="GPS point ")
    plt.plot()
    plt.show()
    
    
    
    
    
    
    
    rospy.init_node('pure_pursuit', anonymous = False)                              #初始化ROS节点 pure_pursuit
    #向底层发布 方向盘角度、加速度（P挡） 控制指令
    simple_publisher = rospy.Publisher('delta', Float32, queue_size = 10)           #定义Publisher对象
    second_publisher = rospy.Publisher('c_a', Float32, queue_size = 10)
    rate = rospy.Rate(100)                                                           #设置Topic发布的频率（Hz）


#初始化
    #target course
    # cx = np.arange(0, 50, 0.5)
    # cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]
    # #三次多项式规划的目标跟踪曲线
    #
    # target_speed = 10.0 / 3.6  # [m/s]
    # ax = [0.0, 0.0, 10.0, 13.0, 25.0, 20.0, 0.0, -10.0, 30.0, 40.0]
    # ay = [-10.0, 5.0, 50.0, 50.0, 20.0, 0.0, -5.0, 50.0, -10.0, 20.0]
    ax = GPS_x
    ay = GPS_y
    cx, cy, cyaw, ck, s, cv = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=1.0)
    plt.plot(cx,cy, "-k", label="GPS cubic_spline_planner point ")
    plt.plot()
    plt.show()
    T = 10000000000.0  # 最大仿真时间
    

    # 初始状态
    state = State(x=ax[0], y=ay[0], yaw=0, v=0.0)
    #state = State(x=ax[0], y=ay[0], yaw=150, v=0.0)
    Lf = 2.0
    old_target_ind = 0
    Lf_optimal = 2.0
    Lf_optimal_list = []
    prob = []
    NUM = 60
    Lf_index = []
    weight_delta =0.1#0.9990.99
    weight_Lateral = 0.8999#0.0010.01
    weight_curvature = 0.0001

    lastIndex = len(cx) - 1
    time = 0.0
    states = States()
    states.append(time, state)
    target_course = TargetCourse(cx, cy)
    target_ind= target_course.search_target_index(state, Lf_optimal)


    while not rospy.is_shutdown():
    #订阅GPS数据
        sub = rospy.Subscriber('gnss_message', GNSS_CAN, nav_callback,queue_size = 10)                      #订阅GPS数据
        rospy.Subscriber("Motor_Feedback_mssage", Motor_Feedback,RVcallback,queue_size = 10)
        
    #主循环
        while lastIndex > target_ind:
            #target_speed = 5.0#3.6*(2.0* cv[target_ind+0] - 20.0*abs(ck[target_ind]))
            #state = update(state)
            
            
            st, lateral_error, curr_posx, curr_posy = get_lateral_dist(cx, cy, state.x, state.y)
            #print("*" * 50)
            #print("lateral_error",lateral_error)
            
            
            while target_ind - old_target_ind != 0 :
                if Lf > 8.0 :
                    Lf =  2.0
                    Lf_optimal = 3.0
                    prob *= 0
                    Lf_index *= 0
                    print("calc optimal Lf over")
                elif Lf <= 8.0 :
                    for num in range(NUM):
                        gx, gy, gyaw, gk, gs, gv, fit_ind = fit_geometric_curve(state, target_course, Lf)
                        #gx, gy, gyaw, gk, gs, gv = fit_expect_path(state, target_course, Lf, target_ind)
                        fit_data_len = len(gx)-1
                        dx = [gx[int (fit_data_len/2)] - icx for icx in cx]
                        dy = [gy[int (fit_data_len/2)] - icy for icy in cy]
                        # dx = [gx[int (fit_data_len/2)] - icx for icx in cx]
                        # dy = [gy[int (fit_data_len/2)] - icy for icy in cy]
                        Lateral_error = np.hypot(dx, dy)
                        Lateral_error_ind = np.argmin(Lateral_error)
                        
                        cost = weight_delta * (gyaw[fit_data_len] - cyaw[fit_ind]) ** 2
                        cost += weight_Lateral * (Lateral_error[Lateral_error_ind]) ** 2
                        cost -=weight_curvature * (ck[fit_ind])**2                        
                        
                        #cost = weight_delta * (np.mean(gyaw) - cyaw[target_ind]) ** 2
                        #print("*"*100)
                        #print("np.mean(gyaw)","cyaw[target_ind]",np.mean(gyaw)*57.3,cyaw[target_ind]*57.3)
                        #cost = weight_delta * (gyaw[fit_data_len-1] - cyaw[target_ind]) ** 2
                        #cost += weight_Lateral * (Lateral_error[Lateral_error_ind]) ** 2
                        #cost -= weight_curvature * (ck[fit_ind])
                        #print("-"*100)
                        #print("Lateral_error[Lateral_error_ind]",Lateral_error[Lateral_error_ind])
                        #cost += -100/target_speed
                        prob.append(cost)
                        Lf_index.append(Lf)
                        if Lf > 8 :
                            break
                        Lf += 0.1
                    promin_index = np.argmin(prob)
                    Lf_optimal = Lf_index[promin_index]
                    break
            old_target_ind = target_ind

            #print("Lf_optimal=", Lf_optimal)
            
            if object_bool == 0:
                if Lf_optimal<3.5 :
                    target_speed = 4.0
                else :
                    target_speed = 5.0
            elif object_bool == 1:
                target_speed = -2.0
                
            ai = proportional_control(target_speed, state.v)
            #print("*-"*50)
            #print("acc",ai)
            di, target_ind = pure_pursuit_steer_control(
                state, target_course, target_ind, Lf_optimal)

            
            time += dt
            state = update(state)
            #print("*"*50)
            #print(state.x,state.y,state.yaw)
            states.append(time, state)
            Lateral_Error.append(lateral_error)
            Lf_optimal_list.append(Lf_optimal)

            simple_publisher.publish(di)
            second_publisher.publish(ai)
            # 发布Topic
            rate.sleep()
            #print("-"*50)
            #print("ai:", ai)
            #print("di",di)
            
            if show_animation:  # pragma: no cover
                plt.cla()
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                plot_arrow(state.x, state.y, state.yaw) #np.radians(state.yaw)
                plt.plot(cx, cy, "-r", label = "course")
                #plt.plot(gx, gy, "-k", label = "fit_trajectory")
                plt.plot(states.x, states.y, "-b", label="trajectory")
                plt.plot(cx[target_ind], cy[target_ind], "*-k", label="target")
                plt.axis("equal")
                plt.grid(True)
                #plt.title("Lf_optimal_list:" + str(Lf_optimal_list)[1:])
                plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4] )
                plt.pause(0.001)

        # Test
        assert lastIndex >= target_ind, "Cannot goal"
        #np.savetxt('/home/robot/Robot/Smart_robot_ws/src/Data/improvedata_cx.txt',cx,fmt ='%f',header='cx')
        #np.savetxt('/home/robot/Robot/Smart_robot_ws/src/Data/improvedata_cy.txt',cy,fmt ='%f',header='cy')
        #np.savetxt('/home/robot/Robot/Smart_robot_ws/src/Data/improvedata_statex.txt',states.x,fmt ='%f',header='states.x')
        #np.savetxt('/home/robot/Robot/Smart_robot_ws/src/Data/improvedata_statey.txt',states.y,fmt ='%f',header='states.y')
        #np.savetxt('/home/robot/Robot/Smart_robot_ws/src/Data/improvedata_states_yaw.txt',states.yaw,fmt ='%f',header='states.yaw')
        #np.savetxt('/home/robot/Robot/Smart_robot_ws/src/Data/improvedata_states_v.txt',states.v,fmt ='%f',header='states.v')
        #np.savetxt('/home/robot/Robot/Smart_robot_ws/src/Data/improvedata_lateral_error.txt',Lateral_Error,fmt ='%f',header='Lateral_Error')
        if show_animation:  # pragma: no cover
            plt.cla()
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(states.x, states.y, "-b", label="trajectory")
            plt.legend()
            plt.xlabel("x[m]")
            plt.ylabel("y[m]")
            plt.axis("equal")
            plt.grid(True)

            plt.subplots(1)
            plt.plot(states.t, [iv * 3.6 for iv in states.v], "-r")
            plt.xlabel("Time[s]")
            plt.ylabel("Speed[km/h]")
            plt.grid(True)

            plt.subplots(1)
            plt.plot(states.t, states.yaw, "-r")
            plt.xlabel("Time[s]")
            plt.ylabel("yaw[rad]")
            plt.grid(True)

            # plt.subplots(1)
            # plt.plot(gx, gy, "-r")
            # plt.xlabel("x[m]")
            # plt.ylabel("y[m]")
            # plt.grid(True)

            plt.subplots(1)
            plt.plot(Lf_optimal_list, "-k")
            plt.xlabel("t[s]")
            plt.ylabel("Lf[m]")
            plt.grid(True)
            
            plt.subplots(1)
            plt.plot(Lateral_Error, ".k")
            plt.xlabel("t[s]")
            plt.ylabel("lateral_error[m]")
            plt.grid(True)
            plt.show()            
            
            
            
            
            


            
            
            



def main_MIT():
    vel_msg = Twist()
    
    blank = []                                                                      #buffer
    white = []                                                                      #buffer
    yellow = []                                                                     #buffer
    GPS_x = []                                                                         #所采集预描点的x
    GPS_y = []                                                                         #所采集预描点的x
 #读取预描点
    nums, ber = np.loadtxt("/home/robot/Robot/Smart_robot_ws/src/GNSS_driver/save_point_data/show_1.txt", dtype=str, delimiter=',', unpack=True)
    for i in range(len(nums)):
        if not nums[i] in blank:                                                #去除重复点
                #blank.append(nums[i])
            yellow.append(float(nums[i]))
            white.append(float(ber[i]))
    bx = yellow[0]                                                              #起始点坐标
    by = white[0]
    for i in range(len(yellow)):
        dx = yellow[i] - bx
        dy = white[i] - by
        dis = math.sqrt(dx ** 2 + dy ** 2) 
        if dis > bet:                                                           #选取大于设定的距离的点
            GPS_x.append(yellow[i])                                                #使cx，cy中点均满足要求
            GPS_y.append(white[i])
            bx = yellow[i]
            by = white[i]   
    GPS_x = np.array(GPS_x)                                                           #将列表转换成数组
    GPS_y = np.array(GPS_y)
    #print("cx:",cx)
    #print("cy:",cy)
    global zero_cord_x,zero_cord_y
    zero_cord_x = GPS_x[0]
    zero_cord_y = GPS_y[0]
    GPS_x = GPS_x - zero_cord_x
    GPS_y = GPS_y - zero_cord_y
    plt.plot(GPS_x,GPS_y, "-r", label="GPS point ")
    plt.plot()
    plt.show()
    
    
    
    
    
    
                                                              #设置Topic发布的频率（Hz）


#初始化
    #target course
    # cx = np.arange(0, 50, 0.5)
    # cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]
    # #三次多项式规划的目标跟踪曲线
    #
    # target_speed = 10.0 / 3.6  # [m/s]
    # ax = [0.0, 0.0, 10.0, 13.0, 25.0, 20.0, 0.0, -10.0, 30.0, 40.0]
    # ay = [-10.0, 5.0, 50.0, 50.0, 20.0, 0.0, -5.0, 50.0, -10.0, 20.0]
    ax = GPS_x
    ay = GPS_y
    cx, cy, cyaw, ck, s, cv = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=0.1) #0.1
    plt.plot(cx,cy, "-k", label="GPS cubic_spline_planner point ")
    plt.plot()
    plt.show()
    
    #goal_x = [cx[500],cx[1000], cx[1500], cx[2000], cx[2500], cx[3000], cx[3500], cx[4000], cx[4500]]
    #goal_y = [cy[500],cy[1000], cy[1500], cy[2000], cy[2500], cy[3000], cy[3500], cy[4000], cy[4500]]
    T = 10000000000.0  # 最大仿真时间
    

    # 初始状态
    
    #state = State(x=ax[0], y=ay[0], yaw=150, v=0.0)
    Lf = 2.0
    old_target_ind = 0
    Lf_optimal = 2.0
    Lf_optimal_list = []
    prob = []
    NUM = 40
    Lf_index = []
    weight_delta =0.9#0.999
    weight_Lateral = 0.1#0.001
    weight_curvature = 0.0
    count = 0
    lastIndex = len(cx) - 1
    time = 0.0
     
    states = States()
    
    target_course = TargetCourse(cx, cy)
    
    

    while not rospy.is_shutdown():
        rospy.init_node('pure_pursuit', anonymous = False)                              #初始化ROS节点 pure_pursuit
        simple_publisher = rospy.Publisher('delta', Float32, queue_size = 10)           #定义Publisher对象
        second_publisher = rospy.Publisher('c_a', Float32, queue_size = 10)
        carpose_publisher = rospy.Publisher('car_pose', Twist, queue_size = 20)
        rospy.Subscriber('break_action', String, avoid_object_callback, queue_size=50)
        sub = rospy.Subscriber('gnss_message', GNSS_CAN, nav_callback,queue_size = 10)                      #订阅GPS数据
        rospy.Subscriber("Motor_Feedback_mssage", Motor_Feedback,RVcallback,queue_size = 10)
        rate = rospy.Rate(100) 
        
        if GNSS_flag == 1:
        
        
            state = State(x=sx, y=sy, yaw=ryaw, v=0.0) 
            
            states.append(time, state)
            target_ind= target_course.search_target_index(state, Lf_optimal)
            
            
        #主循环
            while T >= time and lastIndex > target_ind:
                st, lateral_error, curr_posx, curr_posy = get_lateral_dist(cx, cy, state.x, state.y)
                global target_speed
                if object_bool == 0: 
                    print("lateral_error",lateral_error)
                    print("rv",rv)
                    if rv<=2.0:
                        target_speed = 5.0
                        Lf_optimal = 8.0
                    elif rv <= 5.0 and abs(lateral_error) >= 0.8:
                        target_speed = 5.0
                        Lf_optimal = 8.0
                    elif rv > 4.6 and abs(lateral_error) < 0.8 :    
                    
                        if st <= 2200 :
                            target_speed = 15.0
                        elif 2200<st<2900:
                            target_speed = 8.0
                        elif 2900<=st <= 3900 :
                            target_speed = 20.0
                        elif 3900<st<4700:
                            target_speed = 8.0 
                        elif 4700<=st<=5800:
                            target_speed = 18.0
                        elif 5800<st<6500:
                            target_speed = 6.0
                        elif 6500<=st<=7500:
                            target_speed = 18.0
                        elif 7500<st:
                            target_speed = 8.0
                            
                            
                        if (rv)/3.6 < 1.34 :
                            Lf_optimal = 3
                        elif (rv)/3.6 >= 1.34 and (rv)/3.6 <= 5.36:
                            Lf_optimal =2.24*(rv/3.6)
                        elif (rv)/3.6 > 5.36:
                            Lf_optimal = 12 
                elif object_bool == 1:
                    target_speed = -0.4            
                
                
                print("*" * 50)
                print("target_speed",target_speed)    
                 
                #print("Lf_optimal=", Lf_optimal)
    
                acc_pub = proportional_control(target_speed, state.v)
                #print("*-"*50)
                #print("acc",ai)
                state = update(state)
                steer_pub, target_ind = pure_pursuit_steer_control(
                    state, target_course, target_ind, Lf_optimal)
                
                
                
                
                
                
                time += dt
                states.append(time, state)
                Lf_optimal_list.append(Lf_optimal)
                Lateral_Error.append(lateral_error)
                
                simple_publisher.publish(steer_pub)
                second_publisher.publish(acc_pub)
                
                
                vel_msg.linear.x = sx
                vel_msg.linear.y = sy
                vel_msg.linear.z = ryaw

                carpose_publisher.publish(vel_msg)
                # 发布Topic
                rate.sleep()
                #print("-"*50)
                #print("ai:", ai)
                #print("di",di)
    
                if show_animation:  # pragma: no cover
                    plt.cla()
                    # for stopping simulation with the esc key.
                    plt.gcf().canvas.mpl_connect(
                        'key_release_event',
                        lambda event: [exit(0) if event.key == 'escape' else None])
                    plot_arrow(state.x, state.y, state.yaw) #np.radians(state.yaw)
                    
                    plt.plot(cx, cy, "-r", label = "course")
                    
                    #plt.plot(gx, gy, "-k", label = "fit_trajectory")
                    plt.plot(states.x, states.y, "-b", label="trajectory")
                    plt.plot(cx[target_ind], cy[target_ind], "*-k", label="target")
                    plt.axis("equal")
                    plt.grid(True)
                    #plt.title("Lf_optimal_list:" + str(Lf_optimal_list)[1:])
                    plt.title("Speed[km/h]:" + str(state.v)[:4] )
                    plt.pause(0.001)
    
            # Test
            assert lastIndex >= target_ind, "Cannot goal"
            
            #np.savetxt('/home/robot/Robot/Smart_robot_ws/src/Data/mitdata_cx.txt',cx,fmt ='%f',header='cx')
            #np.savetxt('/home/robot/Robot/Smart_robot_ws/src/Data/mitdata_cy.txt',cy,fmt ='%f',header='cy')
            #np.savetxt('/home/robot/Robot/Smart_robot_ws/src/Data/mitdata_states_x.txt',states.x,fmt ='%f',header='states.x')
            #np.savetxt('/home/robot/Robot/Smart_robot_ws/src/Data/mitdata_states_y.txt',states.y,fmt ='%f',header='states.y')
            #np.savetxt('/home/robot/Robot/Smart_robot_ws/src/Data/mitdata_states_yaw.txt',states.yaw,fmt ='%f',header='states.yaw')
            #np.savetxt('/home/robot/Robot/Smart_robot_ws/src/Data/mitdata_states_v.txt',states.v,fmt ='%f',header='states.v')
            #np.savetxt('/home/robot/Robot/Smart_robot_ws/src/Data/mitdata_lateral_error.txt',Lateral_Error,fmt ='%f',header='Lateral_Error')
            if show_animation:  # pragma: no cover
                plt.cla()
                plt.plot(cx, cy, "-r", label="course")
                plt.plot(states.x, states.y, "-b", label="trajectory")
                plt.legend()
                plt.xlabel("x[m]")
                plt.ylabel("y[m]")
                plt.axis("equal")
                plt.grid(True)
    
                plt.subplots(1)
                plt.plot(states.t, [iv * 3.6 for iv in states.v], "-r")
                plt.xlabel("Time[s]")
                plt.ylabel("Speed[km/h]")
                plt.grid(True)
    
                plt.subplots(1)
                plt.plot(states.t, states.yaw, "-r")
                plt.xlabel("Time[s]")
                plt.ylabel("yaw[rad]")
                plt.grid(True)
    
                # plt.subplots(1)
                # plt.plot(gx, gy, "-r")
                # plt.xlabel("x[m]")
                # plt.ylabel("y[m]")
                # plt.grid(True)
    
                plt.subplots(1)
                plt.plot(Lf_optimal_list, "-k")
                plt.xlabel("t[s]")
                plt.ylabel("Lf[m]")
                plt.grid(True)
                
                plt.subplots(1)
                plt.plot(Lateral_Error, ".k")
                plt.xlabel("t[s]")
                plt.ylabel("lateral_error[m]")
                plt.grid(True)
                plt.show()
        else :
            print("no recieve data")
            
            
            
def load_globle_path():
    #定义局部变量 对加载的航路点进行数据处理
    blank = []                                                                      #buffer
    white = []                                                                      #buffer
    yellow = []                                                                     #buffer
    GPS_x = []                                                                         #所采集预描点的x
    GPS_y = []                                                                         #所采集预描点的x



##################################
#加载全局路径序列点迹
##################################

    nums, ber = np.loadtxt("/home/robot/Robot/Smart_robot_ws/src/GNSS_driver/save_point_data/rightdoubleliner.txt", dtype=str, delimiter=',', unpack=True)
    for i in range(len(nums)):
        if not nums[i] in blank:                                                #去除重复点
                #blank.append(nums[i])
            yellow.append(float(nums[i]))
            white.append(float(ber[i]))
    bx = yellow[0]                                                              #起始点坐标
    by = white[0]
    for i in range(len(yellow)):
        dx = yellow[i] - bx
        dy = white[i] - by
        dis = math.sqrt(dx ** 2 + dy ** 2) 
        if dis > bet:                                                           #选取大于设定的距离的点
            GPS_x.append(yellow[i])                                                #使cx，cy中点均满足要求
            GPS_y.append(white[i])
            bx = yellow[i]
            by = white[i]   
    GPS_x = np.array(GPS_x)                                                           #将列表转换成数组
    GPS_y = np.array(GPS_y)
    #print("cx:",cx)
    #print("cy:",cy)
    global zero_cord_x,zero_cord_y
    zero_cord_x = GPS_x[0]
    zero_cord_y = GPS_y[0]
    GPS_x = GPS_x - zero_cord_x
    GPS_y = GPS_y - zero_cord_y
    plt.plot(GPS_x,GPS_y, "-r", label="GPS point ")
    plt.plot()
    plt.show()
    recive_global_point_final = 1
    
    
    
    
    
    
                                                              #设置Topic发布的频率（Hz）

    

            
def main_local_path():
    
    #load_globle_path()
    vel_msg = Twist()
    
    

    # 初始状态
    
    #state = State(x=ax[0], y=ay[0], yaw=150, v=0.0)
    Lf = 2.0
    old_target_ind = 0
    Lf_optimal = 4.5
    Lf_optimal_list = []
    prob = []
    NUM = 40
    Lf_index = []

    count = 0
    
    time = 0.0
    states = States()
    
    
    

    while not rospy.is_shutdown():
        rospy.init_node('pure_pursuit', anonymous = False)                              #初始化ROS节点 pure_pursuit
        simple_publisher = rospy.Publisher('delta', Float32, queue_size = 10)           #定义Publisher对象
        second_publisher = rospy.Publisher('c_a', Float32, queue_size = 10)
        carpose_publisher = rospy.Publisher('car_pose', Twist, queue_size = 20)
        
        
        
        rospy.Subscriber('trajectory', localPath, localPath_callback, queue_size=50)
        rospy.Subscriber('break_action', String, avoid_object_callback, queue_size=50)
        rospy.Subscriber('gnss_message', GNSS_CAN, nav_callback,queue_size = 10)                      #订阅GPS数据
        rospy.Subscriber("Motor_Feedback_mssage", Motor_Feedback,RVcallback,queue_size = 10)
        rate = rospy.Rate(100) 
        
        if (GNSS_flag == 1 and localPath_flag == 1): #判断组合惯导就绪 局部轨迹规划就绪
            
                      
            #print("path_x,path_y",LocalPath_x, LocalPath_y)
            
            #初始化车辆状态
            state = State(x=sx, y=sy, yaw=ryaw, v=0.0) 
            
            states.append(time, state)
            
            
            
            #主循环
            while localPath_flag == 1:   
                #lastIndex = len(LocalPath_x) - 1  
                target_course = TargetCourse(LocalPath_x, LocalPath_y) #产生目标跟踪曲线
                target_ind= target_course.search_target_index(state, Lf_optimal)
                
                st, lateral_error, curr_posx, curr_posy = get_lateral_dist(LocalPath_x, LocalPath_y, state.x, state.y)
                global target_speed
                target_speed = 7.0
                if object_bool == 0: 
                    #print("lateral_error",lateral_error)
                    #print("rv",rv)
                    
                    target_speed = 7.0
                   
                elif object_bool == 1:
                    target_speed = 7.0            
                
   
                            
                if (rv)/3.6 < 1.34 :
                    Lf_optimal = 3
                elif (rv)/3.6 >= 1.34 and (rv)/3.6 <= 5.36:
                    Lf_optimal =2.24*(rv/3.6)
                elif (rv)/3.6 > 5.36:
                    Lf_optimal = 12 
                #print("*" * 50)
                #print("target_speed",target_speed)    
                 
                #print("Lf_optimal=", Lf_optimal)
    
                acc_pub = proportional_control(target_speed, state.v)
                #print("*-"*50)
                #print("acc",ai)
                state = update(state)
                steer_pub, target_ind = pure_pursuit_steer_control(
                    state, target_course, target_ind, Lf_optimal)
                
                
                
                
                
                
                time += dt
                states.append(time, state)
                Lf_optimal_list.append(Lf_optimal)
                Lateral_Error.append(lateral_error)
                
                simple_publisher.publish(steer_pub)
                second_publisher.publish(acc_pub)
                
                
                vel_msg.linear.x = sx
                vel_msg.linear.y = sy
                vel_msg.linear.z = ryaw

                carpose_publisher.publish(vel_msg)
                # 发布Topic
                rate.sleep()
                #print("-"*50)
                #print("ai:", ai)
                #print("di",di)
    
                if show_animation:  # pragma: no cover
                    plt.cla()
                    # for stopping simulation with the esc key.
                    plt.gcf().canvas.mpl_connect(
                        'key_release_event',
                        lambda event: [exit(0) if event.key == 'escape' else None])
                    plot_arrow(state.x, state.y, state.yaw) #np.radians(state.yaw)
                    
                    plt.plot(LocalPath_x, LocalPath_y, "-r", label = "course")
                    
                    #plt.plot(gx, gy, "-k", label = "fit_trajectory")
                    plt.plot(states.x, states.y, "-b", label="trajectory")
                    plt.plot(LocalPath_x[target_ind], LocalPath_y[target_ind], "*-k", label="target")
                    plt.axis("equal")
                    plt.grid(True)
                    #plt.title("Lf_optimal_list:" + str(Lf_optimal_list)[1:])
                    plt.title("Speed[km/h]:" + str(state.v)[:4] )
                    plt.pause(0.001)
    

            
        else :
            print("no recieve sensor data")       
            
            
            
            
            

















if __name__ == '__main__':
    main_local_path()
    #main_MIT()
