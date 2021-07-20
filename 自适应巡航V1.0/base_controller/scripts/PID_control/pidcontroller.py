#!/usr/bin/python2
# coding: utf-8
#!/usr/bin/env python



import matplotlib.pyplot as plt
import math
import numpy as np
import PID
import sys
import rospy
import time
import matplotlib.pyplot as plt
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from move_base.msg import Motor_Feedback
from mems.msg import nav

sys.path.append("../../PathPlanning/CubicSpline/")



sx = 0.0
sy = 0.0
ryaw = 0.0
angle = 0.0
rv = 0.0
zero_cord_x = 0.0
zero_cord_y = 0.0



try:
    import cubic_spline_planner
except:
    raise

Kp = 1.0  # 速度比例
dt = 0.1  # [s]
WB = 2.5  # [m] 轴距
THRESH_DIST=0.10
bet = 1.0
show_animation = True

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x + ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y + ((WB / 2) * math.sin(self.yaw))

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
    global ryaw, sx, sy, WB
 
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
    state.rear_x = sx  #+ ((WB / 2) * math.cos(ryaw))     #计算当前的后轮位置X
    state.rear_y = sy  #+ (WB / 2) * math.sin(ryaw))     #计算当前的后轮位置Y

    return state




def proportional_control(target, current):

    a = Kp * (target - current)

    return a




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




def Longitudinal_pid(P, I, D ,target,feedback):
    pid = PID.PID(P, I, D)
    pid.SetPoint = target
    pid.update(feedback)
    output = pid.output
    print("output",output)
    return output


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



def gps_to_map(state):
    
    state.x = sx 
    state.y = sy 
    state.yaw = 90-state.yaw + 180 
    print("zero_cord_x:", zero_cord_x)
    print("zero_cord_y:", zero_cord_y)
    print("state.x:", state.x)
    print("state.y:", state.y)
    print("state.yaw:", state.yaw)
    state.yaw = (state.yaw * np.pi) / 180

#######################################
#函数：RVcallback(data)
#功能：订阅当前车速
#data的数据类型与Subscriber接收的Topic对应的消息类型一致
#######################################
def RVcallback(Motor_Feedback):
    global rv
    rv = Motor_Feedback.Base_Vehspd
    print("*"*50)
    print("rv:",rv)
    #rospy.loginfo('I heard: %s', data.data)


#######################################
# 函数：callback(msg)
# 功能：订阅组合惯导纬度、经度、航向角
# data的数据类型与Subscriber接收的Topic对应的消息类型一致
#######################################
def nav_callback(nav_msg):
    global ryaw, sx, sy
    sx = nav_msg.latitude - zero_cord_x
    sy = nav_msg.longitude - zero_cord_y
    ryaw = nav_msg.course_angle
    #sx.strip()
    print("nav_msg:", sx, sy, ryaw)

    #state = update(state)
    # return yaw,sx,sy





def main():

    blank = []                                                                      #buffer
    white = []                                                                      #buffer
    yellow = []                                                                     #buffer
    GPS_x = []                                                                         #所采集预描点的x
    GPS_y = []                                                                         #所采集预描点的x
 #读取预描点
    nums, ber = np.loadtxt("/home/robot/Robot/Ob_tracking_WS/src/mems/data/ABC.txt", dtype=str, delimiter=',', unpack=True)
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
#初始化
    target_lat = 0.0
    target_speed = 5.0
    ax = GPS_x
    ay = GPS_y
    cx, cy, cyaw, ck, s, cv = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=1.0)
    plt.plot(cx,cy, "-k", label="GPS cubic_spline_planner point ")
    plt.plot()
    plt.show()
    T = 100000.0  # 最大仿真时间

    rospy.init_node('pure_pursuit', anonymous = False)                              #初始化ROS节点 pure_pursuit
    simple_publisher = rospy.Publisher('delta', Float32, queue_size = 10)           #定义Publisher对象
    second_publisher = rospy.Publisher('c_a', Float32, queue_size = 10)
    rate = rospy.Rate(90)
    # 初始状态
    state = State(x=ax[0], y=ay[0], yaw=0, v=0.0)

    time = 0.0
    states = States()
    states.append(time, state)



    while not rospy.is_shutdown():
        sub = rospy.Subscriber('nav_mssage', nav, nav_callback,queue_size = 10)                       #订阅GPS数据
        rospy.Subscriber("Motor_Feedback_mssage", Motor_Feedback,RVcallback,queue_size = 10)

#主循环
        while T >= time :
    
            st, lateral_error, curr_posx, curr_posy = get_lateral_dist(cx, cy, state.rear_x, state.rear_y)
            print("*" * 50)
            print("lateral_error",lateral_error)
            di = Longitudinal_pid(15.8, 0.0, 8.9, target_lat, lateral_error)
            ai = proportional_control(target_speed, state.v)
    
    
            time += dt
            state = update(state)
            print("*"*50)
            print(state.x,state.y,state.yaw)
            states.append(time, state)
            
            simple_publisher.publish(di)
            second_publisher.publish(ai)
            # 发布Topic
            rate.sleep()
            print("-"*50)
            print("ai:", ai)
            print("di",di)
            
    
    
            if show_animation:  # pragma: no cover
                plt.cla()
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                plot_arrow(state.x, state.y, state.yaw)
                plt.plot(cx, cy, "-r", label = "course")
                #plt.plot(gx, gy, "-k", label = "fit_trajectory")
                plt.plot(states.x, states.y, "-b", label="trajectory")
                plt.plot(cx[st], cy[st], "*-k", label="target")
                plt.axis("equal")
                plt.grid(True)
                #plt.title("Lf_optimal_list:" + str(Lf_optimal_list)[1:])
                plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4] )
                plt.pause(0.001)
    
        # Test
    
    
        lastIndex = len(cx) - 1
    
        if state.x > cx[-1] and state.y > cy[-1]:  # pragma: no cover
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
    
            plt.show()



if __name__ == '__main__':
    print("PID path tracking simulation start")
    main()
