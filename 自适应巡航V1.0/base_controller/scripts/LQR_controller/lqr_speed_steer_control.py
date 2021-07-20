#!/usr/bin/python2
# -*- coding: UTF-8 -*-
# coding: utf-8
#!/usr/bin/env python

import math
import sys
import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg as la
import rospy
import math
import time
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from move_base.msg import Motor_Feedback
from mems.msg import nav
import sys




sys.path.append("../../PathPlanning/CubicSpline/")

try:
    import cubic_spline_planner
except ImportError:
    raise

# === Parameters =====
bet = 1.0
sx = 0.0
sy = 0.0
ryaw = 0.0
angle = 0.0
rv = 0.0
zero_cord_x = 0.0
zero_cord_y = 0.0
Kp = 1.5

# LQR parameter
lqr_Q = np.eye(5)
lqr_R = np.eye(2)
dt = 0.1  # time tick[s]
L = 2.49  # Wheel base of the vehicle [m]
max_steer = np.deg2rad(40.0)  # maximum steering angle[rad]

show_animation = True


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

#修改




def update(state):
    #global ryaw, sx, sy
 
    try:
        state.x = sx 
        state.yaw = ryaw
        state.y = sy 
    except:
        print("state updata error")
    gps_to_map(state)

    state.v = rv  # state.v + a * dt                                   #更新此时的速度
    state.rear_x = state.x  # - ((L / 2) * math.cos(state.yaw))     #计算当前的后轮位置X
    state.rear_y = state.y  # - ((L / 2) * math.sin(state.yaw))     #计算当前的后轮位置Y

    return state



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

















def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def solve_dare(A, B, Q, R):
    """
    solve a discrete time_Algebraic Riccati equation (DARE)
    """
    x = Q
    x_next = Q
    max_iter = 150
    eps = 0.01

    for i in range(max_iter):
        x_next = np.dot(np.dot(A.T , x) , A) - np.dot(np.dot(np.dot(A.T , x) , B) , np.dot(np.dot(np.dot(la.inv(R + np.dot(np.dot(B.T , x) , B)) , B.T) , x) , A) + Q)
        if (abs(x_next - x)).max() < eps:
            break
        x = x_next

    return x_next


def dlqr(A, B, Q, R):
    """Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    # ref Bertsekas, p.151
    """

    # first, try to solve the ricatti equation
    X = solve_dare(A, B, Q, R)

    # compute the LQR gain
    K = np.dot(la.inv( np.dot(np.dot(B.T , X) , B) + R) , (np.dot(np.dot(B.T , X) , A)))

    eig_result = la.eig(A - np.dot(B , K) )

    return K, X, eig_result[0]


def lqr_speed_steering_control(state, cx, cy, cyaw, ck, pe, pth_e, sp, Q, R):
    ind, e = calc_nearest_index(state, cx, cy, cyaw)

    tv = sp[ind]

    k = ck[ind]
    v = state.v
    th_e = pi_2_pi(state.yaw - cyaw[ind])

    # A = [1.0, dt, 0.0, 0.0, 0.0
    #      0.0, 0.0, v, 0.0, 0.0]
    #      0.0, 0.0, 1.0, dt, 0.0]
    #      0.0, 0.0, 0.0, 0.0, 0.0]
    #      0.0, 0.0, 0.0, 0.0, 1.0]
    A = np.zeros((5, 5))
    A[0, 0] = 1.0
    A[0, 1] = dt
    A[1, 2] = v
    A[2, 2] = 1.0
    A[2, 3] = dt
    A[4, 4] = 1.0

    # B = [0.0, 0.0
    #     0.0, 0.0
    #     0.0, 0.0
    #     v/L, 0.0
    #     0.0, dt]
    B = np.zeros((5, 2))
    B[3, 0] = v / L
    B[4, 1] = dt

    K, _, _ = dlqr(A, B, Q, R)

    # state vector
    # x = [e, dot_e, th_e, dot_th_e, delta_v]
    # e: lateral distance to the path
    # dot_e: derivative of e
    # th_e: angle difference to the path
    # dot_th_e: derivative of th_e
    # delta_v: difference between current speed and target speed
    x = np.zeros((5, 1))
    x[0, 0] = e
    x[1, 0] = (e - pe) / dt
    x[2, 0] = th_e
    x[3, 0] = (th_e - pth_e) / dt
    x[4, 0] = v - tv

    # input vector
    # u = [delta, accel]
    # delta: steering angle
    # accel: acceleration
    ustar = np.dot(-K , x)

    # calc steering input
    ff = math.atan2(L * k, 1)  # feedforward steering angle
    fb = pi_2_pi(ustar[0, 0])  # feedback steering angle
    delta = ff + fb

    # calc accel input
    accel = ustar[1, 0]

    return delta, ind, e, th_e, accel


def calc_nearest_index(state, cx, cy, cyaw):
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind)

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


def do_simulation(cx, cy, cyaw, ck, speed_profile, goal):
    T = 5000000.0  # max simulation time
    goal_dis = 0.3
    stop_speed = 0.05

    state = State(x=-0.0, y=-0.0, yaw=0.0, v=0.0)

    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]

    e, e_th = 0.0, 0.0

    while T >= time:
        dl, target_ind, e, e_th, ai = lqr_speed_steering_control(
            state, cx, cy, cyaw, ck, e, e_th, speed_profile, lqr_Q, lqr_R)

        state = update(state)

        if abs(state.v) <= stop_speed:
            target_ind += 1

        time = time + dt

        # check goal
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        if math.hypot(dx, dy) <= goal_dis:
            print("Goal")
            break

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        if target_ind % 1 == 0 and show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "ob", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.plot(t, yaw, "-k", label="speed")
            plt.axis("equal")
            plt.grid(True)
            plt.title("speed[km/h]:" + str(round(state.v , 2))
                      + ",target index:" + str(target_ind))
            plt.pause(0.0001)

    return t, x, y, yaw, v


def calc_speed_profile(cyaw, target_speed):
    speed_profile = [target_speed] * len(cyaw)

    direction = 1.0

    # Set stop point
    for i in range(len(cyaw) - 1):
        dyaw = abs(cyaw[i + 1] - cyaw[i])
        switch = math.pi / 4.0 <= dyaw < math.pi / 2.0

        if switch:
            direction *= -1

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

        if switch:
            speed_profile[i] = 0.0

    # speed down
    for i in range(40):
        speed_profile[-i] = target_speed / (50 - i)
        if speed_profile[-i] <= 1.0 / 3.6:
            speed_profile[-i] = 1.0 / 3.6

    return speed_profile




def proportional_control(target, current):
    print("*"*50)
    print("target=",current)
    print("target - current",target - current)
    a = Kp * (target - current)

    return a









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
    
    
    
    rospy.init_node('LQR_control', anonymous = False)                              #初始化ROS节点 pure_pursuit
    simple_publisher = rospy.Publisher('delta', Float32, queue_size = 10)           #定义Publisher对象
    second_publisher = rospy.Publisher('c_a', Float32, queue_size = 10)
    rate = rospy.Rate(90)
    
    ax = GPS_x
    ay = GPS_y
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=1.0)
    plt.plot(cx,cy, "-k", label="GPS cubic_spline_planner point ")
    plt.plot()
    plt.show()

    goal = [ax[-1], ay[-1]]

    target_speed = 10.0   # simulation parameter km/h -> m/s

    sp = calc_speed_profile(cyaw, target_speed)
    
    
    while not rospy.is_shutdown():
        sub = rospy.Subscriber('nav_mssage', nav, nav_callback,queue_size = 10)                       #订阅GPS数据
        rospy.Subscriber("Motor_Feedback_mssage", Motor_Feedback,RVcallback,queue_size = 10)
    

        t, x, y, yaw, v = do_simulation(cx, cy, cyaw, ck, sp, goal)
    
        ai = proportional_control(v, state.v)
        simple_publisher.publish(di)
        second_publisher.publish(ai)
            # 发布Topic
        rate.sleep()
        print("-"*50)
        print("ai:", ai)
        print("di",di)
    
    
        if show_animation:  # pragma: no cover
            plt.close()
            plt.subplots(1)
            plt.plot(ax, ay, "xb", label="waypoints")
            plt.plot(cx, cy, "-r", label="target course")
            plt.plot(x, y, "-g", label="tracking")
    
            plt.grid(True)
            plt.axis("equal")
            plt.xlabel("x[m]")
            plt.ylabel("y[m]")
            plt.legend()
    
            plt.subplots(1)
            plt.plot(s, [np.rad2deg(iyaw) for iyaw in cyaw], "-r", label="yaw")
            plt.grid(True)
            plt.legend()
            plt.xlabel("line length[m]")
            plt.ylabel("yaw angle[deg]")
    
            plt.subplots(1)
            plt.plot(s, ck, "-r", label="curvature")
            plt.grid(True)
            plt.legend()
            plt.xlabel("line length[m]")
            plt.ylabel("curvature [1/m]")
    
            plt.show()


if __name__ == '__main__':
    main()
