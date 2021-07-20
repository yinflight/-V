#!/usr/bin/python2
# coding: utf-8
#!/usr/bin/env python

"""
三次多项式曲线规划+拟合
读取txt文件进行循迹或轨迹
规划二合一文件，可单独使用
固定速度行驶速行驶

"""


import numpy as np
import math
import bisect
import time
import matplotlib.pyplot as plt
import rospy
import scipy
import warnings
warnings.filterwarnings("ignore",".*GUI is implemented.*")
from scipy.signal import savgol_filter
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from control.msg import Motor_Feedback



k = 0.6  # control gain
Kp = 1.5  # speed propotional gain
dt = 0.1  # [s] time difference
L = 2.49  # [m] Wheel base of vehicle
bet = 0.2 # [m] dis of tracking points
max_steer = np.radians(30.0)  # [rad] max steering angle
sx, sy, yaw = 0, 0, 0 # [m] coordinate and heading of tracking points
zero_coord_x = 0.0
zero_coord_y = 0.0
rv = 0.0
show_animation = True


class Spline:
    """
    Cubic Spline class
    """

    def __init__(self, x, y):
        self.b, self.c, self.d, self.w = [], [], [], []

        self.x = x
        self.y = y

        self.nx = len(x)  # dimension of x
        h = np.diff(x)

        # calc coefficient c
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h)
        self.c = np.linalg.solve(A, B)
        #  print(self.c1)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            self.d.append((self.c[i + 1] - self.c[i]) / (3.0 * h[i]))
            tb = (self.a[i + 1] - self.a[i]) / h[i] - h[i] * \
                (self.c[i + 1] + 2.0 * self.c[i]) / 3.0
            self.b.append(tb)

    def calc(self, t):
        """
        Calc position

        if t is outside of the input x, return None

        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.a[i] + self.b[i] * dx + \
            self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

        return result

    def calcd(self, t):
        """
        Calc first derivative

        if t is outside of the input x, return None
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return result

    def calcdd(self, t):
        """
        Calc second derivative
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return result

    def __search_index(self, x):
        """
        search data segment index
        """
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        """
        calc matrix A for spline coefficient c
        """
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        #  print(A)
        return A

    def __calc_B(self, h):
        """
        calc matrix B for spline coefficient c
        """
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (self.a[i + 2] - self.a[i + 1]) / \
                h[i + 1] - 3.0 * (self.a[i + 1] - self.a[i]) / h[i]
        return B


class Spline2D:
    """
    2D Cubic Spline class

    """

    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = Spline(self.s, x)
        self.sy = Spline(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = [math.sqrt(idx ** 2 + idy ** 2)
                   for (idx, idy) in zip(dx, dy)]
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        """
        calc position
        """
        x = self.sx.calc(s)
        y = self.sy.calc(s)

        return x, y

    def calc_curvature(self, s):
        """
        calc curvature
        """
        dx = self.sx.calcd(s)
        ddx = self.sx.calcdd(s)
        dy = self.sy.calcd(s)
        ddy = self.sy.calcdd(s)
        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2))
        return k

    def calc_yaw(self, s):
        """
        calc yaw
        """
        dx = self.sx.calcd(s)
        dy = self.sy.calcd(s)
        yaw = math.atan2(dy, dx)
        return yaw
        
    def calc_ref_v(self, s):
        """
        calc velocity
        """
        ref_vx = self.sx.calcd(s)
        ref_vy = self.sy.calcd(s)
        rv = math.sqrt(ref_vx ** 2 + ref_vy ** 2)
        # print("ref_vx:",ref_vx)
        # print("ref_vy:", ref_vy)
        #print("rv:", rv)

        return  rv

    def calc_ref_a(self, s):
        """
        calc a
        """
        ref_ax = self.sx.calcdd(s)
        ref_ay = self.sy.calcdd(s)
        ra = math.sqrt(ref_ax ** 2 + ref_ay ** 2)
        # print("ref_ax:",ref_ax)
        # print("ref_ay:", ref_ay)
        #print("ra:", ra)

        return ra


def calc_spline_course(x, y, ds=0.1):
    sp = Spline2D(x, y)
    s = list(np.arange(0, sp.s[-1], ds))

    rx, ry, ryaw, rv, ra, rk = [], [], [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rv.append(sp.calc_ref_v(i_s))
        ra.append(sp.calc_ref_a(i_s))
        rk.append(sp.calc_curvature(i_s))

    return rx, ry, ryaw, rv, ra, rk, s





class State:
    #车辆运动学模型#
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((L / 2) * math.cos(self.yaw))   #计算后轮位置X
        self.rear_y = self.y - ((L / 2) * math.sin(self.yaw))   #计算后轮位置Y

#######################################
#定义update(state, a, delta) 
#输入：状态，GPS位置队列，
#返回：返回角度和当前预描点位置
#######################################
def update(state, a, delta):   
    global yaw,sx,sy
    try:
        state.yaw=float(yaw)
    	state.x=float(sx)
        state.y=float(sy)
    except:
        print("state updata error")
    state.x = state.x - zero_coord_x
    state.y  = state.y - zero_coord_y
    state.yaw= 90 - state.yaw
    print("state.yaw:",state.yaw)
    state.yaw = (state.yaw * np.pi)/180
    '''
    while state.yaw > np.pi:
        state.yaw -= 2.0 * np.pi

    while state.yaw < -np.pi:
        state.yaw += 2.0 * np.pi
    '''
    print("state:",state.x,state.y,state.yaw)
    print("state.yaw:",math.sin(state.yaw))  
    #state.v=float(rv)
    #state.v=state.v/3.6   # speed is m/s
   
    '''state.x = state.x + state.v * math.cos(state.yaw) * dt
    print("state.x:",state.x)
    print("state.x:",state.x)
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt'''
    state.v = rv / 3.6#state.v + a * dt                                   #更新此时的速度    
    state.rear_x = state.x #- ((L / 2) * math.cos(state.yaw))     #计算当前的后轮位置X
    state.rear_y = state.y #- ((L / 2) * math.sin(state.yaw))     #计算当前的后轮位置Y

    return state   



def pid_control(target, current):
    """
    Proportional control for the speed.

    :param target: (float)
    :param current: (float)
    :return: (float)
    """
    return Kp * (target - current)


def stanley_control(state, cx, cy, cyaw, cv, ca, ck, last_target_idx, last_idx):
    """
    Stanley steering control.

    :param state: (State object)
    :param cx: ([float])
    :param cy: ([float])
    :param cyaw: ([float])
    :param last_target_idx: (int)
    :return: (float, int)
    """
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw) 

    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, state.v)
    if last_idx - current_target_idx > 4:
        cur = [ck[current_target_idx],ck[current_target_idx + 1],ck[current_target_idx + 2],ck[current_target_idx + 3],ck[current_target_idx + 4]]
        mid = np.median(cur)
        k_cur = abs(mid) // abs(ck[current_target_idx])    
    else:
        k_cur = 1
    #print("ck[current_target_idx]:",k_cur)
    
    # Steering control
    print("180-theta_e:", (np.pi - theta_e) * 180 / np.pi)
    print("v_2_theta_d:", theta_d * 180 / np.pi)
    delta = np.pi - theta_e + theta_d
    delta = normalize_angle(delta)
    delta = np.clip(delta, -max_steer, max_steer)
    delta = - delta * 180 / np.pi
    delta = 2.8 * delta
    # delta = k_cur * delta
    print("delta_pre",delta)

    # Speed control
    #error_v = cv[current_target_idx] - state.v

    return delta, current_target_idx


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


def calc_target_index(state, cx, cy):
    """
    Compute index in the trajectory list of the target.

    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
   # Calc front axle position
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = [np.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
    closest_error = min(d)
    target_idx = d.index(closest_error)

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                      - np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle


def Remove_Duplicates():

    blank = []
    white = []
    yellow = []
    cx = []
    cy = []
    nums, ber = np.loadtxt("/home/robot/Robot/SmartCar_WS/src/fuzzy_purepursuit/scripts/tir_planning.txt", dtype=str, delimiter=',',
                           unpack=True)
    print(len(nums))
    for i in range(len(nums)):
        if not nums[i] in blank:
            #blank.append(nums[i])
            yellow.append(float(nums[i]))
            white.append(float(ber[i]))
    bx = yellow[0]
    by = white[0]
    for i in range(len(yellow)):
        dx = yellow[i] - bx
        dy = white[i] - by
        dis = math.sqrt(dx ** 2 + dy ** 2)
        if dis > bet:
            cx.append(yellow[i])
            # print(type(cx))
            cy.append(white[i])
            bx = yellow[i]
            by = white[i]
            # print(dis)

    cx = np.array(cx)
    cy = np.array(cy)
    #print(type(cy))
    #print(cy[2])
    global zero_coord_x,zero_coord_y
    zero_coord_x = cx[0]
    zero_coord_y = cy[0]
    
    cx = cx - cx[0]
    cy = cy - cy[0]
    cy = scipy.signal.savgol_filter(cy, 63, 3)
    print(len(cx))
    return cx,cy




def RVcallback(Motor_Feedback):
    global rv
    rv = Motor_Feedback.Base_Vehspd

def Statuscallback(msg):
    global sx, sy, yaw
    sx, sy, yaw = msg.data.split(',')

    # state = update(state, ai, di,msg)
    # return sx,sy,yaw

def main():

    rospy.init_node('Stanley_controller_node', anonymous=False)  # 初始化ROS节点
    first_publisher = rospy.Publisher('delta', Float32, queue_size=10)  # 定义Publisher对象
    second_publisher = rospy.Publisher('c_a', Float32, queue_size=10)  # 定义Publisher对象
    rate = rospy.Rate(10)  # 设置Topic发布的频率（Hz）

    while not rospy.is_shutdown():
        sub = rospy.Subscriber('chatter',
                               String, Statuscallback, queue_size=10)
        rospy.Subscriber("Motor_Feedback_mssage", Motor_Feedback, RVcallback, queue_size=10)

        tx, ty = Remove_Duplicates() #得到xy点
        print("zero_coord_x:",zero_coord_x)
       
        cx, cy, cyaw, cv, ca, ck, s = calc_spline_course(
            tx, ty, ds=0.1) #tracking points、coordinate、heading、K
        print("cx:",cx)
        print("cy:",cy)
        print("cyaw:",cyaw)
        print("cv:",cv)
        print("ca:",ca)
        print("ck:",ck)
        plt.plot(cx,cyaw, "-r", label="ccc")
        plt.plot()
        plt.show()
        print("cyaw",cyaw)
        target_speed = 10.0 / 3.6  # [m/s]

        max_simulation_time = 150.0

        # Initial state
        state = State(x=cx[0], y=cy[0], yaw=np.radians(-90.0), v=0.0)
        ##state.yaw_init待更新

        last_idx = len(cx) - 1
        time = 0.0
        x = [state.x]
        y = [state.y]
        yaw = [state.yaw]
        v = [state.v]
        a = [0.0]
        t = [0.0]
        target_idx, _ = calc_target_index(state, cx, cy)

        while max_simulation_time >= time and last_idx > target_idx:
            ai = pid_control(target_speed, state.v)
            di, target_idx = stanley_control(state, cx, cy, cyaw, cv, ca, ck, target_idx, last_idx)
            #state.update(ai, di)
            state = update(state, ai, di)
            
            first_publisher.publish(di)
            second_publisher.publish(ai)

            time += dt

            x.append(state.x)
            y.append(state.y)
            yaw.append(state.yaw)
            v.append(state.v)
            a.append(ai)
            t.append(time)
            rate.sleep()


            if show_animation:  # pragma: no cover
                plt.cla()
                plt.plot(cx, cy, ".r", label="course")
                plt.plot(x, y, "-b", label="trajectory")
                plt.plot(cx[target_idx], cy[target_idx], "xg", label="target")
                plt.axis("equal")
                plt.grid(True)
                plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
                plt.pause(0.001)

        # Test
        assert last_idx >= target_idx, "Cannot reach goal"
        print("x",x)
        print("*********************************************************************************")   
        print("y",y)        
        print("*********************************************************************************")  
        print("yaw",yaw)
        print("*********************************************************************************")
        print("v",v)
        print("*********************************************************************************")
        print("a",a)

        if show_animation:  # pragma: no cover
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.legend()
            plt.xlabel("x[m]")
            plt.ylabel("y[m]")
            plt.axis("equal")
            plt.grid(True)

            plt.subplots(1)
            plt.plot(t, [iv * 3.6 for iv in v], "-r")
            plt.xlabel("Time[s]")
            plt.ylabel("Speed[km/h]")
            plt.grid(True)
            plt.show()


if __name__ == '__main__':
    main()
