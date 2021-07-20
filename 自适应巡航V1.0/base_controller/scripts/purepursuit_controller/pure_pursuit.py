#!/usr/bin/python2
# coding: utf-8
#!/usr/bin/env python
'''
解析文件可以单独外置进行操作
target_speed可以设置成类似cx，cy的数组
1、确定车辆的位置：通过gps，惯导等设备，确定车辆的位置，航向角等实时状态信息；
2、找到离车最近的位置：在前视距离范围内，可能会有多个数据目标路径的点，应当选取一2个距离起点最接近前视距离的那个点，为了找到最满足要求的这个点，首先可以选取一个在路径上里此刻最近的点，以确定自己此刻在规划出的路径中的位置。
3、找到目标点：利用上一步获取的位置点，采取一定的计算方法，获取规划出的路径中，距离此点距离最接近前视距离的点，把这个点设为目标点。
4、利用pure pursuit算法的计算公式，计算出到达目标点所需的转向角δ。
5、根据单位时间内车辆的运动更新车辆的状态。
'''

import rospy 
import numpy as np
import math
import time
import matplotlib.pyplot as plt
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from control.msg import Motor_Feedback


k = 0.1                                #前向增益
Kp = 0.2                               #速度比例增益
dt = 0.1                               #[s]
L = 2.49                               #车辆轴距[m] 
Lfc = 3                                #超前距离
bet = 1.0                              #缓冲距离
yaw = " "
sx = " "
sy = " "
angle = 0.0
rv = 0.0
zero_cord_x = 0.0
zero_cord_y = 0.0


old_nearest_point_index = None         #上次最近点
show_animation = True                  #显示动画


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
    state.x = state.x - zero_cord_x
    state.y  = state.y  - zero_cord_y
    state.yaw= 90 - state.yaw
    print("state.yaw:",state.yaw)
    state.yaw = (state.yaw * np.pi)/180
    '''
    while state.yaw > np.pi:
        state.yaw -= 2.0 * np.pi

    while state.yaw < -np.pi:
        state.yaw += 2.0 * np.pi
    '''
    print("state.yaw:",state.yaw)
    print("state.yaw:",math.sin(state.yaw))  
    #state.v=float(rv)
    #state.v=state.v/3.6   # speed is m/s
   
    '''state.x = state.x + state.v * math.cos(state.yaw) * dt
    print("state.x:",state.x)
    print("state.x:",state.x)
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt'''
    state.v = rv#state.v + a * dt                                   #更新此时的速度    
    state.rear_x = state.x #- ((L / 2) * math.cos(state.yaw))     #计算当前的后轮位置X
    state.rear_y = state.y #- ((L / 2) * math.sin(state.yaw))     #计算当前的后轮位置Y

    return state   


#######################################
#函数：纵向PID 
#输入：目标值，测量值
#返回：P_PIDControl
#######################################
def PIDControl(target, current):
    a = Kp * (target - current*1.5)

    return a

#######################################
#函数：pure_pursuit_control 
#输入：状态，GPS位置队列，
#返回：返回角度和当前预描点
#######################################
def pure_pursuit_control(state, cx, cy, pind):
    global angle
    ind = calc_target_index(state, cx, cy)

    if pind >= ind:
        ind = pind

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1
     
    apl = (math.atan2(ty - state.rear_y, tx - state.rear_x))
    alpha = apl - state.yaw
    kx=tx-state.rear_x
    ky=ty-state.rear_y
    #L = math.sqrt(kx ** 2 + ky ** 2)
    Lf =Lfc # k * state.v + Lfc
    buf1 = math.sin(alpha)
    buf2 = 2.0 * L * math.sin(alpha) / Lf
    buf3 = (math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0))
    print("alpha_chuliqian_hudu",apl)
    print("alpha_chuliqian_jiaodu",apl/math.pi*180)
    print("alpha_chulihou_hudu",alpha)
    print("alpha_chulihou_jiaodu",alpha/math.pi*180)
    print("buf1",buf1)
    print("buf2",buf2)
    print("buf3",buf3)
    #bu = (math.atan2(ty - state.rear_y, tx - state.rear_x))
    #bu2 = (math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0))
    #bu3 = (math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0))/math.pi*180
    delta = -(math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0))/math.pi*180
    
    '''st_delta = (math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0))/math.pi*180
    #delta = p*(math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0))/math.pi*180
    delta = st_delta - angle
    angle = st_delta'''
    
        

    print("Lf:",Lf)
    print("delta:jiaodu:",delta)
    print("state.v::",state.v)
    
    return delta, ind


#######################################
#函数：calc_distance(state, point_x, point_y) 
#输入：state, point_x, point_y
#返回：距离
#######################################
def calc_distance(state, point_x, point_y):
    dx = state.rear_x - point_x
    dy = state.rear_y - point_y
    return math.sqrt(dx ** 2 + dy ** 2)



#######################################
#函数：calc_target_index(state, cx, cy)
#功能：遍历全部cx，cy如果需要提高实时性
#功能：可以根据当前的ind来缩小计算的范围
#输入：state, point_x, point_y
#返回：距离
#######################################
def calc_target_index(state, cx, cy):
    
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d))                                                           #最近的目标点的距离和索引号
    print("dddd:",min(d))
    print("ind:",ind)
    L = 0.0

    Lf = Lfc #k * state.v + Lfc

    while Lf > L and (ind + 1) < len(cx):
        #寻找前视距离内最远的路径点
        dx = cx[ind]-state.rear_x
        dy = cx[ind]-state.rear_y
        #dx = cx[ind + 1] - cx[ind]
        #dy = cx[ind + 1] - cx[ind]
        L = math.sqrt(dx ** 2 + dy ** 2)
        print("LLLL:",L)
        ind += 1

    return ind


#######################################
#函数：plot_arrow
#######################################
def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
        print("xxxxxxxxxxxxxxx")
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)



#######################################
#函数：RVcallback(data)
#功能：订阅当前车速
#data的数据类型与Subscriber接收的Topic对应的消息类型一致
#######################################
def RVcallback(Motor_Feedback): 
    global rv
    rv = Motor_Feedback.Base_Vehspd
    #rospy.loginfo('I heard: %s', data.data)


#######################################
#函数：callback(msg)
#功能：订阅组合惯导纬度、经度、航向角
#data的数据类型与Subscriber接收的Topic对应的消息类型一致
#######################################
def callback(msg):
    
    global yaw,sx,sy
    sx,sy,yaw=msg.data.split(',')
    
   
    print("msg.data:",sx,sy,yaw)

    #state = update(state, ai, di,msg) 
    #return yaw,sx,sy


def moving_average(interval, windowsize):
    window = np.ones(int(windowsize)) / float(windowsize)
    re = np.convolve(interval, window, 'same')
    return re

#######################################
#函数：main()
#######################################
def main():

    blank = []                                                                      #buffer
    white = []                                                                      #buffer
    yellow = []                                                                     #buffer
    cx = []                                                                         #所采集预描点的x
    cy = []                                                                         #所采集预描点的x
    rospy.init_node('pure_pursuit', anonymous = False)                              #初始化ROS节点 pure_pursuit 
    simple_publisher = rospy.Publisher('delta', Float32, queue_size = 10)           #定义Publisher对象
    second_publisher = rospy.Publisher('c_a', Float32, queue_size = 10)  
    rate = rospy.Rate(90)                                                           #设置Topic发布的频率（Hz）

    while not rospy.is_shutdown():
        sub = rospy.Subscriber('chatter', String, callback,queue_size = 10)                       #订阅GPS数据
        rospy.Subscriber("Motor_Feedback_mssage", Motor_Feedback,RVcallback,queue_size = 10)             #订阅车速
        
        #读取预描点
        nums, ber = np.loadtxt("/home/robot/Robot/SmartCar_WS/src/fuzzy_purepursuit/scripts/curve.txt", dtype=str, delimiter=',', unpack=True)
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
                cx.append(yellow[i])                                                #使cx，cy中点均满足要求
                cy.append(white[i])
                bx = yellow[i]
                by = white[i]

        cx = np.array(cx)                                                           #将列表转换成数组
        cy = np.array(cy)
        #print("cx:",cx)
        #print("cy:",cy)
        global zero_cord_x,zero_cord_y
        zero_cord_x = cx[0]
        zero_cord_y = cy[0]
        cx = cx - cx[0]
        cy = cy - cy[0]
        cy = moving_average(cy, 10)
        plt.plot(cx,cy, "-r", label="ccc")
        plt.plot()
        plt.show()

        target_speed = 10.0                                                  # [m/s]

        T = 1000.0                                                                  # max simulation time

        # initial state
        state = State(x=cx[0], y=cy[0], yaw=0.0, v=0.0)                             #初始状态
        

        lastIndex = len(cx) - 1                                                     #定义最后一个点是第几个
        time = 0.0
        x = [state.x]
        y = [state.y]
        yaw = [state.yaw]
        v = [state.v]
        t = [0.0]
        print("state")
        target_ind = calc_target_index(state, cx, cy)                               #目标预描点是第几个

        print("state")
        print("target:",target_ind)
        print("target:",lastIndex)
        

        while T >= time and lastIndex > target_ind:
            ai = PIDControl(target_speed, rv)                                  #车的加速度
            di, target_ind = pure_pursuit_control(state, cx, cy, target_ind)        #返回角度和当前预描点
            state = update(state, ai, di)
            '''print("state:",state.x)
            print("state:",state.y)
            print("state:",state.yaw)'''
            print("target:",target_ind)


            time = time + dt

            x.append(state.x)
            y.append(state.y)
            #print(type(x))
            #print(y)
            yaw.append(state.yaw)
            v.append(state.v)
            t.append(time)
            simple_publisher.publish(di)
            second_publisher.publish(ai)
            #发布Topic
            rate.sleep()
            print("state_x:",x)
            print("state_y:",y)
            if show_animation:  # pragma: no cover
                plt.cla()
                plot_arrow(state.x, state.y, state.yaw)
                plt.plot(cx,cy, "-r", label="course")
                plt.plot(x, y, "-b", label="trajectory")
                plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
                #kx=cx(target_ind)-x
                #ky=cy(target_ind)-y
                #L = math.sqrt(kx ** 2 + ky ** 2)
                #print("Lf:",math.sqrt(kx ** 2 + ky ** 2))
                plt.axis("equal")
                #plt.axis([-270,110,-190,190])#北汽
                #plt.axis([-50, 250, -250, 250])#abc
                plt.grid(True)
                plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
                plt.pause(0.001)

        # Test
        assert lastIndex >= target_ind, "Cannot goal"

        if show_animation:  # pragma: no cover
            plt.cla()
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
    print("Pure pursuit path tracking start")
    main()
