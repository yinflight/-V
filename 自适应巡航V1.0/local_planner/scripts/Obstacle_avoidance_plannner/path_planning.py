#!/usr/bin/python2
# -*- coding: UTF-8 -*-
# coding: utf-8
#!/usr/bin/env python


'''
发布轨迹信息 
path.x; path.y; c_speed;

'''







import numpy as np
import matplotlib.pyplot as plt
import copy
import math
from cubic_spline import Spline2D
from polynomials import QuarticPolynomial, QuinticPolynomial
import time
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from local_planner.msg import localPath
from geometry_msgs.msg import PoseStamped, Quaternion
import tf
from CAN_driver.msg import Motor_Feedback
from GNSS_driver.msg import GNSS_CAN
import sys



# 参数
MAX_SPEED = 30.0 # 最大速度 [m/s]
MAX_ACCEL = 50.0  # 最大加速度 [m/ss]
MAX_CURVATURE = 30.0  # 最大曲率 [1/m]
MAX_ROAD_WIDTH = 10.0  # 最大道路宽度 [m]
D_ROAD_W = 2.0 # 路宽采样间隔 [m]
DT = 0.3  # Delta T[s]
MAXT = 6.0  # 最大预测时间 [m]
MINT = 4.0  # 最小预测时间 [m]
TARGET_SPEED = 15.0/3.6   # 目标速度 [m/s] 即纵向速度保持
D_T_S = 10.0/3.6   # 目标opo][]o][o][\o][o][o速度采样间隔 [m/s]
N_S_SAMPLE = 0.1  # 目标速度采样数量
ROBOT_RADIUS = 2.3  # 车辆半径 [m]
THRESH_DIST=0.01

# 损失函数权重
KJ = 0.8
KT = 0.1
KD = 20.0
KLAT = 0.8
KLON = 0.2
show_animation = True


Gob_x = []
Gob_y = []


#规划失败标志 1   决策层需要
PathFail_flag = 0 


class FrenetPath:

    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []


def calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0):

    frenet_paths = []

    # generate path to each offset goal
    for di in np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W):
        # 采样，并对每一个目标配置生成轨迹
        # Lateral motion planning
        for Ti in np.arange(MINT, MAXT, DT):
            fp = FrenetPath()
            # 计算出关于目标配置di，Ti的横向多项式
            lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

            fp.t = [t for t in np.arange(0.0, Ti, DT)]
            fp.d = [lat_qp.calc_point(t) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            # 纵向速度规划 (速度保持)
            # Loongitudinal motion planning (Velocity keeping)
            for tv in np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE, TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S):
                tfp = copy.deepcopy(fp)
                lon_qp = QuarticPolynomial(s0, c_speed, 0.0, tv, 0.0, Ti)

                tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]


                ###########################################################
                #高速时的损失函数
                ###########################################################
                Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk
                # square of diff from target speed
                ds = (TARGET_SPEED - tfp.s_d[-1])**2
                # 横向的损失函数
                tfp.cd = KJ * Jp + KT * Ti + KD * tfp.d[-1]**2
                # 纵向的损失函数
                tfp.cv = KJ * Js + KT * Ti + KD * ds
                # 总的损失函数为d 和 s方向的损失函数乘对应的系数相加

                #########################################################
                #低速时的损失函数
                #########################################################
                # # 低速时的损失函数
                # ltfp = copy.deepcopy(tfp)
                # ltfp.d_sss = [lat_qp.calc_third_derivative(s) for s in tfp.s]
                # Jp_s = sum(np.power(ltfp.d_sss, 2))  # square of jerk
                # Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk
                # # S = s1 - s0
                # dS = tfp.s[-1] - s0
                # #横向的损失函数
                # tfp.cd = KJ * Jp_s + KT * dS + KD * tfp.d[-1] ** 2
                # #纵向的损失函数
                # tfp.cv = KJ * Js + KT * Ti + KD * ds
                
                tfp.cf = KLAT * tfp.cd + KLON * tfp.cv
                frenet_paths.append(tfp)
    return frenet_paths


def calc_global_paths(fplist, csp):
    for fp in fplist:
        # calc global positions
        for i in range(len(fp.s)):
            ix, iy = csp.calc_position(fp.s[i])
            if ix is None:
                break
            iyaw = csp.calc_yaw(fp.s[i])
            di = fp.d[i]
            fx = ix + di * math.cos(iyaw + math.pi / 2.0)
            fy = iy + di * math.sin(iyaw + math.pi / 2.0)
            fp.x.append(fx)
            fp.y.append(fy)

        # calc yaw and ds
        for i in range(len(fp.x) - 1):
            dx = fp.x[i + 1] - fp.x[i]
            dy = fp.y[i + 1] - fp.y[i]
            fp.yaw.append(math.atan2(dy, dx))
            fp.ds.append(math.sqrt(dx**2 + dy**2))

        fp.yaw.append(fp.yaw[-1])
        fp.ds.append(fp.ds[-1])

        # calc curvature
        for i in range(len(fp.yaw) - 1):
            fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

    return fplist


def check_collision(fp, ob):
  
    for i in range(len(ob[:, 0])):
        d = [((ix - ob[i, 0])**2 + (iy - ob[i, 1])**2)
             for (ix, iy) in zip(fp.x, fp.y)]
        collision = any([di <= ROBOT_RADIUS**2 for di in d])
        if collision:
            return False
    return True


def check_paths(fplist, ob):

    """
    check path above max speed, max a, does collision or not
    """
    okind = []
    for i in range(len(fplist)):
        if any([v > MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
            continue
        elif any([abs(a) > MAX_ACCEL for a in fplist[i].s_dd]):  # Max accel check
            continue
        elif any([abs(c) > MAX_CURVATURE for c in fplist[i].c]):  # Max curvature check
            continue
        elif not check_collision(fplist[i], ob):
            continue
        okind.append(i)
    return [fplist[i] for i in okind]


def frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob):
    ob = np.array(ob)
    fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0)
    fplist = calc_global_paths(fplist, csp)
    fplist = check_paths(fplist, ob)

    # find minimum cost path
    mincost = float("inf")
    bestpath = None
    for fp in fplist:
        if mincost >= fp.cf:
            mincost = fp.cf
            bestpath = fp
    return bestpath


def generate_road_widle(x,y):
    csp = Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)
    road_left_x, road_left_y, road_right_x, road_right_y = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        road_left_ix = ix + MAX_ROAD_WIDTH/2 * math.cos(csp.calc_yaw(i_s)+math.pi / 2.0)
        road_left_iy = iy + MAX_ROAD_WIDTH/2 * math.sin(csp.calc_yaw(i_s)+math.pi / 2.0)
        road_right_ix = ix - MAX_ROAD_WIDTH/2 * math.cos(csp.calc_yaw(i_s)+math.pi / 2.0)
        road_right_iy = iy - MAX_ROAD_WIDTH/2 * math.sin(csp.calc_yaw(i_s)+math.pi / 2.0)
        road_left_x.append(road_left_ix)
        road_left_y.append(road_left_iy)
        road_right_x.append(road_right_ix)
        road_right_y.append(road_right_iy)
    return road_left_x, road_left_y, road_right_x, road_right_y

def generate_target_course(x, y):
    csp = Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)    #0.1
    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))
    return rx, ry, ryaw, rk, csp


#######################################################################################
def load_global_path():
    global zero_cord_x,zero_cord_y
    bet = 0.1  
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
    
    zero_cord_x = GPS_x[0]
    zero_cord_y = GPS_y[0]
    GPS_x = GPS_x - zero_cord_x
    GPS_y = GPS_y - zero_cord_y
    plt.plot(GPS_x,GPS_y, "-r", label="GPS point ")
    plt.plot()
    plt.show()    

    return GPS_x, GPS_y

class Info(object):
    def __init__(self):
        self.CurrGPS_lat = float(-1)
        self.CurrGPS_lon = float(-1)
        self.CurrentVelocity = float(-1)
        self.Target_Velocity = float(-1)
        self.ImuYaw = float(-1)
        self.Target_Theta = float(-1)
        #self.CommandMessage = Car_Input()
        self.gob = np.array([])
        self.ob = np.array([])
        self.gobx = np.array([])
        self.goby = np.array([])

        # Subscribers

        rospy.Subscriber("coordinate", Point, self.FeedbackCallbackObs)
        sub = rospy.Subscriber('gnss_message', GNSS_CAN, self.FeedbackCallbackGPSIMU,queue_size = 10)                      #订阅GPS数据
        rospy.Subscriber("Motor_Feedback_mssage", Motor_Feedback,self.RVcallback,queue_size = 10)
        

    
    
    def FeedbackCallbackGPSIMU(self, msg): 
        self.CurrGPS_lat = msg.latitude 
        self.CurrGPS_lon = msg.longitude 
        self.ImuYaw = (90-msg.course_angle)*np.pi/180
        #print(self.CurrGPS_lat,self.CurrGPS_lon,self.ImuYaw)

    def FeedbackCallbackObs(self, msg):
        global Gob_x
        global Gob_y
        self.gobx = msg.x
        self.goby = msg.y
        #print("msg.x","msg.y", msg.x, msg.y)
        Gob_x.append(self.gobx)
        Gob_y.append(self.goby) 
        #print("Gob_x","Gob_y", Gob_x, Gob_y)
        #np.append(self.gobx,5)
        #np.append(self.goby,5)
        
        self.gob = np.column_stack((Gob_x, Gob_y))
        #print(self.gobx,self.goby)
        #print(self.gob)

    def RVcallback(self,msg):
    
        self.CurrentVelocity = msg.Base_Vehspd
        #print("*"*50)
        #print("rv:",rv)
        #rospy.loginfo('I heard: %s', data.data)


    def init(self):
        return self.CurrGPS_lat, self.CurrGPS_lon, self.ImuYaw, self.gobx, self.goby, self.gob, self.CurrentVelocity


    def talker(self,Target_Velocity, path_record):
        self.rate = rospy.Rate(100) # 10hz
        self.pub_Velocity = rospy.Publisher('Car_Velocity', Float32, queue_size = 10)           #定义Publisher对象
        # 定义发布器 path_pub 发布 trajectory
        self.path_pub = rospy.Publisher('trajectory', localPath, queue_size = 50)           #定义Publisher对象
        self.pub_Velocity.publish(Target_Velocity)
        # 发布路径
        self.path_pub.publish(path_record)
        #self.rate.sleep()



#    def talker(self,Target_Velocity,Target_Theta):
#        self.pub_Velocity = rospy.Publisher('Car_Velocity', Float32, queue_size = 10)           #定义Publisher对象
#        self.pub_Steering = rospy.Publisher('Car_Steering', Float32, queue_size = 10)
#        self.rate = rospy.Rate(100) # 10hz
#       self.pub_Velocity.publish(Target_Velocity)
#        self.pub_Steering.publish(Target_Theta)
#        self.rate.sleep()






#######################################################################################
def get_transalation(curr_gps_lat,curr_gps_lon):
    curr_posy=(float(curr_gps_lon)-zero_cord_y)
    curr_posx=(float(curr_gps_lat)-zero_cord_x)
    #print("curr_posy,curr_posx=",curr_posy,curr_posx)
    return curr_posx, curr_posy



def get_transformation(pt,curr_yaw,T):
    c, s = np.cos(curr_yaw), np.sin(curr_yaw)
    R = (np.array(((c,-s), (s, c))))
    pt=pt.dot(R)+T
    return pt



def get_arc_length(tx,ty,st):
    arc_length=0
    for x in range(1,st):
        arc_length=arc_length+(np.hypot((tx[x-1]-tx[x]),(ty[x-1]-ty[x])))
    return arc_length



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



def proportional_control(target, current):
    #print("*"*50)
    #print("current=",current)
    #print("target - current",target - current)
    a = 1.0 * (target - current)

    return a






def main():

    ptx = []
    pty = []

    ptx, pty = load_global_path()
    tx, ty, tyaw, tc, csp = generate_target_course(ptx, pty)
    #print(csp)
    road_left_x, road_left_y, road_right_x, road_right_y = generate_road_widle(ptx, pty)
    
    #当前车速及加速度
    c_speed = 5.0/3.6
    c_acc = 1.0
    c_d_dd = 0
    c_d_d = 0
    area = 25.0  # animation area length [m]
    start = time.time()
    rospy.init_node('AvoidObstacles_PlannerOut', anonymous = False)
    my_node = Info()
    
    
    while not rospy.is_shutdown():
        CurrGPS_lat, CurrGPS_lon, ImuYaw, gobx, goby, gob, CurrentVelocity = my_node.init()
        #print("gob",gob)
        ob = []
        
        if (CurrGPS_lat != -1 and CurrGPS_lon != -1 and ImuYaw != -1):
            
            

            
            
            
                #print(CurrGPS_lat,CurrGPS_lon,ImuYaw, curr_posx, curr_posy)
                #print(gobx,goby,gob)
            #path = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob)
            #s0 = path.s[1]
            #c_d = path.d[1]
            #c_d_d = path.d_d[1]
            #c_d_dd = path.d_dd[1]
            #c_speed = path.s_d[1]
            
            curr_posx, curr_posy = get_transalation(CurrGPS_lat, CurrGPS_lon)
            T = [curr_posx, curr_posy]
            
            
            
            
            curr_yaw = ImuYaw #+ math.pi / 2
            
            
            if (len(gob) == 0):
                ob = [[-20, -20]]
            
            else:
                ob = gob
            
            
            ob_len = len(ob)-1
            for x in xrange(0, ob_len):
                #print("ob_transformation",ob)
                ob = np.array(ob)
                #ob[x, :] = .2 * ob[x, :]
                ob[x, :] = get_transformation(ob[x, :], -curr_yaw, T)
            #print("ob_transformation",ob)
                #############################################################
            
            
            
            
            
            # c_d_dd = c_acc*math.cos(math.atan2((ty[spt]-curr_posy),(tx[spt]-curr_posx))+curr_yaw)
            
            
            #spt, c_d, curr_posx, curr_posy = get_lateral_dist(tx, ty, curr_posx, curr_posy)
            
            #curr_posx, curr_posy = get_transalation(CurrGPS_lat, CurrGPS_lon)
            
            
            
            try:
                curr_posx, curr_posy = get_transalation(CurrGPS_lat, CurrGPS_lon)
                spt, c_d, curr_posx, curr_posy = get_lateral_dist(tx, ty, curr_posx, curr_posy)
                s0 = get_arc_length(tx, ty, spt)
                path = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob)
                c_speed = path.s_d[1] 
                #c_d_d = c_speed*math.cos(math.atan2((ty[spt]-curr_posy),(tx[spt]-curr_posx))-curr_yaw)
                c_d_d = path.d_d[1] 
                c_d_dd = path.d_dd[1] 
                
                if np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 1.0:
                    print("Goal")
                    c_speed = 0.0
                    break
                if show_animation:
                    plt.cla()
                    plt.plot(tx, ty, "-.k")
                    plt.plot(road_left_x, road_left_y, "-k")
                    plt.plot(road_right_x, road_right_y, "-k")
                    plt.plot(ob[:, 0], ob[:, 1], "ob")
                    plt.plot(path.x[1:], path.y[1:], "-or")
                    plt.plot(path.x[1], path.y[1], "vc")
                    plt.xlim(path.x[1] - area, path.x[1] + area)
                    plt.ylim(path.y[1] - area, path.y[1] + area)
                    plt.arrow(curr_posx, curr_posy, math.cos(curr_yaw), math.sin(curr_yaw),fc="r", ec="k", head_width=0.5, head_length=1.0)
                    plt.title("v[km/h]:" + str(c_speed)[0:4])
                    plt.xlabel(u'x/m', fontsize=14)  # 设置x轴，并设定字号大小
                    plt.ylabel(u'y/m', fontsize=14)  # 设置y轴，并设定字号大小
                    plt.pause(0.0001)
                
                
                
                ####################规划成功###############                
                ###########################################
                PathFail_flag = 0 
                ###########################################
                
                
            except:
                ###############规划失败################
                PathFail_flag = 1
                print("Don't find optimal path")
            
            ################对障碍物堆栈清空############
            ############################################
            ############################################
            global Gob_x
            global Gob_y
            Gob_x*=0
            Gob_y*=0        
            ############################################
            ############################################
            
            
            
###############################################################################            
               
                
            try:
                '''
                acc = proportional_control(6, CurrentVelocity)
                temp1=path.yaw[1]     `
                temp2=curr_yaw 
                
                if temp1<0:
                    temp1=6.28+temp1
                if temp2<0:
                    temp2=6.28+temp2

                val = temp1-temp2
                
                if val > 3.14:
                    val = val - 6.28
                if val < -3.14:
                    val = val + 6.28
                
                val = math.degrees(val)
                
                if val > 50:
                    val = 50
                if val < -50:
                    val = -50
                
                my_node.talker(acc,val)
                '''
                path_record = localPath()

                # 配置路径
                for i in range(len(path.x[1:])):

                    #print("path_x",path.x[i])
                    
                    path_record.path_x.append(path.x[i])
                    path_record.path_y.append(path.y[i])   
                # 路径数量限制
                if len(path_record.path_x) > 10000:
                    path_record.path_x.pop(0)
                    path_record.path_y.pop(0)
                # 发布路径`
                my_node.talker(c_speed, path_record)
                
            except: 
                print("local path send fail")
                pass
                #my_node.talker(c_speed, path.x[1:], path.y[1:])
            #except:
            #    pass

    print("Finish")
    end = time.time()
    #print("total time: ", end - start)

    if show_animation:
        plt.grid(True)
        plt.show()



if __name__ == "__main__":
    main()
