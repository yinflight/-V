#!/usr/bin/python3
# coding: utf-8
#!/usr/bin/env python

"""

Path tracking simulation with iterative linear model predictive control for speed and steer control
定速规划
未添加速度规划
try：
except：
finally：
未正确return di ai


"""

import rospy 
import time
import cvxpy
import math
import bisect
import numpy as np
import scipy
from scipy.signal import savgol_filter
import warnings
warnings.filterwarnings("ignore",".*GUI is implemented.*")
import matplotlib.pyplot as plt
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from control.msg import Motor_Feedback

import sys
sys.path.append("/home/robot/Robot/SmartCar_WS/src/fuzzy_purepursuit/scripts")

try:
    import cubic_spline_planner
except:
    raise

sx, sy, yaw, sv = 0.0, 0.0, 0.0, 0.0 # [m] coordinate and heading of tracking points
zero_coord_x = 0.0
zero_coord_y = 0.0
bet = 0.2 # [m] dis of tracking points

NX = 4  # x = x, y, v, yaw，状态量
NU = 2  # a = [accel, steer]，控制量
T = 5  # horizon length，预测长度

# mpc parameters
R = np.diag([0.01, 0.01])  # input cost matrix
Rd = np.diag([0.01, 1.0])  # input difference cost matrix
Q = np.diag([1.0, 1.0, 0.5, 0.5])  # state cost matrix
Qf = Q  # state final matrix
GOAL_DIS = 1.5  # goal distance
STOP_SPEED = 0.5 / 3.6  # stop speed 0.13888
MAX_TIME = 500.0  # max simulation time

# iterative paramter
MAX_ITER = 3  # Max iteration
DU_TH = 0.1  # iteration finish param

TARGET_SPEED = 10.0 / 3.6  # [m/s] target speed 2.7777
N_IND_SEARCH = 10  # Search index number

DT = 0.2  # [s] time tick

# Vehicle parameters
LENGTH = 4.5  # [m]
WIDTH = 2.0  # [m]
BACKTOWHEEL = 1.0  # [m]
WHEEL_LEN = 0.3  # [m]
WHEEL_WIDTH = 0.2  # [m]
TREAD = 0.7  # [m]
WB = 2.5  # [m] 轴距

MAX_STEER = np.deg2rad(45.0)  # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(30.0)  # maximum steering speed [rad/s]
MAX_SPEED = 55.0 / 3.6  # maximum speed [m/s]
MIN_SPEED = -20.0 / 3.6  # minimum speed [m/s]
MAX_ACCEL = 1.0  # maximum accel [m/ss]

show_animation = True


class State:
    """
    vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None


def pi_2_pi(angle):
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle


def get_linear_model_matrix(v, phi, delta):

    A = np.zeros((NX, NX))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[3, 3] = 1.0
    A[0, 2] = DT * math.cos(phi)
    A[0, 3] = - DT * v * math.sin(phi)
    A[1, 2] = DT * math.sin(phi)
    A[1, 3] = DT * v * math.cos(phi)
    A[3, 2] = DT * math.tan(delta) / WB

    B = np.zeros((NX, NU))
    B[2, 0] = DT
    B[3, 1] = DT * v / (WB * math.cos(delta) ** 2)

    C = np.zeros(NX)
    C[0] = DT * v * math.sin(phi) * phi
    C[1] = - DT * v * math.cos(phi) * phi
    C[3] = - DT * v * delta / (WB * math.cos(delta) ** 2)

    return A, B, C


def plot_car(x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover

    outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                        [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(x, y, "*")


def update_state(state, a, delta):

    # input check
    if delta >= MAX_STEER:
        delta = MAX_STEER
    elif delta <= -MAX_STEER:
        delta = -MAX_STEER

    try:
        state.x = float(sx)
        state.y = float(sy)
        state.yaw = float(yaw)
        state.v = float(sv)
    except:
        print("error,error,error")
    state.yaw = 90.0 - float(yaw)

    # state.x = state.x + state.v * math.cos(state.yaw) * DT
    # state.y = state.y + state.v * math.sin(state.yaw) * DT
    # state.yaw = state.yaw + state.v / WB * math.tan(delta) * DT
    # state.v = state.v + a * DT

    if state. v > MAX_SPEED:
        state.v = MAX_SPEED
    elif state. v < MIN_SPEED:
        state.v = MIN_SPEED

    return state


def get_nparray_from_matrix(x):
    return np.array(x).flatten()


def calc_nearest_index(state, cx, cy, cyaw, pind):

    dx = [state.x - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind) + pind

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


def predict_motion(x0, oa, od, xref):
    xbar = xref * 0.0
    for i, _ in enumerate(x0):
        xbar[i, 0] = x0[i]

    state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
    for (ai, di, i) in zip(oa, od, range(1, T + 1)):
        state = update_state(state, ai, di)
        '''
        state = update_state2(state, ai, di)
        '''
        xbar[0, i] = state.x
        xbar[1, i] = state.y
        xbar[2, i] = state.v
        xbar[3, i] = state.yaw

    return xbar


def iterative_linear_mpc_control(xref, x0, dref, oa, od):
    """
    MPC contorl with updating operational point iteraitvely
    """

    if oa is None or od is None:
        oa = [0.0] * T
        od = [0.0] * T

    for i in range(MAX_ITER):
        xbar = predict_motion(x0, oa, od, xref)
        poa, pod = oa[:], od[:]
        oa, od, ox, oy, oyaw, ov = linear_mpc_control(xref, xbar, x0, dref)
        du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
        print("du:",du)
        if du <= DU_TH:
            break
    else:
        print("Iterative is max iter")
    print("ox:", ox)
    print("oy:", oy)
    print("ov:", ov)
    print("ooyaw:", oyaw)
    print("oa:", oa)
    print("odelta:", od)

    return oa, od, ox, oy, oyaw, ov


def linear_mpc_control(xref, xbar, x0, dref):
    """
    linear mpc control

    xref: reference point
    xbar: operational point
    x0: initial state
    dref: reference steer angle
    """

    x = cvxpy.Variable((NX, T + 1))
    u = cvxpy.Variable((NU, T))


    cost = 0.0
    constraints = []

    for t in range(T):
        cost += cvxpy.quad_form(u[:, t], R)
        # print("t:",t)
        # print("-------------------")
        # print("t:",  u)
        # print("-------------------")
        # print("cost1:", cost)
        # print("u[:,t]:",u[:,t])

        if t != 0:
            cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)
            # print("-------------------")
            # print("cost2:", cost)
        A, B, C = get_linear_model_matrix(
            xbar[2, t], xbar[3, t], dref[0, t])
        constraints += [x[:, t + 1] == A * x[:, t] + B * u[:, t] + C]

        if t < (T - 1):
            cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
            # print("-------------------")
            # print("cost3:", cost)
            constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <=
                            MAX_DSTEER * DT]

    cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf)
    print("-------------------")
    #print("cost4:", cost)
    #print("cost:",cost)

    constraints += [x[:, 0] == x0]
    constraints += [x[2, :] <= MAX_SPEED]
    constraints += [x[2, :] >= MIN_SPEED]
    constraints += [cvxpy.abs(u[0, :]) <= MAX_ACCEL]
    constraints += [cvxpy.abs(u[1, :]) <= MAX_STEER]

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.ECOS, verbose=False)

    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        ox = get_nparray_from_matrix(x.value[0, :])

        oy = get_nparray_from_matrix(x.value[1, :])

        ov = get_nparray_from_matrix(x.value[2, :])

        oyaw = get_nparray_from_matrix(x.value[3, :])

        oa = get_nparray_from_matrix(u.value[0, :])

        odelta = get_nparray_from_matrix(u.value[1, :])


    else:
        print("Error: Cannot solve mpc..")
        oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

    return oa, odelta, ox, oy, oyaw, ov


def calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, pind):
    xref = np.zeros((NX, T + 1))
    dref = np.zeros((1, T + 1))
    ncourse = len(cx)

    ind, _ = calc_nearest_index(state, cx, cy, cyaw, pind)

    if pind >= ind:
        ind = pind

    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    xref[2, 0] = sp[ind]
    xref[3, 0] = cyaw[ind]
    dref[0, 0] = 0.0  # steer operational point should be 0

    travel = 0.0

    for i in range(T + 1):
        travel += abs(state.v) * DT
        dind = int(round(travel / dl))

        if (ind + dind) < ncourse:
            xref[0, i] = cx[ind + dind]
            xref[1, i] = cy[ind + dind]
            xref[2, i] = sp[ind + dind]
            xref[3, i] = cyaw[ind + dind]
            dref[0, i] = 0.0
        else:
            xref[0, i] = cx[ncourse - 1]
            xref[1, i] = cy[ncourse - 1]
            xref[2, i] = sp[ncourse - 1]
            xref[3, i] = cyaw[ncourse - 1]
            dref[0, i] = 0.0

    return xref, ind, dref


def check_goal(state, goal, tind, nind):

    # check goal
    dx = state.x - goal[0]
    dy = state.y - goal[1]
    d = math.sqrt(dx ** 2 + dy ** 2)

    isgoal = (d <= GOAL_DIS)

    if abs(tind - nind) >= 5:
        isgoal = False

    isstop = (abs(state.v) <= STOP_SPEED)

    if isgoal and isstop:
        return True

    return False


def do_simulation(cx, cy, cyaw, ck, sp, dl, initial_state):
    """
    Simulation

    cx: course x position list
    cy: course y position list
    cy: course yaw position list
    ck: course curvature list
    sp: speed profile
    dl: course tick [m]

    """

    goal = [cx[-1], cy[-1]]

    state = initial_state

    # initial yaw compensation
    if state.yaw - cyaw[0] >= math.pi:
        state.yaw -= math.pi * 2.0
    elif state.yaw - cyaw[0] <= -math.pi:
        state.yaw += math.pi * 2.0

    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    d = [0.0]
    a = [0.0]
    target_ind, _ = calc_nearest_index(state, cx, cy, cyaw, 0)

    odelta, oa = None, None

    cyaw = smooth_yaw(cyaw)

    while MAX_TIME >= time:
        xref, target_ind, dref = calc_ref_trajectory(
            state, cx, cy, cyaw, ck, sp, dl, target_ind)

        x0 = [state.x, state.y, state.v, state.yaw]  # current state

        oa, odelta, ox, oy, oyaw, ov = iterative_linear_mpc_control(
            xref, x0, dref, oa, odelta)

        #print("oa:",oa,odelta)

        if odelta is not None:
            di, ai = odelta[0], oa[0]

        '''   
        try:
            s
        except:
            state
        finally:
        
        '''

        #return di, ai
        #print("ai:",ai,di)

        state = update_state(state, ai, di)


        time = time + DT

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        d.append(di)
        a.append(ai)

        '''
        simple_publisher.publish(di)
        sec_publisher.publish(ai)
        # 发布Topic
        rate.sleep()
        '''

        if check_goal(state, goal, target_ind, len(cx)):
            print("Goal")
            break

        if show_animation:  # pragma: no cover
            plt.cla()
            if ox is not None:
                plt.plot(ox, oy, "xr", label="MPC")
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "ob", label="trajectory")
            plt.plot(xref[0, :], xref[1, :], "xk", label="xref")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plot_car(state.x, state.y, state.yaw, steer=di)
            plt.axis("equal")
            plt.grid(True)
            plt.title("Time[s]:" + str(round(time, 2))
                      + ", speed[km/h]:" + str(round(state.v * 3.6, 2)))
            plt.pause(0.0001)

    return t, x, y, yaw, v, d, a


def calc_speed_profile(cx, cy, cyaw, target_speed):

    speed_profile = [target_speed] * len(cx)
    direction = 1.0  # forward

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]

        move_direction = math.atan2(dy, dx)

        if dx != 0.0 and dy != 0.0:
            dangle = abs(pi_2_pi(move_direction - cyaw[i]))
            if dangle >= math.pi / 4.0:
                direction = -1.0
            else:
                direction = 1.0

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

    speed_profile[-1] = 0.0

    return speed_profile


def smooth_yaw(yaw):

    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]

        while dyaw >= math.pi / 2.0:
            yaw[i + 1] -= math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

        while dyaw <= -math.pi / 2.0:
            yaw[i + 1] += math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

    return yaw


def get_straight_course(dl):
    ax = [0.0, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    ay = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck

def get_straight_course2(dl):
    ax = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
    ay = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck

def get_straight_course3(dl):
    ax = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
    ay = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    cyaw = [i - math.pi for i in cyaw]

    return cx, cy, cyaw, ck

def get_straight_course4(dl):
    '''
    换道车辆曲线
    :param dl:
    :return:
    '''
    ax = [0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0, 100.0 ]
    ay = [0.0, 0.10, 0.63, 1.50, 2.55, 3.35, 3.71, 3.75]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    cyaw = [i - math.pi for i in cyaw]

    return cx, cy, cyaw, ck

def get_straight_course5(dl):
    '''
    换道车辆曲线
    :param dl:
    :return:
    '''
    ax = [0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0, 100.0 ]
    ay = [0.0, 0.10, 0.63, 1.50, 2.55, 3.35, 3.71, 3.75]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    '''
    time, cx, cy, cyaw, sp, a, j, cK = 
    self_quintic_polynomials_planner.quintic_polynomials_planner(
            sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga, max_accel, max_jerk, dt)
    '''

    cyaw = [i - math.pi for i in cyaw]

    #return cx, cy, cyaw, ck, sp
    return cx, cy, cyaw, ck

def get_forward_course(dl):
    ax = [0.0, 60.0, 125.0, 50.0, 75.0, 30.0, -10.0]
    ay = [0.0, 0.0, 50.0, 65.0, 30.0, 50.0, -20.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck

def get_switch_back_course(dl):
    ax = [0.0, 30.0, 6.0, 20.0, 35.0]
    ay = [0.0, 0.0, 20.0, 35.0, 20.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    ax = [35.0, 10.0, 0.0, 0.0]
    ay = [20.0, 30.0, 5.0, 0.0]

    cx2, cy2, cyaw2, ck2, s2 = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    cyaw2 = [i - math.pi for i in cyaw2]
    cx.extend(cx2)
    cy.extend(cy2)
    cyaw.extend(cyaw2)
    ck.extend(ck2)


    return cx, cy, cyaw, ck



def Remove_Duplicates():

    blank = []
    white = []
    yellow = []
    GPS_x = []
    GPS_y = []
    nums, ber = np.loadtxt("/home/robot/Robot/SmartCar_WS/src/fuzzy_purepursuit/scripts/curve.txt", dtype=str, delimiter=',',
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
            GPS_x.append(yellow[i])
            # print(type(cx))
            GPS_y.append(white[i])
            bx = yellow[i]
            by = white[i]
            # print(dis)

    GPS_x = np.array(GPS_x)
    GPS_y = np.array(GPS_y)
    #print(type(cy))
    #print(cy[2])
    global zero_coord_x,zero_coord_y
    zero_coord_x = GPS_x[0]
    zero_coord_y = GPS_y[0]
    GPS_x = GPS_x - GPS_x[0]
    GPS_y = GPS_y - GPS_y[0]
    #cy = scipy.signal.savgol_filter(cy, 63, 3)
    #print(len(cx))
    
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
            GPS_x, GPS_y, ds=0.2) #tracking points、coordinate、heading、K
    return cx, cy, cyaw, ck





def RVcallback(Motor_Feedback):
    global rv
    rv = Motor_Feedback.Base_Vehspd

def Statuscallback(msg):
    global sx, sy, yaw
    sx, sy, yaw = msg.data.split(',')

    # state = update(state, ai, di,msg)
    # return sx,sy,yaw

def main():
    print(__file__ + " start!!")

    dl = 1.0  # course tick
    # cx, cy, cyaw, ck = get_straight_course(dl) #直线
    # cx, cy, cyaw, ck = get_straight_course2(dl) #正弦
    # cx, cy, cyaw, ck = get_straight_course3(dl) #
    #cx, cy, cyaw, ck = get_straight_course4(dl)  # 换道曲线
    # cx, cy, cyaw, ck = get_forward_course(dl) #复杂曲线1
    # cx, cy, cyaw, ck = get_switch_back_course(dl) #复杂曲线2包括倒车
    # cx, cy, cyaw, ck, cv = get_straight_course5(dl)  # 换道曲线改进（ 五次多项式）
    cx, cy, cyaw, ck = Remove_Duplicates() # GPS循线
    

    # print("cx:", cx)
    # print("cy:", cy)
    print("len(cx):", len(cx))

    sp = calc_speed_profile(cx, cy, cyaw, TARGET_SPEED)
    # print("sp:", sp)
    print("len(sp):", len(sp))

    initial_state = State(x=cx[0], y=cy[0], yaw=cyaw[0], v=0.0)

    rospy.init_node('MPC_controller_node', anonymous=False)  # 初始化ROS节点
    simple_publisher = rospy.Publisher('delta', Float32, queue_size=10)  # 定义delta_Publisher对象
    second_publisher = rospy.Publisher('c_a', Float32, queue_size=10)  # 定义ia_Publisher对象
    #simple_publisher = rospy.Publisher('delta', String, queue_size=10)
    rate = rospy.Rate(10)  # 设置Topic发布的频率（Hz）

    while not rospy.is_shutdown():
        sub = rospy.Subscriber('chatter', String, Statuscallback,  queue_size=10)  # 订阅GPS数据
        rospy.Subscriber("Motor_Feedback_mssage", Motor_Feedback, RVcallback, queue_size=10) # 订阅车速
        initial_state = State(x=cx[0], y=cy[0], yaw=cyaw[0], v=0.0)

        #t, x, y, yaw, v, d, a = do_simulation(
        #    cx, cy, cyaw, ck, sp, dl, initial_s tate)

        goal = [cx[-1], cy[-1]]

        state = initial_state

        # initial yaw compensation
        if state.yaw - cyaw[0] >= math.pi:
            state.yaw -= math.pi * 2.0
        elif state.yaw - cyaw[0] <= -math.pi:
            state.yaw += math.pi * 2.0

        time = 0.0
        x = [state.x]
        y = [state.y]
        yaw = [state.yaw]
        v = [state.v]
        t = [0.0]
        d = [0.0]
        a = [0.0]
        target_ind, _ = calc_nearest_index(state, cx, cy, cyaw, 0)

        odelta, oa = None, None

        cyaw = smooth_yaw(cyaw)

        while MAX_TIME >= time:
            xref, target_ind, dref = calc_ref_trajectory(
                state, cx, cy, cyaw, ck, sp, dl, target_ind)

            x0 = [state.x, state.y, state.v, state.yaw]  # current state

            oa, odelta, ox, oy, oyaw, ov = iterative_linear_mpc_control(
                xref, x0, dref, oa, odelta)

            # print("oa:",oa,odelta)

            if odelta is not None:
                di, ai = odelta[0], oa[0]

            # print("ai:",ai,di)

            state = update_state(state, ai, di)

            time = time + DT

            x.append(state.x)
            y.append(state.y)
            yaw.append(state.yaw)
            v.append(state.v)
            t.append(time)
            d.append(di)
            a.append(ai)

            simple_publisher.publish(di)
            second_publisher.publish(ai)
            # 发布Topic
            rate.sleep()

            if check_goal(state, goal, target_ind, len(cx)):
                print("Goal")
                break

            if show_animation:  # pragma: no cover
                plt.cla()
                if ox is not None:
                    plt.plot(ox, oy, "xr", label="MPC")
                plt.plot(cx, cy, "-r", label="course")
                plt.plot(x, y, "ob", label="trajectory")
                plt.plot(xref[0, :], xref[1, :], "xk", label="xref")
                plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
                plot_car(state.x, state.y, state.yaw, steer=di)
                plt.axis("equal")
                plt.grid(True)
                plt.title("Time[s]:" + str(round(time, 2))
                          + ", speed[km/h]:" + str(round(state.v * 3.6, 2)))
                plt.pause(0.0001)

        if show_animation:  # pragma: no cover
            plt.close("all")
            plt.subplots()
            plt.plot(cx, cy, "-r", label="spline")
            plt.plot(x, y, "-g", label="tracking")
            plt.grid(True)
            plt.axis("equal")
            plt.xlabel("x[m]")
            plt.ylabel("y[m]")
            plt.legend()

            plt.subplots()
            plt.plot(t, v, "-r", label="speed")
            plt.grid(True)
            plt.xlabel("Time [s]")
            plt.ylabel("Speed [kmh]")

            plt.show()

if __name__ == '__main__':
    print("MPC path tracking start !!!")
    main()

