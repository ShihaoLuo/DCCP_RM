# coding: utf-8
import cv2

import cvxpy as cp
import time
import dccp
#import dccp
import numpy as np
# import matplotlib.pyplot as plt
import math
from scipy import sparse
import scipy.interpolate as interpolate
from pose_estimater.pose_estimater import PoseEstimater
#from robomaster import robot
import multiprocessing
import sys
import websocket
import json

def get_pose(Estimater, _img, caculate_pose):
    while True:
        tmp = caculate_pose
        img = _img
        # print('in get pose :', img.shape)
        pose = Estimater.estimate_pose(img, tmp)
        # print('in get pose:', pose)
        if pose[0] is not None:
            return pose
        else:
            return None

def DCCP_planning(listx, listy, r, startpoint, end_point):
    time_start=time.time()
    #l = 10
    n = 25
    a = np.matrix(startpoint).T
    b = np.matrix(end_point).T
    d = 2
    # listx是obstacle的x轴，listy是obstacle的y轴，r是半径
    # listx = [16.3,14.7,15.5,16.3,17.3,18,18.5,15.5,15.5,16.3,17.65,17.65,17.65,17.65]
    # listy = [8.3,6.7,10.3,6.3,10,6.8,8.7,0.5,5,2.5,3.65,2.95,2.25,1.55]
    # r = [1.4,0.3,0.3,0.3,0.3,0.3,0.3,0.5,0.5,0.7,0.35,0.35,0.35,0.35]
    p = np.matrix([listx,listy])
    m = len(listx)
    x = []
    for i in range(n+1):
        x += [cp.Variable((d,1))]
    L = cp.Variable(1)
    constr = [x[0] == a, x[n] == b]
    # constr += [x>=14]
    cost = L
    for i in range(n):
        constr += [cp.norm(x[i]-x[i+1]) <= L/n]
        for j in range(m):
            constr += [cp.norm(x[i]-p[:,j]) >= r[j]+0.8]
            #rectangular
            # constr += [abs(x[i][0]-2.5)+abs(x[i][1]-11 >= 2+0.1)]
    prob = cp.Problem(cp.Minimize(cost), constr)
    print("begin to solve")
    result = prob.solve(method='dccp')
    print("end")
    dccp_path = np.array([])
    for xx in x:
        dccp_path = np.append(dccp_path, xx.value)
    dccp_path = dccp_path.reshape(math.floor((len(dccp_path)/2)),2).T
    print(dccp_path)
    time_end=time.time()
    print("time =", time_end - time_start)
    return(dccp_path)

def process_waypoints(dccp_path):
    cvxp = dccp_path
    cvxd = np.zeros(len(cvxp[0]))
    for i in range(len(cvxp[0])-1):
        v0 = [cvxp[0][i+1] - cvxp[0][i], cvxp[1][i+1]-cvxp[1][i]]
        ctheta = (v0[0])/np.linalg.norm(v0)
        # print(ctheta)
        cvxd[i+1] = np.arccos(ctheta)
        # print(cvxd[i+1])
    cvxp = np.insert(cvxp, 0, initpose[0:2], axis=1)
    cvxp_x = cvxp[0, :]
    cvxp_y = cvxp[1, :]
    cvxd = np.append([0], cvxd)
    for i in range(12):
        cvxp_x = cvxp_x.repeat(2)[:-1]
        cvxp_y = cvxp_y.repeat(2)[:-1]
        cvxd = cvxd.repeat(2)[:-1]
        cvxp_x[1::2] += (cvxp_x[2::2] - cvxp_x[1::2]) / 2
        cvxp_y[1::2] += (cvxp_y[2::2] - cvxp_y[1::2]) / 2
        cvxd[1::2] += (cvxd[2::2] - cvxd[1::2]) / 2
    distance = np.sqrt(np.ediff1d(cvxp_x) ** 2 + np.ediff1d(cvxp_y) ** 2)
    d = 0.0
    equdist_waypoint = np.array([[cvxp_x[0]], [cvxp_y[0]], [cvxd[0]]])
    for i in range(len(distance)):
        d = distance[i] + d
        if d >= resolution:
            equdist_waypoint = np.append(equdist_waypoint, [[cvxp_x[i]], [cvxp_y[i]], [cvxd[i]]], axis=1)
            d = 0.0
    equdist_waypoint = np.append(equdist_waypoint, [[cvxp_x[-1]], [cvxp_y[-1]], [cvxd[-1]]], axis=1)
    tVec2 = np.linspace(0, len(equdist_waypoint[0, :]), num=int(len(equdist_waypoint[0, :])))
    tVec = np.linspace(0, len(equdist_waypoint[0, :]),
                       num=int(len(equdist_waypoint[0, :]) * (resolution / (DesireSpeed * Ts))))
    f = interpolate.interp1d(tVec2, equdist_waypoint, kind='cubic')
    ref = f(tVec)
    # plot_figure(ref)
    print(ref)
    return ref

# def circle_draw(x0,y0,r,ax):
#     circ = np.linspace(0,2*math.pi,50)
#     x = []
#     y = []
#     for i in circ:
#         x.append(x0 + r*math.cos(i))
#         y.append(y0 + r*math.sin(i))
#     ax.plot(x, y,'b')

# def plot_figure(ref):
#     fig2, ax2 = plt.subplots(1,1,figsize = (30,30))
#     # ax2[0].plot(cvxp[0,:], cvxp[1,:], 'bo',markersize = 10,label = "Original dccp path points")
#     # ax2[0].plot(equdist_waypoint[0,:], equdist_waypoint[1,:], 'ro',label = "Equal distance path points")
#     ax2.plot(ref[0,:], ref[1,:], 'go',markersize = 1,label = "Sampling time path ponints")
#     # ax2[0].plot(path[0,:], path[1,:], 'g-',markersize = 1,label = "Mpc controlling path")
#     for i in range(len(listx)):
#         circle_draw(listx[i],listy[i],r[i],ax2)
#     ax2.legend(fontsize= 20)
#     plt.show()

def cal_M(theta,Ts):
    M = np.zeros((3,3))
    M[0,0] = np.cos(theta)*Ts
    M[0,1] = np.sin(theta)*Ts
    M[1,0] = np.sin(theta)*Ts
    M[1,1] = np.cos(theta)*Ts
    M[2,2] = Ts
    return M

def run(ws,ref):
    Estimater = PoseEstimater(min_match=20)
    Estimater.loaddata('./pose_estimater/dataset/')
    cap = cv2.VideoCapture(2)
    cap.set(3, 1280)
    cap.set(4, 960)
    cmd = {"op": "publish",
           "id": "1",
           "topic": "/cmd_vel",
           "msg": {

               "linear": {"x": 0.0, "y": 0.0, "z": 0.0},

               "angular": {"x": 0.0, "y": 0.0, "z": 0.0}

           }}
    cmd1 = json.dumps(cmd)
    # weighting matrix function
    Q = sparse.diags([1., 1., 1.])
    QN = Q * 10
    R = 0.1 * sparse.eye(3)
    # Define problem
    u = cp.Variable((3, N))
    x = cp.Variable((3, N + 1))
    x_init = cp.Parameter(3)
    M = cp.Parameter((3, 3))
    xr = cp.Parameter((3, N + 1))
    objective = 0
    constraints = [x[:, 0] == x_init]
    for k in range(N):
        Q = Q * N * 0.5
        objective += cp.quad_form(x[:, k] - xr[:, k], Q) + cp.quad_form(u[:, k], R)
        constraints += [x[:, k + 1] == x[:, k] + np.dot(M, u[:, k])]
        constraints += [umin <= u[:, k], u[:, k] <= umax]
        if k > 0:
            constraints += [dumin * Ts <= u[:, k] - u[:, k - 1], u[:, k] - u[:, k - 1] <= dumax * Ts]
    objective += cp.quad_form(x[:, N] - xr[:, N], QN)
    prob = cp.Problem(cp.Minimize(objective), constraints)
    # chassis.drive_speed(x=0, y=0, z=0, timeout=5)
    ws.send(cmd1)
    time.sleep(3)
    last_ref = ref[:, -1]
    for i in range(N - 1):
        last_ref = np.append(last_ref, ref[:, -1])
    last_ref = last_ref.reshape((N, 3)).T
    fianlusingref = np.append(ref, last_ref, axis=1)
    nsim = len(ref[0, :])
    path = np.array([])
    com = np.array([])
    pose = initpose
    path = np.append(path, pose)
    for i in range(nsim):
        ret, _img = cap.read()
        start_time = time.time()
        x_init.value = pose
        M.value = cal_M(pose[2], Ts)
        xr.value = fianlusingref[:, i:i + N + 1]
        prob.solve(solver=cp.OSQP, warm_start=True)
        pose = pose + np.dot(M.value, (u[:, 0].value))
        print('caculate:', pose)
        # print(pose)
        PNP_pose = get_pose(Estimater, _img, pose*100)
        if PNP_pose[0] is not None:
            # print(PNP_pose[0]/100)
            pose[0] = PNP_pose[0][0]/100
            pose[1] = PNP_pose[0][1]/100
            pose[2] = PNP_pose[1]/180*3.1416
            print('before:',pose)
            pose = pose + np.dot(M.value, (u[:, 0].value))*3
            # pose[2] = PNP_pose[1]/180*3.1416
            if pose[2] > 3.14159:
                pose[2] = pose[2] - 6.2832
            if pose[2] < -3.14159:
                pose[2] = pose[2] + 6.2832
            print('after:',pose)
        path = np.append(path, pose)
        com = np.append(com, u[:, 0].value)
        end_time = time.time()
        wait_t = Ts - (end_time - start_time)
        if wait_t >= 0:
            time.sleep(wait_t)
        else:
            print(wait_t)
        command1 = u[:, 0].value
        command1[2] = command1[2]
        print('command:', command1)
        # chassis.drive_speed(x=command1[0], y=command1[1], z=command1[2]*0.95, timeout=5)
        cmd["msg"]["linear"]["x"] = command1[0]
        cmd["msg"]["linear"]["y"] = command1[1]
        cmd["msg"]["angular"]["z"] = command1[2]
        cmd1 = json.dumps(cmd)
        # setter.set(command1[0], command1[1],command1[2])
        ws.send(cmd1)
    time.sleep(Ts)
    cmd["msg"]["linear"]["x"] = 0
    cmd["msg"]["linear"]["y"] = 0
    cmd["msg"]["angular"]["z"] = 0
    cmd1 = json.dumps(cmd)
    ws.send(cmd1)
    # chassis.drive_speed(x=0, y=0, z=0, timeout=5)
    # time.sleep(1)
    # chassis.move(0, 0, 0)
    # time.sleep(1)
    # robot.close()
    # while True:
    #     img = camera.read_cv2_image()
    #     old = time.time()
    #     pose = get_pose(img, [0, 0, 0])
    #     print(time.time() - old)
    #     print(pose)
    #     time.sleep(1)

if __name__ == '__main__':
    ws = websocket.WebSocket()
    ws.connect("ws://192.168.1.174:9090")
    # parameters
    # listx = np.array([0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7, 7.5, 8, 8.5, 9, 9.5, 10, 10.5, 11, 11, 11, 11, 11])  # x for the barriers
    # listy = np.array([0.2, 0.25, 0.3, 0.4, 0.45, 0.55, 0.8, 1.2, 1.8, 2, 2.2, 2.4, 2.6, 2.8, 3, 3.2, 3.5, 3.8, 4.2, 4.8, 5.8, 8, 9,10, 11, 12, 13])  # y for the barriers
    listx = np.array([-10])
    listy = np.array([0])
    r = np.array([0.5]*len(listx))  # radius for barriers
    startpoint = np.array([0, 0])
    endpoint = np.array([1, 0])

    # Parameters for the mpc control
    Ts = 0.2  # sampletime
    DesireSpeed = 0.4 # m/s
    initpose = np.append(startpoint, np.array([0]))
    pose = np.array([0, 0, 0])
    umax = np.array([0.5, 0.5, 0.8])  # ([maxspeed for x, for y , for w])
    umin = np.array([-0.5, -0.5, -0.8])  # ([maxspeed for x, for y , for w])
    dumax = np.array([0.3, 0.3, 0.4])  # the  accelerations of vx,vy,w
    dumin = np.array([-0.3, -0.3, -0.4])
    N = 5  # Prediction horizon
    resolution = 0.1  # the distance of equdistpoint.

    ref = process_waypoints(DCCP_planning(listx, listy, r, startpoint, endpoint))


    # pose_queue = multiprocessing.Queue()
    # caculte_pose = multiprocessing.Queue()
    # img = multiprocessing.Queue()
    # pose_thread = multiprocessing.Process(target=get_pose, args=(Estimater, img, pose_queue, caculte_pose,), daemon=True)
    # pose_thread.start()
    run(ws, ref)
    ws.close()
