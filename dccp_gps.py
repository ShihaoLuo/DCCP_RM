# -*- coding: utf-8 -*-
"""
Created on Sun Mar 14 15:52:21 2021

@author: 13738
"""
import cvxpy as cp
import time
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import dccp
import math
import scipy as sp
from scipy import sparse
import scipy.interpolate as interpolate
import serial
import multiprocessing as mp
import pyproj
import socket
import sys


#from mpl_toolkits.mplot3d import Axes3D

pose_queue = mp.Queue()
#parameters
listx = np.array([0.3,0.5,0.6])#x for the barriers
listy = np.array([0.3,0.5,0.7])#y for the barriers
r = np.array([0.1,0.2,0.1])# radius for barriers
startpoint = np.array([0,0])
endpoint = np.array([1.2,1.2])
#Parameters for the mpc control
Ts = 0.5 #sampletime
DesireSpeed = 0.3#m/s
initpose = np.array([77.6, 50.8])
pose = np.array([0,0,0])
umax = np.array([0.5,0.5,0.5])# ([maxspeed for x, for y , for w])
umin = np.array([-0.5,-0.5,-0.5])# ([maxspeed for x, for y , for w])
dumax = np.array([0.2,0.2,0.2])# the  accelerations of vx,vy,w
dumin = np.array([-0.2,-0.2,-0.2])
N = 5 # Prediction horizon
resolution = 0.5# the distance of equdistpoint.

# weighting matrix function
Q = sparse.diags([1.,1.,1.])
QN = Q *10
R = 0.1*sparse.eye(3)

def get_pose(port, pose):
    print("get pose thread start.")
    try:
        ser = serial.Serial(port, 115200)
        #now = time.time()
        #arr = np.array([])
        arr_x = np.array([])
        arr_y = np.array([])
        wgs84 = pyproj.CRS("EPSG:4326")
        UTM50N = pyproj.CRS(proj='utm', zone=50,ellps='WGS84', units='m',vunits='m', datum='WGS84',\
                       lat_0=22.4, lon_0=114.1)
        print(UTM50N.to_string)
        transformer = pyproj.Transformer.from_crs(wgs84, UTM50N, always_xy=True)
        while True:
            #print('out:{}'.format(serrefout.in_waiting))
            if ser.in_waiting:
                raw = ser.readline()
                #print(raw)
                try:
                    if 'GNGGA' in raw.decode():
        #                     print(raw.decode())
                        data = raw.decode().split(',')
                        if data[6] == '5' or data[6] == '4':
        #                         print("get DGNSS.")
                            print(raw.decode())
                            tmp = np.array([float(data[4][0:3]),float(data[4][3:5]),float(data[4][5:])])
                            tmp_x = tmp[0] + tmp[1]/60 + tmp[2]/36
                            #arr_x = np.append(arr_x, tmp)
                            tmp = np.array([float(data[2][0:2]), float(data[2][2:4]),float(data[2][4:])])
                            tmp_y = tmp[0] + tmp[1]/60 + tmp[2]/36
                            #arr_y = np.append(arr_y, tmp)
                            #time.sleep(0.002)
                            x, y = transformer.transform(tmp_x, tmp_y)
                            x /= 1.4
                            y /= 1.4
                            x -= 152400
                            y -= 1794400
                            print("in get _pose, update pose:",x ,y)
                            while pose.empty() is False:
                                pose.get()
                            pose.put(x)
                            pose.put(y)
                except Exception:
                    pass
        #           print(e)
        #           print("error")
            time.sleep(0.015)
           # if time.time() - now > 30:
            #    break
        #ser.close()
    except serial.SerialException:
        print("ser creating error.")


def DCCP_planning(listx, listy, r, startpoint, end_point):
    time_start=time.time()
    #l = 10
    n = 25
    a = np.matrix(startpoint).T
    b = np.matrix(end_point).T
    d = 2
    #listx是obstacle的x轴，listy是obstacle的y轴，r是半径
    # listx = [16.3,14.7,15.5,16.3,17.3,18,18.5,15.5,15.5,16.3,17.65,17.65,17.65,17.65]
    # listy = [8.3,6.7,10.3,6.3,10,6.8,8.7,0.5,5,2.5,3.65,2.95,2.25,1.55]
    # r = [1.4,0.3,0.3,0.3,0.3,0.3,0.3,0.5,0.5,0.7,0.35,0.35,0.35,0.35]
    p = np.matrix([listx,listy])
    m = len(listx)
    x = []
    for i in range(n+1):
        x += [cp.Variable((d, 1))]
    L = cp.Variable(1)
    constr = [x[0] == a, x[n] == b]
    # constr += [x>=14]
    cost = L
    for i in range(n):
        constr += [cp.norm(x[i]-x[i+1]) <= L/n]
        for j in range(m):
            constr += [cp.norm(x[i]-p[:,j]) >= r[j]+0.1]
            #rectangular
            # constr += [abs(x[i][0]-2.5)+abs(x[i][1]-11 >= 2+0.1)]
    prob = cp.Problem(cp.Minimize(cost), constr)
    print("begin to solve")
    result = prob.solve(method='dccp')
    print("end") 
    dccp_path = np.array([]) 
    for xx in x:
        dccp_path = np.append(dccp_path,xx.value)
    dccp_path = dccp_path.reshape(math.floor((len(dccp_path)/2)),2).T
    time_end=time.time()
    print("time =", time_end - time_start)
    return(dccp_path)


def command(s,msg):
    msg +=';'
    print ('send -> ', msg)
    s.send(msg.encode('utf-8'))
    try:
        buf = s.recv(1024)
        print(buf.decode('utf-8'))
    except socket.error as e:
        print("Error receiving :", e)
        sys.exit(1)

def sendcommand(s, command1):
    spell = 'chassis move x ' + str(command1[0]) + ' y ' + str(command1[1])
    print(spell)
    command(s,spell)

def run(s, pose_queue, ref):
    now = time.time()
    pose = initpose
    print("len:", len(ref[0]))
    for i in range(len(ref[0])):
        print('estimated pose:', pose)
        print("target:", [ref[0][i], ref[1][i]])
        x = pose[0]-ref[0][i]
        y = pose[1]-ref[1][i]
        if x > 5:
            x = 5
        if x < -5:
            x = -5
        if y > 5:
            y = 5
        if y < -5:
            y = -5
        sendcommand(s, [x, -y])
        # time.sleep(2)
        pose[0] = pose[0] - x
        pose[1] = pose[1] - y
        t = np.linalg.norm([x, y]) / 0.5
        time.sleep(t)
        if time.time() - now > 5:
                sendcommand(s, np.zeros(3))
                time.sleep(1)
                if pose_queue.empty() is False:
                    x = pose_queue.get()
                    y = pose_queue.get()
                    print("gps pose:",x,y)
                    pose[0] = x
                    pose[1] = y
                else:
                    print('pose is empty.')
                now = time.time()
    sendcommand(s, np.zeros(2))
    s.shutdown(socket.SHUT_WR)
    s.close()

def main():
    pose_thread = mp.Process(target=get_pose, args=('COM4', pose_queue,))
    pose_thread.start()
    # dccp_path = DCCP_planning(listx,listy,r,startpoint,endpoint)
    dccp_path = np.array([[77.6, 69.78, 77.5, 85.12],
                          [50.8, 58.5, 66.17, 58.37]])
    cvxp = dccp_path
    cvxd = np.zeros(len(cvxp[0]))

    # cvxp = np.insert(cvxp,0,initpose[0:2],axis=1)
    cvxp_x = cvxp[0, :]
    cvxp_y = cvxp[1, :]
    for i in range(10):
        cvxp_x = cvxp_x.repeat(2)[:-1]
        cvxp_y = cvxp_y.repeat(2)[:-1]
        cvxp_x[1::2] += (cvxp_x[2::2] - cvxp_x[1::2]) / 2
        cvxp_y[1::2] += (cvxp_y[2::2] - cvxp_y[1::2]) / 2
    # cvxp_x,cvxp_y
    distance = np.sqrt(np.ediff1d(cvxp_x) ** 2 + np.ediff1d(cvxp_y) ** 2)
    d = 0.0
    equdist_waypoint = np.array([[cvxp_x[0]], [cvxp_y[0]], [cvxd[0]]])
    for i in range(len(distance)):
        d = distance[i] + d
        if d >= resolution:
            equdist_waypoint = np.append(equdist_waypoint, [[cvxp_x[i]], [cvxp_y[i]], [cvxd[0]]], axis=1)
            d = 0.0
    equdist_waypoint = np.append(equdist_waypoint, [[cvxp_x[-1]], [cvxp_y[-1]], [cvxd[-1]]], axis=1)
    tVec2 = np.linspace(0, len(equdist_waypoint[0, :]), num=int(len(equdist_waypoint[0, :])))
    tVec = np.linspace(0, len(equdist_waypoint[0, :]),
                       num=int(len(equdist_waypoint[0, :]) * (resolution / (DesireSpeed * Ts))))
    f = interpolate.interp1d(tVec2, equdist_waypoint, kind='cubic')
    ref = equdist_waypoint
    # 组网模式下，机器人当前 IP 地址为 192.168.0.115, 控制命令端口号为 40923
    # 机器人 IP 地址根据实际 IP 进行修改
    # host = "192.168.50.76"
    # port = 40923
    host = '192.168.2.1'# 直连模式
    port = 40923
    address = (host, int(port))
    # 与机器人控制命令端口建立 TCP 连接
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print("Connecting...")
    s.setblocking(True)
    s.connect(address)
    print("Connected!")
    command(s,'command')
    sendcommand(s, np.zeros(3))
    command(s,'chassis move x 0 y 0 z 0')
    while True:
        if pose_queue.empty() is False:
            break
    run(s, pose_queue, ref)

if __name__ == '__main__':
    main()
