import cvxpy as cp
import numpy as np
import math
from pose_estimater.pose_estimater import PoseEstimater
import multiprocessing
import time
import sys
sys.path.append("/home/nvidia/roborts_ws/src/key_ctrl_robo/src/")
import ctrl_node


def get_pose(Estimater, _img, pose_queue, caculate_pose):
    while True:
        tmp = caculate_pose.get()
        while _img.empty() is True:
            time.sleep(0.001)
        img = _img.get()
        pose = Estimater.estimate_pose(img, tmp)
        # print(pose)
        if pose[0] is not None:
            while pose_queue.empty() is False:
                pose_queue.get()
            pose_queue.put(pose)
            time.sleep(0.6)
        time.sleep(0.01)

def DCCP_planning(listx, listy, r, startpoint, end_point):
    time_start=time.time()
    #l = 10
    n = 20
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

def process_waypoints(dccp_path):
    cvxp = dccp_path
    cvxd = np.zeros(len(cvxp[0]))
    cvxp = np.insert(cvxp, 0, initpose[0:2], axis=1)
    cvxp_x = cvxp[0, :]
    cvxp_y = cvxp[1, :]
    for i in range(12):
        cvxp_x = cvxp_x.repeat(2)[:-1]
        cvxp_y = cvxp_y.repeat(2)[:-1]
        cvxp_x[1::2] += (cvxp_x[2::2] - cvxp_x[1::2]) / 2
        cvxp_y[1::2] += (cvxp_y[2::2] - cvxp_y[1::2]) / 2
    distance = np.sqrt(np.ediff1d(cvxp_x) ** 2 + np.ediff1d(cvxp_y) ** 2)
    d = 0.0
    equdist_waypoint = np.array([[cvxp_x[0]], [cvxp_y[0]], [cvxd[0]]])
    for i in range(len(distance)):
        d = distance[i] + d
        if d >= resolution:
            equdist_waypoint = np.append(equdist_waypoint, [[cvxp_x[i]], [cvxp_y[i]], [cvxd[0]]], axis=1)
            d = 0.0
    equdist_waypoint = np.append(equdist_waypoint, [[cvxp_x[-1]], [cvxp_y[-1]], [cvxd[-1]]], axis=1)
    # tVec2 = np.linspace(0, len(equdist_waypoint[0, :]), num=int(len(equdist_waypoint[0, :])))
    # tVec = np.linspace(0, len(equdist_waypoint[0, :]),
    #                    num=int(len(equdist_waypoint[0, :]) * (resolution / (DesireSpeed * Ts))))
    # f = interpolate.interp1d(tVec2, equdist_waypoint, kind='cubic')
    # ref = f(tVec)
    return equdist_waypoint

def cal_M(theta,Ts):
    M = np.zeros((3,3))
    M[0,0] = np.cos(theta)*Ts
    M[0,1] = np.sin(theta)*Ts
    M[1,0] = np.sin(theta)*Ts*(-1)
    M[1,1] = np.cos(theta)*Ts*(-1)
    M[2,2] = 1*Ts
    return M

def run(setter, caculate_pose, img):
    ref = process_waypoints(DCCP_planning(listx, listy, r, startpoint, endpoint))
    pose = np.array([-2, 2, 0])
    for i in range(len(ref)):
        # while img.empty() is False:
        #     img.get()
        # img.put(camera.read_cv2_image(timeout=0.1, strategy='newest'))
        start_time = time.time()
        # print(pose)
        while caculate_pose.empty() is False:
            caculate_pose.get()
        caculate_pose.put(ref[i])
        # PNP_pose = get_pose(camera.read_cv2_image(timeout=0.1, strategy='newest'), pose)
        if pose_queue.empty() is False:
            PNP_pose = pose_queue.get()
            if PNP_pose[0] is not None:
                # print(PNP_pose[0]/100)
                pose[0] = PNP_pose[0][0]/100
                pose[1] = PNP_pose[0][1]/100
                pose[2] = PNP_pose[1]
                print(pose)
        deltax = -ref[i][0] + pose[0]
        deltay = -ref[i][1] + pose[1]
        deltaz = -ref[i][2] + pose[2]
        pose = ref[i]
        # print('command:', command1)
        # chassis.drive_speed(x=command1[0], y=command1[1], z=command1[2], timeout=5)
        T = np.linalg.norm([deltax,deltay]) / 0.5
        # chassis.move(deltay, deltax, deltaz, 0.5, deltaz/T)
        time.sleep(T+5)

    # time.sleep(2)
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
    setter = ctrl_node.SetSpeed()
    # parameters
    listx = np.array([0])  # x for the barriers
    listy = np.array([0])  # y for the barriers
    r = np.array([0.01])  # radius for barriers
    startpoint = np.array([-2, 1.5])
    endpoint = np.array([2, 1.5])

    initpose = np.array([-2, 1.5, 0])
    pose = np.array([0, 0, 0])
    resolution = 0.5  # the distance of equdistpoint.

    # initial robot
    #robot = robot.Robot()
    #robot.initialize(conn_type='ap')
    #version = robot.get_version()
    #print("Robot version: {0}".format(version))
    #camera = robot.camera
    #camera.start_video_stream(display=True, resolution='720p')
    #chassis = robot.chassis
    Estimater = PoseEstimater(min_match=20)
    Estimater.loaddata('./pose_estimater/dataset/')
    pose_queue = multiprocessing.Queue()
    caculte_pose = multiprocessing.Queue()
    img = multiprocessing.Queue()
    pose_thread = multiprocessing.Process(target=get_pose, args=(Estimater, img, pose_queue, caculte_pose,), daemon=True)
    pose_thread.start()
    run(setter, caculte_pose, img)
