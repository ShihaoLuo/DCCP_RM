import pose_estimater
import cv2 as cv
import numpy as np
import time

pose_estimater = pose_estimater.PoseEstimater('SIFT', 25)
pose_estimater.loaddata('./dataset/')
img = cv.imread('../192.168.31.179_screenshot_15.04.20211.png')
pose = np.array([240, 500, 240, 180])
old = time.time()
_pose, yaw = pose_estimater.estimate_pose(img, pose)
print("{}s".format(time.time() - old))
print(_pose, yaw)