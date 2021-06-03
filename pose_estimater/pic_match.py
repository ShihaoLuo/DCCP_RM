#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 26 11:04:13 2020

@author: jake
"""

import numpy as np
import cv2.cv2 as cv
import matplotlib.pyplot as plt
import json
import multiprocessing
# import imutils
import numpy as np
# import joblib
#import set_world_point

object_name = 'flag'

def save_2_jason(_file, arr):
    data = {}
    cnt = 0
    for i in arr:
        data['KeyPoint_%d' % cnt] = []
        data['KeyPoint_%d' % cnt].append({'x': i.pt[0]})
        data['KeyPoint_%d' % cnt].append({'y': i.pt[1]})
        data['KeyPoint_%d' % cnt].append({'size': i.size})
        cnt += 1
    with open(_file, 'w') as outfile:
        json.dump(data, outfile)


def save_2_npy(_file, arr):
    np.save(_file, arr)


def read_from_jason(_file):
    result = []
    with open(_file) as json_file:
        data = json.load(json_file)
        cnt = 0
        while(data.__contains__('KeyPoint_%d' % cnt)):
            pt = cv.KeyPoint(x=data['KeyPoint_%d' % cnt][0]['x'],
                             y=data['KeyPoint_%d' % cnt][1]['y'],
                             _size=data['KeyPoint_%d' % cnt][2]['size'])
            result.append(pt)
            cnt += 1
    return result


def read_from_npy(_file):
    return np.load(_file)


def get_ROI(_img):
    cv.namedWindow('roi', cv.WINDOW_NORMAL)
    roi = cv.selectROI('roi', _img, True, False)
    cv.destroyAllWindows()
    x, y, w, h = roi
    dst = _img[y:y + h, x:x + w]
    return dst

pts = []

# :mouse callback function
def draw_roi(event, x, y, flags, param):
    img2 = img.copy()
    if event == cv.EVENT_LBUTTONDOWN:  # Left click, select point
        pts.append((x, y))
    if event == cv.EVENT_RBUTTONDOWN:  # Right click to cancel the last selected point
        pts.pop()
    if event == cv.EVENT_MBUTTONDOWN:  #
        mask = np.zeros(img.shape, np.uint8)
        points = np.array(pts, np.int32)
        points = points.reshape((-1, 1, 2))
        mask = cv.polylines(mask, [points], True, (255, 255, 255), 2)
        mask2 = cv.fillPoly(mask.copy(), [points], (255, 255, 255))  # for ROI
        mask3 = cv.fillPoly(mask.copy(), [points], (0, 255, 0))  # for displaying images on the desktop
        show_image = cv.addWeighted(src1=img, alpha=0.8, src2=mask3, beta=0.2, gamma=0)
        cv.imshow("mask", mask2)
        cv.imshow("show_img", show_image)
        ROI = cv.bitwise_and(mask2, img)
        cv.imshow("ROI", ROI)
        cv.imwrite('./dataset/' + object_name + '/images/' + object_name + '.jpg', ROI)
        cv.waitKey(0)
    if len(pts) > 0:
        # Draw the last point in pts
        cv.circle(img2, pts[-1], 3, (0, 0, 255), -1)
    if len(pts) > 1:
        #
        for i in range(len(pts) - 1):
            cv.circle(img2, pts[i], 5, (0, 0, 255), -1)  # x ,y is the coordinates of the mouse click place
        cv.line(img=img2, pt1=pts[i], pt2=pts[i + 1], color=(255, 0, 0), thickness=2)
    cv.imshow('image', img2)

    # Create images and windows and bind windows to callback functions



MIN_MATH_COUNT = 5
kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])

img_test = cv.imread('./dataset/'+object_name+'/images/'+object_name+'0.jpg')
img_query = cv.imread('./dataset/'+object_name+'/images/'+object_name+'5.jpg')
# img_query = cv.resize(img_query, (int(4032/2), int(3024/2)), interpolation=cv.INTER_AREA)
# print('./dataset/'+object_name+'/images/'+object_name+'0.jpg')
#img_test = cv.filter2D(img_test, -1, kernel)
#img_query = cv.filter2D(img_query, -1, kernel)
img_query = get_ROI(img_query)
#img_test = get_ROI(img_test)
# img = img_query
# # img = imutils.resize(img, width=500)
# cv.namedWindow('image')
# cv.setMouseCallback('image', draw_roi)
# print("[INFO] Click the left button: select the point, right click: delete the last selected point, click the middle button: determine the ROI area")
# print("[INFO] Press ‘S’ to determine the selection area and save it")
# print("[INFO] Press ESC to quit")
# while True:
#     key = cv.waitKey(1) & 0xFF
#     if key == 27:
#         break
#     if key == ord("s"):
#         saved_data = {
#             "ROI": pts
#         }
#         # joblib.dump(value=saved_data, filename="config.pkl")
#         print("[INFO] ROI coordinates have been saved to local.")
#         break
# cv.destroyAllWindows()

# img_query = cv.imread('./dataset/'+object_name+'/images/'+object_name+'.jpg')
# img_query = get_ROI(img_query)
cv.imwrite('./dataset/' + object_name + '/images/' + object_name + '.jpg', img_query)

sift_paras = dict(nfeatures=0,
                 nOctaveLayers=3,
                 contrastThreshold=0.05,
                 edgeThreshold=10,
                 sigma=0.8)

'''surf_paras = dict(hessianThreshold=100,
                  nOctaves=10,
                  nOctaveLayers=2,
                  extended=1,
                  upright=0)
surf = cv.xfeatures2d.SURF_create(**surf_paras)'''
sift = cv.xfeatures2d.SIFT_create(**sift_paras)
kp_query, des_query = sift.detectAndCompute(img_query, None)
# kp_query = kp_query[0:len(kp_query):20]
# des_query = des_query[0:len(des_query):20]
# print(kp_query[0].class_id)
save_2_jason('dataset/'+object_name+'/kp.json',kp_query)
save_2_npy('dataset/'+object_name+'/des.npy',des_query)
#save_2_jason('kp_query.jason', kp_query)
#save_2_npy('des_query.npy', des_query)
kp_test, des_test = sift.detectAndCompute(img_test, None)
# kp_test = kp_query[0:len(kp_query):20]
# des_test = des_query[0:len(des_query):20]
#kp_query_1 = read_from_jason('kp_goodm.jason')
#des_query_1 = read_from_npy('des_goodm.npy')
FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
search_params = dict(checks=50)

flann = cv.FlannBasedMatcher(index_params, search_params)

matches = flann.knnMatch(des_query, des_test, k=2)

good = []
kp_good_match_query = []
des_good_match_query = []
for m, n in matches:
    if m.distance < 0.56*n.distance:
        good.append(m)
        print('--------------------\n')
        print('m.imgIdx: {}\n'.format(m.imgIdx))
        print('m.queryIdx: {}\n'.format(m.queryIdx))
        print('m.trainIdx: {}\n'.format(m.trainIdx))
        print('kp_query: {}\n'.format(kp_query[m.queryIdx].pt))
        print('kp_test: {}\n'.format(kp_test[m.trainIdx].pt))
        kp_good_match_query.append(kp_query[m.queryIdx])
        des_good_match_query.append(des_query[m.queryIdx])
print('the num of finding featurs of query is {}\n'.format(len(des_query)))
print('the num of finding featurs of test is {}\n'.format(len(des_test)))
print('the num of finding matches is {}\n'.format(len(matches)))
print("the len of good match is {}\n".format(len(good)))
#save_2_jason('dataset/'+object_name+'/kp.json',kp_good_match_query)
#save_2_npy('dataset/'+object_name+'/des.npy',des_good_match_query)
if len(good)>=MIN_MATH_COUNT:
    src_pts = np.float32([kp_good_match_query[i].pt for i in range(len(kp_good_match_query))]).reshape(-1,1,2)
    #src_pts = np.float32([kp_query_1[m.queryIdx].pt for m in good]).reshape(-1,1,2)
    dst_pts = np.float32([kp_test[m.trainIdx].pt for m in good]).reshape(-1,1,2)

    M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC, 2.0)
    matchesMask = mask.ravel().tolist()
    h,w = img_query.shape[0:2]
    d = 1
    pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
    dst = cv.perspectiveTransform(pts,M)
    img_test = cv.polylines(img_test,[np.int32(dst)],True,255,1,cv.LINE_AA)
else:
    print("Not enough matchs are found - {}/{}".format(len(good),MIN_MATH_COUNT))
    matchesMask = None
draw_params = dict(matchColor = (0,255,0),
                   singlePointColor = None,
                   matchesMask = matchesMask,
                   flags = 2)
img = cv.drawMatches(img_query,kp_query,img_test,kp_test,good,None,**draw_params)

def show_pic(_img, _img_query):
    #fig = plt.figure(figsize=(12, 10))
    #plt.subplot(1, 1, 1).axis("off")
    plt.imshow(_img)
    plt.show()
    #plt.subplot(1, 1, 1).axis("off")
    plt.imshow(_img_query)
    plt.show()

thread = multiprocessing.Process(target=show_pic, args=(img, img_query,))
thread.start()
wpixel = np.array([])
wpoint = np.array([])
for i in range(8):
    wpxlx = input('input the x of No.{} wpixel:'.format(i+1))
    wpxly = input('input the y of No.{} wpixel:'.format(i+1))
    wptx = input('input the x of No.{} wpoint:'.format(i+1))
    wpty = input('input the y of No.{} wpoint:'.format(i+1))
    wptz = input('input the z of No.{} wpoint:'.format(i + 1))
    wpxl = np.array([float(wpxlx), float(wpxly)])
    wpt = np.array([float(wptx), float(wpty), float(wptz)])
    wpixel =np.append(wpixel, wpxl)
    wpoint = np.append(wpoint, wpt)
ob_pointx = input('input the x of the ob point:')
ob_pointy = input('input the y of the ob point:')
ob_pointz = input('input the z of the ob point:')
ob_point_yaw = input('input the yaw of the ob point:')
ob_point = np.array([float(ob_pointx), float(ob_pointy), float(ob_pointz), float(ob_point_yaw)])
save_2_npy('dataset/'+object_name+'/obpoint.npy', ob_point)
save_2_npy('dataset/'+object_name+'/wpixel.npy', wpixel)
save_2_npy('dataset/'+object_name+'/wpoint.npy', wpoint)
print(wpixel)
print(wpoint)
print(ob_point)

'''
img_gkp = cv.drawKeypoints(img_query, kp_good_match_query, None)
plt.imshow(img_gkp)
wps = [[None, None, None]]*len(kp_good_match_query)
# pixelpts = [[541.0, 318.5], [625.0, 268.5], [445.0, 301.0], [433.0, 315.6], [413.0, 338.0], [450.0, 440.0], [479.0, 335.5], [482.0, 314.0], [486.0, 305.0]]
# worldpts = [[-35.0,81.0, 22.0], [-73.0, 60.0, 21.0], [9.0, 67.0, 41.4], [14.2, 76.9, 41.4], [23.0, 86.5,41.4],[-5.0, 88.0, 41.4],[-12.5, 85.5, 41.4],[-14.0, 77.0, 41.4],[-15.5, 72.0, 41.4]]
#for i in range(len(pixelpts)):
  #  set_world_point.setworldpoint(wps, kp_good_match_query, pixelpts[i], 5, worldpts[i])

img = cv.drawKeypoints(img_query, kp_query, None)
img2 = cv.drawKeypoints(img_test, kp_test, None)
fig = plt.figure(figsize=(22, 10))
plt.subplot(1, 2, 1)
plt.subplot(1, 2, 1).axis("off")
plt.imshow(img)
plt.subplot(1, 2, 2)
plt.subplot(1, 2, 2).axis("off")
plt.imshow(img2)
plt.show()
222.9 , 115.6 ,  77.95, 117.  ,  78.6 , 257.2 , 218.8 , 255.8 
'''
