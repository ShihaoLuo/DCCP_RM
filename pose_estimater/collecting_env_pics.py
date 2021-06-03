#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Sep 25 09:33:51 2020

@author: jake
"""
import time
import os
import cv2

name = 'flag'
num = 10

pic_folder = './dataset/'+name
if not os.path.exists(pic_folder):
    os.mkdir(pic_folder)
pic_folder = './dataset/'+name+'/images/'
if not os.path.exists(pic_folder):
    os.mkdir(pic_folder)

cap = cv2.VideoCapture(2)
cap.set(3, 1280)
cap.set(4, 960)
for i in range(10):
    ret, img = cap.read()
    cv2.imwrite(pic_folder+name+str(i)+'.jpg', img)
    time.sleep(1)



