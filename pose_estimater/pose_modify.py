# -*- coding: utf-8 -*-
# @Time    : 2021/2/2 下午3:28
# @Author  : JakeShihao Luo
# @Email   : jakeshihaoluo@gmail.com
# @File    : pose_m0dify.py.py
# @Software: PyCharm

from pose_estimater import *
import numpy as np

po = PoseEstimater(min_match=25)
po.loaddata('./dataset/')
po.showdataset()
# po.modifydata('flag', False, False, True, False, 0, [800, 0, 0])
