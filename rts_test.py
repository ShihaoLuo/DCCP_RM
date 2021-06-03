import os
import time
import sys
sys.path.append("/home/nvidia/roborts_ws/src/key_ctrl_robo/src/")
import ctrl_node

setter = ctrl_node.SetSpeed()
for i in range(20):
    setter.set(1, 0 ,0)
    time.sleep(0.5)


