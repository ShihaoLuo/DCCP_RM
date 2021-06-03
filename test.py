import time

from robomaster import robot

robot = robot.Robot()
robot.initialize(conn_type='ap')
version = robot.get_version()
print("Robot version: {0}".format(version))
camera = robot.camera
camera.start_video_stream(display=True, resolution='720p')
img = camera.read_cv2_image()
time.sleep(10)
camera.stop_video_stream()
robot.close()
