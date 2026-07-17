import time

from FlightController.Components.RosManager import RosManager

rm = RosManager()
rm.chmod("/dev/ttyUSB0")
rm.chmod("/dev/video0")
rm.chmod("/dev/video1")
rm.launch_package("ldlidar_stl_ros2", "ld06.launch.py") # 不用自己写
time.sleep(0.5)
rm.launch_package("realsense2_camera", "rs_launch.py") # 不用自己写
time.sleep(0.5)
rm.launch_package("drone_cartographer", "cartographer.launch.py") # 需要在安装后自行编写
time.sleep(1)
rm.run_package("tf2_ros", "static_transform_publisher", "0 0 0 0 0 0 camera_pose_frame base_link")
# rm.run_package("tf2_ros", "tf2_echo", "map camera_pose_frame 10", sub_id=1)
