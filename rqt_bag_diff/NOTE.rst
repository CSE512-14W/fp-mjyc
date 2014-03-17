^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Note for package rqt_bag_diff
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

(2014-03-13)
------------------
* rosbag event conversion command

Terminal1:
rosparam set /use_sim_time true

Terminal2:
rosbag play --clock --pause ~/bagfiles/test.bag

Terminal3:
rosbag record /audio /camera/depth_registered/camera_info /camera/depth_registered/image_raw /camera/rgb/camera_info /camera/rgb/image_raw /imu /manual_elevator_state_publisher/events/door /manual_elevator_state_publisher/events/elevator_state /manual_elevator_state_publisher/events/robot_position /manual_elevator_state_publisher/events/level /scan /wifi_data -O /home/mjyc/bagfiles/test2


(2014-03-10)
------------------
* PyQt Debug Code

from PyQt4.QtCore import pyqtRemoveInputHook
from ipdb import set_trace
pyqtRemoveInputHook()
set_trace()
