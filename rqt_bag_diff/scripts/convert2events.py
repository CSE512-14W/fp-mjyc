#!/usr/bin/env python
import rospy
from elevator_data_collector.msg import *


rospy.init_node('listener', anonymous=True)

prev_door_state = ElevatorState.DOOR_CLOSED
prev_robot_position = ElevatorState.OUT_ELEVATOR
prev_level = 0

door_pub = rospy.Publisher('manual_elevator_state_publisher/events/door', DoorEvent)
robot_position_pub = rospy.Publisher('manual_elevator_state_publisher/events/robot_position', RobotPositionEvent)
level_pub = rospy.Publisher('manual_elevator_state_publisher/events/level', LevelEvent)


def callback(data):
    global prev_door_state
    global prev_robot_position
    global prev_level

    if data.door_state != prev_door_state:
        msg = DoorEvent()
        msg.state = data.door_state
        door_pub.publish(msg)
        prev_door_state = data.door_state

    if data.robot_position != prev_robot_position:
        msg = RobotPositionEvent()
        msg.state = data.robot_position
        robot_position_pub.publish(msg)
        prev_robot_position = data.robot_position

    if data.level != prev_level:
        msg = LevelEvent()
        msg.state = data.level
        level_pub.publish(msg)
        prev_level = data.level

def listener():
    rospy.Subscriber("manual_elevator_state_publisher/events/elevator_state", ElevatorState, callback)
    rospy.spin()

listener()
