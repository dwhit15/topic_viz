#!/usr/bin/env python


import sys
from bag_tools import *


def hook():

    data = msg_data()
    data.get_data_from_bag(["","-l"])

    plot_group_list = [
        ["imu/imu/linear_acceleration","imu/imu/angular_velocity"],
        ["imu/imu/orientation"],["imu/imu/orientation/x"]]
    data.plot(plot_group_list)

# run hook function on shutdown
rospy.on_shutdown(hook)

while not rospy.is_shutdown():
    continue
