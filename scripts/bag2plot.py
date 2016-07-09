#!/usr/bin/env python


# import sys
from topic_viz import *
import rospy

# start ros node
rospy.init_node("plot")

# get parameters
plot_groups = rospy.get_param("~plot_groups")
bag_directory = rospy.get_param("~bag_directory")
bags_to_read = rospy.get_param("~bags_to_read")

# initialize empty data structure
data = msg_data()

# read some bags to get data
data.get_data_from_bag(bag_directory,bags_to_read)

# plot the data in the prescribed format
data.plot(plot_groups)
