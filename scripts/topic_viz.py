#!/usr/bin/env python
'''

Inspired by script written by Nick Speal in May 2013 at McGill University's
Aerospace Mechatronics Laboratory (www.speal.ca).Supervised by Professor
Inna Sharf, Professor Meyer Nahon

'''


import string
import os
import inspect
import glob

import numpy as np

import rosbag
import rospy
import inspect

from geometry_msgs.msg import Vector3, Quaternion
from genpy.rostime import Time as ros_time

import numpy as np
import rosbag
from geometry_msgs.msg import Vector3, Quaternion
from genpy.rostime import Time as ros_time
from matplotlib.backends.backend_pdf import PdfPages
from multiplot2d import multiplotter
import matplotlib.pylab as plt
import matplotlib as mpl

class msg_data():
    save_list = [float,int,bool]
    stop_recursion_types = [str,bool,int,float,tuple,dict,list,ros_time]

    def __init__(self):
        self.data_dict = dict()
        self.name = "data"
        self.time_stamp_field = "/header/stamp"

    def get_fields(cls,obj):
        field_dict = dict()
        # loop through all attributes
        for name in dir(obj):
            # filter out private attributes and methods
            if not name.startswith('__') and not name.startswith('_') and not inspect.ismethod(getattr(obj, name)):
                # if the value is another message, recurse
                value = getattr(obj, name)
                value_type = type(value)
                if value_type not in cls.stop_recursion_types:
                	field_dict[name] = cls.get_fields(value)
                # otherwise, add the data to the list
                else:
                	if value_type in cls.save_list:
                		field_dict[name] = value
                	elif value_type is ros_time:
                		field_dict[name] = value.to_sec()
        return field_dict

    def pull_data_from_fields(cls,data_dict,field_dict,namespace,obj):
        for name in field_dict.keys():
            full_name = namespace+"/"+name
            msg_value = getattr(obj, name)
            if type(msg_value) in cls.save_list:
                data_dict.setdefault(full_name,[]).append(msg_value)
            elif type(msg_value) is ros_time:
                data_dict.setdefault(full_name,[]).append(msg_value.to_sec())
            else:
                cls.pull_data_from_fields(data_dict,field_dict[name],full_name,msg_value)

    def parse_topic_str(self,topic_string):
        # look for a topic that starts with the input string
        available_topic_list = self.data_dict.keys()
        matching_topic_list = [topic_name for topic_name in available_topic_list if topic_string.startswith(topic_name)]
        if len(matching_topic_list) > 1:
            raise NameError("There are multiple topic names which start with %s. " %topic_string)
        elif len(matching_topic_list) == 0:
            option_str = "\n".join(available_topic_list)
            raise NameError("\n\nNo topics are named %s. \nOptions are:\n%s\n" %(topic_string,option_str) )
        matching_topic_name = matching_topic_list[0]
        # remove the topic name, yielding the desired field name string
        field_name = topic_string.replace(matching_topic_name,"")

        # figure out which field to plot
        topic_data_dict = self.data_dict[matching_topic_name]["data"]
        topic_info_dict = self.data_dict[matching_topic_name]["info"]
        available_field_list = topic_data_dict.keys()

        matching_field_list = []
        if field_name in available_field_list:
                matching_field_list.append(field_name)
        else:
            try:
                for available_field in available_field_list:
                    if available_field.startswith(field_name):
                        matching_field_list.append(available_field)
            except:
                "no matching fields"
                return -1
        matching_field_list.sort()
        return matching_topic_name, matching_field_list

    def parse_group(self,group_list):
        formatted_group_list = []
        style_list = []
        for group_num, group in enumerate(group_list):
            figure_list = []
            style_list.append(group["style"])
            for topic_num, topic_str in enumerate(group["topics"]):
                topic_name, field_list = self.parse_topic_str(topic_str)
                formatted_group_dict = dict()
                formatted_group_dict[topic_name] = field_list
                figure_list.append(formatted_group_dict)
            formatted_group_list.append(figure_list)
        return formatted_group_list,style_list

    def plot(self,plot_groups):
        try:
            os.makedirs(self.name)
        except:
            pass

        group_list,style_list = self.parse_group(plot_groups)

        plotter_list = []
        try: plt.style.use("dwplot")
        except: pass
        mpl.rcParams['lines.linewidth'] = 1

        for group_num, group in enumerate(group_list):
            dataset_field_num_list = []
            for dataset in group:
                topic_name = dataset.keys()[0]
                dataset_field_num_list.append(len(dataset[topic_name]))

            if not all(num == dataset_field_num_list[0] for num in dataset_field_num_list):
                raise ValueError("The datasets in each group must have the same number of fields.")

            num_fields = dataset_field_num_list[0]
            num_datasets = len(group)

            plotter = multiplotter([num_fields,1],name="Plot "+str(group_num),size_inches=(6,num_fields*2))
            style_str = style_list[group_num]

            for dataset_num, dataset in enumerate(group):
                # create array of data
                topic_name = dataset.keys()[0]
                field_list = dataset[topic_name]
                topic_data_dict = self.data_dict[topic_name]["data"]
                topic_info_dict = self.data_dict[topic_name]["info"]
                num_msgs = topic_info_dict["num_msgs"]
                data_array = np.zeros((num_msgs,num_fields))
                for field_index, sub_field_name in enumerate(field_list):
                    data_array[:,field_index] = topic_data_dict[sub_field_name]

                # create time matrix
                time = np.zeros((num_msgs,1))
                # TODO check for this topic first
                time[:,0] = topic_data_dict[self.time_stamp_field]
                time[:,0] -= time[0,0]

                subplot_titles=[]
                label = ""
                for field in field_list:
                    subplot_titles.append([x.strip() for x in field.split('/')][-1])
                    namespace_list = [x.strip() for x in field.split('/')][0:-1]
                    label = topic_name + "/".join(namespace_list)

                line_styles = dict()

                if style_str=="marker":
                    line_styles = dict(ls="",marker="o",markersize=1.5,mew=0)

                plot_ids = range(num_fields)
                plotter.add_data(plot_ids,time,data_array,labels=label,line_styles=line_styles)
                plotter.set_axis_titles(plot_ids,"Time [s]","")
                plotter.set_plot_titles(plot_ids,subplot_titles)

            plotter.add_grid("all")
            legend_args=dict()
            plotter.add_figure_legend(
                legend_whitespace=0.2*num_datasets,
                legend_args=legend_args)
            plotter_list.append(plotter)

        filename = "%s/fig.pdf" %self.name
        with PdfPages(filename) as pdf:
            [plotter.save(pdf=pdf) for plotter in plotter_list]


    def write_csv(self):
        #create a new directory
        try:
            os.makedirs(self.name)
        except:
            pass

        for topic_name in self.data_dict.keys():
            topic_data_dict = self.data_dict[topic_name]["data"]
            topic_info_dict = self.data_dict[topic_name]["info"]
            field_dict = topic_data_dict.keys()

            # sort fields in alphabetical order
            # also put the timestamp first if it exists
            field_dict.sort()
            try:
                field_dict.insert(0, field_dict.pop(field_dict.index(self.time_stamp_field)))
            except:
                pass

            # create filename
            filename = self.name + '/' + string.replace(topic_name, '/', '_') + '.csv'
            # use names of fields to create csv header
            csv_header = ",".join(field_dict)

            # fill numpy array with data
            num_msgs = topic_info_dict["num_msgs"]
            output_array = np.zeros((num_msgs,len(field_dict)))
            for col_index, field_name in enumerate(field_dict):
                output_array[:,col_index] = topic_data_dict[field_name]

            # write csv
            np.savetxt(filename,output_array,delimiter=",",fmt="%.5f",
                header=csv_header, comments='#')

    def get_data_from_bag(self,bag_directory,bags_to_read):

        # change to specified directory
        os.chdir(bag_directory)

        # if bags_to_read is a string, assume that it is the name of a bag
        # unless the arg is a -l flag, in which case we want to find the
        # latest bag in the directory
        if type(bags_to_read) == str:
            if bags_to_read == "-l":
                bag_name_list = [max(glob.iglob('*.bag'), key=os.path.getctime)]
            else:
                bag_name_list = [bags_to_read]
        # otherwise, assume it's a list of bags
        else:
            bag_name_list = bags_to_read

        # read all bags
        for bag_name in bag_name_list:

            print "Reading %s." %bag_name

            # read bag
            bag = rosbag.Bag(bag_name)
            bag_contents = bag.read_messages()

            # name attribute will be used as a default name for output
            # TODO decide what to do in the case of mulitple bags
            self.name = string.rstrip(bag_name, ".bag")

            # get list of topics from the bag
            # topic_list = []
            # for topic, msg, t in bag_contents:
            #     if topic not in self.data_dict.keys():
            #         topic_list.append(topic)
            topic_list = []
            for topic, msg, t in bag_contents:
                if topic not in topic_list:
                    topic_list.append(topic)

            print topic_list

            # loop through all topics and store data
            for topic_name in topic_list:
                # look through the message and "figure out" the way the data
                # is structured (this structure is stored in field_dict)
                sub_topic, first_msg, first_time = bag.read_messages(topic_name).next()
                field_dict = self.get_fields(first_msg)

                # if there is no header, than there are no time stamps, so
                # don't save this topic
                if "header" not in field_dict.keys():
                    print "Topic %s does not have a header. Ignoring this topic." %(topic_name)
                    continue

                # initialize storage for this topic
                self.data_dict[topic_name] = dict()
                self.data_dict[topic_name]["data"] = dict()
                self.data_dict[topic_name]["info"] = dict()
                self.data_dict[topic_name]["info"]["num_msgs"] = bag.get_message_count(topic_name)
                for subtopic, msg, t in bag.read_messages(topic_name):
                    self.pull_data_from_fields(self.data_dict[topic_name]["data"],field_dict,"",msg)


            bag.close()
            print "Done reading all bag files."
