#!/usr/bin/env python 
# -*- coding: utf-8 -*-
import sys
#reload(sys)
# sys.setdefaultencoding('utf-8')
import os
import csv
import cv2
import math
import time
import rospy 
import chardet
import threading
import actionlib  
import pandas as pd
import random

import numpy as np
from std_srvs.srv import Empty
# from xbot_talker.srv import play
from std_msgs.msg import String
from actionlib_msgs.msg import *  
# from xbot_face.msg import FaceResult
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
# import pdb
# pdb.set_trace()
from nav_msgs.srv import GetMap
import tf

class random_navi():
	def __init__(self):

		rospy.init_node('random_navi', anonymous=False) 

		###############################################################
		# navigation func
		################################################################
		# 在每个目标位置暂停的时间
		self.rest_time = rospy.get_param("~rest_time", 1) 
		# 读取csv 文件
		self.csv_file = rospy.get_param("~cvs_file", r'free_destination.csv')  

		# 关闭navi
		self.stop_navi_sub = rospy.Subscriber('/stop_navi_sub', String, self.stopCB)
		self.stop_navi_flag = False
		self.pause_navi_flag = False

		# 读取 csv 中待打开信息
		coding = self.get_encoding(self.csv_file)
		self.df = pd.read_csv(self.csv_file, index_col=False, encoding=coding)

		self.num_pos = len(self.df)
		#self.df_names_CN = self.df['name'].tolist() # unicode
		#print(self.df)

		# 发布控制机器人的消息  
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)  
		# 订阅move_base服务器的消息  
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  
		# clear custmap
		self.clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)

		# 启动行走服务

		# 到达目标的状态  
		self.goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',   
					'SUCCEEDED', 'ABORTED', 'REJECTED',  
					'PREEMPTING', 'RECALLING', 'RECALLED',  
					'LOST']  

		self.goal = MoveBaseGoal()

		rospy.on_shutdown(self.shutdown)

		# 60s等待时间限制  
		rospy.loginfo("Waiting for move_base action server...")  
		self.move_base.wait_for_server(rospy.Duration(60))  
		rospy.loginfo("Connected to move base server")  

		# 保存机器人的在rviz中的初始位置  
		initial_pose = PoseWithCovarianceStamped()  
		# 确保有初始位置  
		while initial_pose.header.stamp == "":  
		    rospy.sleep(1)  

		rospy.loginfo("Start free navigation progress")  

		#线程控制
		self.wake_event=threading.Event()

		##################
		# random goal
		###################
		self.distance_threshold_ = rospy.get_param('distance_threshold', 80.0)
		self.robot_radius_ = rospy.get_param('robot_radius', 0.7)
		self.heading_bins_ = rospy.get_param('heading_bins', 20.0)

		self.debug = '/home/nvidia/robot_competition/src/robotMain/' # path to save debug image

		print("distance_threshold : ", self.distance_threshold_)

		rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped, self.PoseCallBack)
		self.costMapInit()
		self.generateTatget()

	   # 开始主循环，随机导航  
		while not rospy.is_shutdown():
			#self.go()  
			if self.stop_navi_flag == True:
				break
			if self.pause_navi_flag == True:
			# 阻塞
				self.wake_event.wait()
				self.wake_event.clear()

			self.generateTatget()
			rospy.sleep(self.rest_time) 


	def go(self):
		rospy.loginfo('Going to the goal..') 
		#self.pub_log.publish('Going to: ' + person['name'])

		self.clear_costmaps_srv()
		self.move_base.send_goal(self.goal) 

		finished_within_time = self.move_base.wait_for_result(rospy.Duration(600))
		if not finished_within_time:
			self.move_base.cancel_goal()
			rospy.loginfo("Goal failed!!") 
		else:
			state = self.move_base.get_state()
			if state == GoalStatus.SUCCEEDED:
				rospy.loginfo("Goal succeeded!") 
				self.clear_costmaps_srv()
				self.success = True
			else:
				rospy.loginfo('goal failed with error code: ' + str(self.goal_states[state])) 


	def shutdown(self):  
		rospy.loginfo("关闭导航...")  
		#self.pub_log.publish("关闭迎宾.")
		#self.cmd_vel_pub(Twist())
		self.move_base.cancel_goal()  
		self.stop_navi_flag = True
		rospy.sleep(1)  

	def get_encoding(self, file): 
		# 二进制方式读取，获取字节数据，检测类型 
		with open(file, 'rb') as f: 
			return chardet.detect(f.read())['encoding']

	def stopCB(self, data):
		# pause, stop restart.
		#print("zzz",data.data)
		if data.data == "pause":
			self.move_base.cancel_goal()
			self.pause_navi_flag = True
		elif data.data == 'stop':
			self.move_base.cancel_goal() 
			self.stop_navi_flag = True


		elif data.data == 'restart':
			if not self.wake_event.isSet():
				self.wake_event.set() #继续　主线程
				self.pause_navi_flag = False
		else:
			rospy.loginfo("# 未知命令!!")


	def PoseCallBack(self, msg):
		#data=""
		#订阅到的坐标信息
		self.current_position_x = msg.pose.pose.position.x
		self.current_position_y = msg.pose.pose.position.y
		#订阅到的四元数的信息，用来表示朝向


	def costMapInit(self):
		rospy.wait_for_service("static_map", rospy.Duration(5.0))
		rospy.loginfo("ready for static map service.")
		try:
			mapData  = rospy.ServiceProxy('static_map',GetMap)
		except rospy.ServiceException as e:
			rospy.logerr("Service call failed: %s"%e)

		mapData = mapData().map #.info.origin.position.x #.map()

		self.map_origin_x_ = mapData.info.origin.position.x #-25.0 -25.0 0.05000000074505806
		self.map_origin_y_ = mapData.info.origin.position.y
		self.map_resolution_ = mapData.info.resolution

		self.map_size_x_ = mapData.info.width # 992 992
		self.map_size_y_ = mapData.info.height

		# 100 to indicate the cell is occupied.  边缘
		# 0 to indicate the cell is free.   ok goal
		# output -1 a.k.a. 255 (as an unsigned char), to indicate that the cell is unknown. 
		self.map_data_ = mapData.data    #984064
		rospy.loginfo("Got costmap")
		# import pdb
		# pdb.set_trace()
		map_array_xy = np.array(self.map_data_).reshape(self.map_size_x_, self.map_size_y_) #
		free_idx_arr = np.argwhere(map_array_xy==0)        #[m,2]
		obstacle_idx_arr = np.argwhere(map_array_xy==100)  #[n,2]

		dist_idx = np.reshape(np.sum(free_idx_arr**2,axis=1),(free_idx_arr.shape[0],1))+ np.sum(obstacle_idx_arr**2,axis=1) - 2*free_idx_arr.dot(obstacle_idx_arr.T) #(14356, 1429)
		dist_idx = dist_idx.min(1)
		radius_cells = self.robot_radius_ / self.map_resolution_
		in_mask = (dist_idx > radius_cells) * (dist_idx > self.distance_threshold_)
		self.free_idx_arr = free_idx_arr[in_mask]

		if len(self.debug):
			debug_arr = np.zeros_like(map_array_xy)
			debug_arr[map_array_xy==0] = 255
			debug_arr[self.free_idx_arr[:,0], self.free_idx_arr[:,1]] = 125
			cv2.imwrite(os.path.join(self.debug, "debug_img.png"), debug_arr)

		import pdb
		pdb.set_trace()

	def generateTatget(self):

		grid_x, grid_y = self.free_idx_arr[random.randint(0, len(self.free_idx_arr))]

		assert self.map_data_[grid_x*self.map_size_x_ + grid_y] == 0, "wrong position!"

		rnd_heading_bin = random.randint(1, self.heading_bins_)
		tgt_angle = ((np.pi*2) / self.heading_bins_) * rnd_heading_bin

		if tgt_angle >= np.pi:
			tgt_angle -= (np.pi * 2.0)
		elif tgt_angle < -np.pi:
			tgt_angle += (np.pi * 2.0)
		#createQuaternionFromYaw
		yaw = tgt_angle
		quaternion  = tf.transformations.quaternion_from_euler(0, 0, yaw)

		# mapToWorld
		world_x = self.map_origin_x_ + (grid_x + 0.5) * self.map_resolution_;
		world_y = self.map_origin_y_ + (grid_y + 0.5) * self.map_resolution_;

		M = Point(world_x, world_y, 0)
		N = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]) 
		self.goal.target_pose.pose = Pose(M, N)
		self.goal.target_pose.header.frame_id = 'map'  
		self.goal.target_pose.header.stamp = rospy.Time.now()


if __name__ == '__main__':  
	try:  
		random_navi()   
	except rospy.ROSInterruptException:  
		rospy.loginfo("random navigation failed.")
