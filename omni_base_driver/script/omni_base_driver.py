#!/usr/bin/python
import rospy
import tf
import time
import sys
import math
from operator import add
import serial
import string
import threading
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
#from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from omni_serial_com import OmniSerialCom

class Omni_base_node:
	def __init__(self):
		#======== Get params =========#
		self.param = {}			# create a dictionary
		#self.debug_mode = bool(rospy.get_param('~debug_mode', False)) # true for detail info        
		self.param["vel_cmd"] = rospy.get_param("~cmd_vel", 'cmd_vel')
		self.param["odom_topic"] = rospy.get_param("~odom_topic", 'odom')
		self.param["imu_topic"] = rospy.get_param("~imu_topic", 'imu')
		self.param["baseId"] = rospy.get_param('~base_id', 'base_link') # base frame id
		self.param["odomId"] = rospy.get_param('~odom_id', 'odom') # odom frame id
		self.param["imuId"] = rospy.get_param('~imu_id', 'imu') # odom frame id
		self.param["enable_tf"] = rospy.get_param("~enable_tf", True)
		self.param["device_port"] = rospy.get_param('~port', '/dev/ttyS2') # port
		self.param["baudrate"] = int( rospy.get_param('~baudrate', '115200') ) # baudrate
		self.param["timeout"] = float( rospy.get_param('~serial_timeout', '10') ) #
		self.param["odom_freq"] = float( rospy.get_param('~odom_freq', '10') ) # hz of communication
		self.param["imu_freq"] = float( rospy.get_param('~imu_freq', '100') )  # hz of communication
		self.param["tx_freq"] = float( rospy.get_param('~tx_freq', '5') )      # hz of communication
		self.param["cmd_timeout"] = float( rospy.get_param('~cmd_vel_timeout', '3') ) #
		self.param["vel_gain"] = float( rospy.get_param('~vel_gain', '70') ) # to match physical world
		self.param["omg_gain"] = float( rospy.get_param('~omg_gain', '500') ) #
		
		rospy.set_param("omni_base_driver", self.param)
		
		#========== ROS message ==========#
		rospy.loginfo("Initiating Node")
		rospy.loginfo("Publishing odometry at: \"" + self.param["odom_topic"] + "\"")
		rospy.loginfo("Publishing imu at: \"" + self.param["imu_topic"] + "\"")
		rospy.loginfo("Subscribing to \"" + self.param["vel_cmd"] + "\"")
		
		self.data_handle_ok = False		# prevent node from starting pre maturely
		
		#========== ROS handler ==========#
		self.enc_sub = rospy.Subscriber(self.param["vel_cmd"], Twist, self.velcmd_cb, queue_size=10) 
		self.odom_pub = rospy.Publisher(self.param["odom_topic"], Odometry, queue_size=10)		# pubisher
		self.imu_pub = rospy.Publisher(self.param["imu_topic"], Imu, queue_size=10)
		#self.timer_odom = rospy.Timer(rospy.Duration(1.0/self.param["odom_freq"]), self.odomPub) # timer
		#self.timer_imu = rospy.Timer(rospy.Duration(1.0/self.param["imu_freq"]), self.imuPub)
		self.timer_txcmd = rospy.Timer( rospy.Duration(1.0/self.param["tx_freq"]),
														self.tx_vel_cmd)
		
		if self.param["enable_tf"]:
			rospy.loginfo("Publishing transform from %s to %s", self.param["odomId"], self.param["baseId"])
			self.tf_br = tf.TransformBroadcaster()

		#========== variable for ROS publisher ==========#
		self.odom = Odometry()
		self.odom.header.seq = 0
		self.odom.header.frame_id = self.param["odomId"]
		self.odom.child_frame_id = self.param["baseId"]
		for i in range(36):
			self.odom.pose.covariance[i] = 0
			self.odom.twist.covariance[i] = 0
		self.odom.twist.covariance[0] = 1.0
		self.odom.twist.covariance[35] = 1.0
		
		self.imu = Imu()
		self.imu.header.seq = 0
		self.imu.header.frame_id = self.param["imuId"]
		self.imu.orientation.x = 0
		self.imu.orientation.y = 0
		self.imu.orientation.z = 0
		self.imu.orientation.w = 0
		for i in range(9):
			self.imu.orientation_covariance[i] = 0
			self.imu.angular_velocity_covariance[i] = 0
			self.imu.linear_acceleration_covariance[i] = 0
		self.imu.orientation_covariance[0] = -1
		self.imu.angular_velocity_covariance[0] = math.radians(0.05)**2		# P.12 of the MPU6050 datasheet
		self.imu.angular_velocity_covariance[4] = math.radians(0.05)**2
		self.imu.angular_velocity_covariance[8] = math.radians(0.05)**2
		self.imu.linear_acceleration_covariance[0] = (400*10**-6*9.81)**2		# in m/s**2
		self.imu.linear_acceleration_covariance[4] = (400*10**-6*9.81)**2
		self.imu.linear_acceleration_covariance[8] = (400*10**-6*9.81)**2
		
		self.x_e = float(0)
		self.y_e = float(0)
		self.th = float(0)
		self.odom_last_seq = 0;
		self.enc_count_per_revo = (390*4)
		self.wheel_radius = 0.029
		self.base_radius = 0.140 		# 14.3 cm radius
		self.accel_sensitivity = 1.8*9.81	# 2g
		self.gyro_sensitivity = math.radians(250) 	# 250deg/sec
		
		self.last_odom_time = time.time()+3		# 3 sec for VDMC initialization
		self.last_cmd_vel_time = self.last_odom_time
		self.last_imu_time = self.last_odom_time
		self.odom_timeout = 1/self.param["odom_freq"] * 3
		self.imu_timeout = 1/self.param["imu_freq"] * 10
		
		self.no_cmd_received = True
		self.first_odom = True
		
		self.veh_cmd = {"Vx":0, "Vy":0, "Omega":0}
	
	'''*******************************************************************
		initial: recieve serial port handle from serial thread
	*******************************************************************'''
	def set_data_port(self, _serial_handle):
		self.data_handle = _serial_handle
		self.data_handle_ok = True

	def spin_listen(self):
		time.sleep(4) #wait for serial to boot
		try:
			while not self.data_handle_ok:	# block before serial is alive
				print("no serial handler")
				time.sleep(1)
			while not self.data_handle.serialOK():
				print("serial not OK")
				time.sleep(1)
			

			while True:
				self.odomPub()
				self.imuPub()
				time.sleep(1.0/self.param["imu_freq"])
		except KeyboardInterrupt:
			print("ctrlC kill")

	'''*******************************************************************
		ROS cmd_vel Subscriber callback
	*******************************************************************'''
	def velcmd_cb(self, vel_cmd):
		self.veh_cmd["Vx"] = vel_cmd.linear.x * self.param["vel_gain"]
		self.veh_cmd["Vy"] = vel_cmd.linear.y * self.param["vel_gain"]
		self.veh_cmd["Omega"] = vel_cmd.angular.z * self.param["omg_gain"]
		self.no_cmd_received = False
		self.last_cmd_vel_time = time.time()
		
	'''*******************************************************************
		send velocity command to serial handler
	*******************************************************************'''
	def tx_vel_cmd(self, event):
		if self.data_handle_ok:
			if( (time.time()-self.last_cmd_vel_time) > self.param["cmd_timeout"] and 
															not self.no_cmd_received ):
				self.veh_cmd["Vx"] = 0
				self.veh_cmd["Vy"] = 0
				self.veh_cmd["Omega"] = 0
				self.no_cmd_received = True
				rospy.logwarn("no topic \"%s\" received, base stop.", self.param["vel_cmd"])
				
			cmd = [ int(self.veh_cmd["Vx"]), int(self.veh_cmd["Vy"]), int(self.veh_cmd["Omega"]) ]
			self.data_handle.send_vel_cmd(cmd)
	

	'''*******************************************************************
		ROS Odometry topic publisher, called by timer
	*******************************************************************'''	
	def odomPub(self):
		if self.data_handle.odom_new_data():
			xyt_rtn = self.data_handle.get_odom_data()
			if xyt_rtn is None:
				return
			# sanity check
			if (xyt_rtn["seq"] != ((self.odom_last_seq + 1)%256) ):
				if (not self.first_odom):
					rospy.logwarn("odom callback sequence mismatch, prev: %d, now:%d", 
													self.odom_last_seq, xyt_rtn["seq"] )
					self.odom_last_seq = xyt_rtn["seq"]
					return
				else:
					self.first_odom = False
					self.odom_last_seq = xyt_rtn["seq"]
					self.last_odom_time = time.time();
					return		# disregard first data
			self.odom_last_seq = xyt_rtn["seq"]
			dx_e = float(xyt_rtn["pos_dt"][0])/10000
			dy_e = float(xyt_rtn["pos_dt"][1])/10000
			d_th = float(xyt_rtn["pos_dt"][2])/10000
			vx = dx_e * math.cos(self.th + d_th/2) + dy_e * math.sin(self.th + d_th/2)
			vy = -dx_e * math.sin(self.th + d_th/2) + dy_e * math.cos(self.th + d_th/2)
			time_now = time.time()
			self.x_e += dx_e
			self.y_e += dy_e
			self.th += d_th
			dt = time_now - self.last_odom_time
			
			#======= ROS topic publisher =======#
			self.odom.header.seq += 1
			self.odom.header.stamp = rospy.Time.now()
			self.odom.pose.pose.position = geometry_msgs.msg.Point(self.x_e, self.y_e, 0.0)
			quat = tf.transformations.quaternion_from_euler(0, 0, self.th);
			self.odom.pose.pose.orientation.x = quat[0]
			self.odom.pose.pose.orientation.y = quat[1]
			self.odom.pose.pose.orientation.z = quat[2]
			self.odom.pose.pose.orientation.w = quat[3]
			self.odom.twist.twist.linear = Vector3(vx / dt, vy / dt, 0.0)
			self.odom.twist.twist.angular = Vector3(0.0, 0.0, d_th/dt)
			
			self.odom_pub.publish(self.odom)
			
			if self.param["enable_tf"]:
				self.tf_br.sendTransform((self.x_e, self.y_e, 0),
								quat,
								self.odom.header.stamp,
								self.param["baseId"],
								self.param["odomId"]			)
			
			self.last_odom_time = time_now;
		elif self.first_odom:
			if (time.time() - self.last_odom_time)>5:
				rospy.logerr('First odom not arrived %f seconds',(time.time() - self.last_odom_time))
				self.last_odom_time = time.time()
			return
		elif (time.time() - self.last_odom_time)>self.odom_timeout:
			# if no new_data
			self.odom.header.seq += 1
			self.odom.header.stamp = rospy.Time.now()
			self.odom.twist.twist.linear = Vector3(0.0, 0.0, 0.0)
			self.odom.twist.twist.angular = Vector3(0.0, 0.0, 0.0)
			self.odom_pub.publish(self.odom)
			rospy.logerr('odom not available for %f seconds',(time.time() - self.last_odom_time))
			self.last_odom_time = time.time();
	
	'''*******************************************************************
		ROS Imu topic publisher, called by timer
	*******************************************************************'''	
	def imuPub(self):
		if self.data_handle.imu_new_data():
			imu_rtn = self.data_handle.get_imu_data()
			if imu_rtn is None:
				return
			self.imu.linear_acceleration.x = imu_rtn["accel"][0] * self.accel_sensitivity / 32768
			self.imu.linear_acceleration.y = imu_rtn["accel"][1] * self.accel_sensitivity / 32768
			self.imu.linear_acceleration.z = imu_rtn["accel"][2] * self.accel_sensitivity / 32768
			self.imu.angular_velocity.x = imu_rtn["gyro"][0] * self.gyro_sensitivity / 32768
			self.imu.angular_velocity.y = imu_rtn["gyro"][1] * self.gyro_sensitivity / 32768
			self.imu.angular_velocity.z = imu_rtn["gyro"][2] * self.gyro_sensitivity / 32768
			
			self.imu.header.seq += 1
			self.imu.header.stamp = rospy.Time.now()

			self.imu_pub.publish(self.imu)
			self.last_imu_time = time.time() 
		elif (time.time() - self.last_imu_time)>self.imu_timeout : 
			#if new_data
			rospy.logerr('imu data not available for %f seconds',(time.time() - self.last_imu_time))
			self.last_imu_time = time.time();
			
		
if __name__ == "__main__":
   
	# ROS Init    
	rospy.init_node('serial_odom', anonymous=True, disable_signals=True)
	node = Omni_base_node()
	#rospy.loginfo("Serial odometry starting")
	try:
		omni_com = OmniSerialCom(	port      = node.param["device_port"] ,
									baudrate  = node.param["baudrate"]   ,
									imu_freq  = node.param["imu_freq"]    ,
									odom_freq = node.param["odom_freq"]   ,
									timeout   = node.param["timeout"]      )
	except:
		print("Fail to create serial connection, shutting down")
		rospy.signal_shutdown("serial err")
		sys.exit(0)
	
	try:
		thread = threading.Thread(target=omni_com.serial_thread)
		#thread.daemon=True
		thread.start()
	except:
		rospy.signal_shutdown("serial thread err")
		omni_com.stopThread()
		sys.exit(0)

	node.set_data_port(omni_com)
	node.spin_listen()
	#rospy.spin() no need, use while loop in node
	
	print("SIGINT, stopping...")
	omni_com.stopThread()
	thread.join()
	rospy.signal_shutdown("user ends")
	print("Node ends")
