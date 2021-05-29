#!/usr/bin/env python
import rospy
import cv2
from cv2 import aruco
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
import copy
import time

from tf.transformations import quaternion_matrix

from tf.transformations import rotation_matrix, quaternion_from_matrix
import tf

def print_matrix(H):
	print("%.3f  %.3f  %.3f    %.3f" % (H[0,0],H[0,1],H[0,2], H[0,3]))
	print("%.3f  %.3f  %.3f    %.3f" % (H[1,0],H[1,1],H[1,2], H[1,3]))
	print("%.3f  %.3f  %.3f    %.3f" % (H[2,0],H[2,1],H[2,2], H[2,3]))
	print("%.3f  %.3f  %.3f    %.3f\n" % (0.0,0.0,0.0,1.0))



class aruco_detection:
	def __init__(self):
		self.image = 0
		self.image_marker = 0
		self.gray_image = 0
		self.dictionary = aruco.Dictionary_get(aruco.DICT_4X4_50)
		self.parameters =  aruco.DetectorParameters_create()
		self.parameters.doCornerRefinement = True # default: False (check version of cv2)
		self.parameters.cornerRefinementMinAccuracy = 0.05 # default: 0.1 (check version of cv2)
		self.parameters.cornerRefinementWinSize = 4 # default: 5 (check version of cv2)
		#Reference: https://docs.opencv.org/3.1.0/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html#accaae2e7f4bf01483e3432adbc5ffa83
		self.intrinsic_matrix = 0
		self.distortion_matrix = 0
		self.corners = 0
		self.rot_mat = 0
		self.pq_b_r = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
		self.pq_c_w = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
		self.H_b_r = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
		self.H_c_w = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
		self.do_computation = 0
		self.H_all = [];
		self.base_frame = ""
		# self.H_1 = [];
		# self.H_2 = [];

		self.points2d_acumulated = []
		self.points3d_acumulated = []

		nodename = "calibrator"
		rospy.init_node(nodename, anonymous=True)



		#Load parameters
		try:
			n_poses = rospy.get_param("/"+nodename+"/n_poses")
			n_poses = int(n_poses)

			for k in range(n_poses):
				try:
					H_now = rospy.get_param("/"+nodename+"/H_"+str(k+1))
					self.H_all.append(np.array(copy.deepcopy(H_now)))
				except:
					print "\33[41mParameter /"+nodename+"/H_"+str(k+1)+" was not provided\33[0m"
			

			# self.H_1 = rospy.get_param("/"+nodename+"/H_1")
			# self.H_2 = rospy.get_param("/"+nodename+"/H_2")
			# self.H_1 = np.array(self.H_1)
			# self.H_2 = np.array(self.H_2)


			# print "H_1 = ", H_1
			# print "H_2 = ", H_2


			self.pq_b_r = rospy.get_param("/"+nodename+"/pq_b_r")
			self.pq_b_b0 = rospy.get_param("/"+nodename+"/pq_b_b0")

			self.H_b_r = quaternion_matrix(self.pq_b_r[3:7])
			self.H_b_r[0][3] = self.pq_b_r[0]
			self.H_b_r[1][3] = self.pq_b_r[1]
			self.H_b_r[2][3] = self.pq_b_r[2]
			self.H_b_b0 = quaternion_matrix(self.pq_b_b0[3:7])
			self.H_b_b0[0][3] = self.pq_b_b0[0]
			self.H_b_b0[1][3] = self.pq_b_b0[1]
			self.H_b_b0[2][3] = self.pq_b_b0[2]

			self.H_b_r = np.array(self.H_b_r)
			self.H_b_b0 = np.array(self.H_b_b0)

			self.base_frame = rospy.get_param("/"+nodename+"/base_frame")

			image_topic = rospy.get_param("/"+nodename+"/image_topic")

		except:
			print "\33[41mProblem occurred when trying to read the parameters!\33[0m"
			print "\33[41mNode detect.py\33[0m"

			

		rospy.Subscriber(image_topic+"image_raw", Image, self.callback_image)
		rospy.Subscriber(image_topic+"camera_info", CameraInfo, self.callback_camera_info)
		self.pub_draw = rospy.Publisher("/aruco_image", Image, queue_size=1)


		time.sleep(1.0)
		print "\33[92mYou will be asked to place your box/robot in a series of places informed in \33[42mconfig/calibration.yaml\33[0m"
		rate = rospy.Rate(30)
		current_pose = 1
		while not rospy.is_shutdown():
			# H_now = eval("self.H_"+str(current_pose))
			H_now = self.H_all[current_pose-1]

			self.do_computation = 0
			print ""
			print "\33[94mPlace the robot at H_"+str(current_pose)+":\33[0m"
			print print_matrix(H_now)
			raw_input('Then press enter to continue: ')
			print ""
			self.do_computation = current_pose

			current_pose = current_pose + 1

			time.sleep(1)

			if current_pose > n_poses:
				break

			rate.sleep()


		


		self.solve_concatenated_PnP(self.points2d_acumulated, self.points3d_acumulated)
		
			
		rospy.spin()




	def solve_concatenated_PnP(self, p2d, p3d):

		# print "\n\33[92mself.points2d_acumulated:\33[0m\n ", self.points2d_acumulated
		# print "\33[92mself.points3d_acumulated:\33[0m\n ", self.points3d_acumulated

		p3d = np.array(self.points3d_acumulated) # written in the world frame
		p2d = np.array(self.points2d_acumulated)
		# print p3d_b0




		succes, rvecs, tvecs = cv2.solvePnP(p3d, p2d, self.intrinsic_matrix, self.distortion_matrix, flags=cv2.SOLVEPNP_ITERATIVE)
		self.rot_mat, _ = cv2.Rodrigues(rvecs) 
		R_ = np.concatenate((self.rot_mat,tvecs), axis=1 )
		H_w_c = np.concatenate((R_,np.array([[0,0,0,1]])), axis = 0)

		H_c_w = np.linalg.inv(H_w_c)
		quat_c_w = quaternion_from_matrix(H_c_w)



		print "\33[92m"
		print "\nResults:"
		print "H_c_w:"
		print_matrix(H_c_w)
		print ""
		print "pq_c_w: "
		print ("[%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]" % (H_c_w[0,3], H_c_w[1,3], H_c_w[2,3], quat_c_w[0], quat_c_w[1], quat_c_w[2], quat_c_w[3]))
		print "x, y, z,   qx, qy, qz, qw"
		print "\33[0m"


	def callback_image(self, data):

		if (self.do_computation == 0):
			return
		pose_id = self.do_computation
		self.do_computation = 0

		print "Capturing pose ", pose_id, "..."

		bridge = CvBridge()
		self.image = bridge.imgmsg_to_cv2(data, desired_encoding='rgb8')

		# Filter image
		kernel = np.array([[1.0, 1.4, 1.0], 
		                   [1.4, 2.0, 1.4], 
		                   [1.0, 1.4, 1.0]])
		kernel = kernel/sum(sum(kernel))
		filtered_image = self.image


		self.gray_image = cv2.cvtColor(filtered_image, cv2.COLOR_BGR2GRAY)

		self.corners, ids, rejectedImgPoints = aruco.detectMarkers(self.gray_image, self.dictionary, parameters=self.parameters)
		# print self.corners

		# frame_markers = aruco.drawDetectedMarkers(self.image.copy(), self.corners, ids)
		frame_markers = aruco.drawDetectedMarkers(filtered_image.copy(), self.corners, ids)
		#Check corners ids
		if ids is not None:
			for i in range(ids.size):
				for k in [1,2,3,4]:
					# k = 4
					pixel2d = self.corners[i].tolist()[0]
					cv2.putText(frame_markers, str(k), (int(pixel2d[k-1][0]), int(pixel2d[k-1][1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 120, 0), 1)
		pub_image = bridge.cv2_to_imgmsg(frame_markers, encoding="rgb8")
		self.pub_draw.publish(pub_image)


		# H_now = eval("self.H_"+str(pose_id))
		H_now = self.H_all[pose_id-1]
		if ids is not None:
			pixel2d, box3d = self.compute_pose(ids, H_now)
			# print "\n\npixel2d:", pixel2d
			# print "box3d:", box3d
			print ("\33[92mCaptured points: %d\33[0m" % (len(box3d)))
			self.points2d_acumulated = self.points2d_acumulated + pixel2d
			self.points3d_acumulated = self.points3d_acumulated + box3d
		else:
			print "\33[93mNo points captured\33[0m"

			
			# print "\n\nself.points2d_acumulated:", self.points2d_acumulated
			# print "self.points2d_acumulated:", self.points2d_acumulated
		


		print "\n----------  ----------\n"

		




	def callback_camera_info(self, data):
		self.intrinsic_matrix = np.array([[data.K[0],data.K[1],data.K[2]],[data.K[3],data.K[4],data.K[5]],[data.K[6],data.K[7],data.K[8]]], dtype = "double")
		self.distortion_matrix = np.array([[data.D[0]],[data.D[1]],[data.D[2]],[data.D[3]],[data.D[4]]], dtype = "double")



	def compute_pose(self, ids, H_base_w):

		# print "\33[91m"
		# print ids
		# print "\33[0m"

		box3d = []
		pixel2d = []
		for k in range(0,ids.size):
			# print "id now: ", ids[k]
			pixel2d = pixel2d + self.corners[k].tolist()[0]

			if (ids[k] == 1):
				# pts3d = [[0.0,6.7,80.7],[0.0,7.8,8.3],[0.0,77.5,6.7],[0.0,78.7,80.1]] #small wood
				pts3d = [[16.1,15.7,0.0],[2.5,15.8,0.0],[2.5,2.2,0.0],[16.0,2.1,0.0]] #big polystyrene
				pts3d = (np.array(pts3d)/100.0).tolist()
			elif (ids[k] == 2):
				# pts3d = [[9.5,79.8,0.0],[10.1,9.8,0.0],[83.6,8.7,0.0],[84,80.4,0.0]]  #small wood
				pts3d = [[16.0,0.0,2.4],[2.5,0.0,2.5],[2.2,0.0,16.1],[15.9,0.0,15.9]] #big polystyrene
				pts3d = (np.array(pts3d)/100.0).tolist()
				# continue
			elif (ids[k] == 3):
				pts3d = [[0.0,2.3,2.3],[0.0,15.9,2.4],[0.0,15.8,15.9],[0.0,2.1,15.8]] #big polystyrene
				pts3d = (np.array(pts3d)/100.0).tolist()
			elif (ids[k] == 4):
				pts3d = [[18.1,15.7,2.7],[18.1,2.0,2.6],[18.1,2.1,16.2],[18.1,15.8,16.3]] #big polystyrene
				pts3d = (np.array(pts3d)/100.0).tolist()
			elif (ids[k] == 5):
				pts3d = [[2.4,18.0,2.3],[16.0,18.0,2.5],[15.9,18.0,16.0],[2.3,18.0,15.9]] #big polystyrene
				pts3d = (np.array(pts3d)/100.0).tolist()
			elif (ids[k] == 6):
				pts3d = [[2.2,15.7,18.4],[15.9,15.7,18.4],[15.9,2.1,18.4],[2.2,2.1,18.4]] #big polystyrene
				pts3d = (np.array(pts3d)/100.0).tolist()
			else:
				return
				# continue

			box3d = box3d + pts3d




		box3d_world = []

		# print "H_b0_w"
		# print_matrix(H_base_w)
		#Write box3d in the world frame
		for k in range(len(box3d)):
			p_b0 = np.array( [[box3d[k][0]], [box3d[k][1]], [box3d[k][2]], [1]] )
			

			if(self.base_frame == "corner"):
				p_w = H_base_w.dot(p_b0)
			elif(self.base_frame == "center"):
				p_b = np.linalg.inv(self.H_b_b0).dot(p_b0)
				p_w = H_base_w.dot(p_b)
			elif(self.base_frame == "robot"):
				p_b = np.linalg.inv(self.H_b_b0).dot(p_b0)
				p_r = self.H_b_r.dot(p_b)
				p_w = H_base_w.dot(p_r)
				# print "\33[93mbase_frame as the robot is not implemeted yet\33[0m"
			else:
				print "\33[41mUnknown base_frame\33[0m"

			# p_w = H_base_w.dot(p_b0)
			# print k
			# print p_w
			box3d_world.append([p_w[0,0], p_w[1,0], p_w[2,0]])

		return (pixel2d, box3d_world)


		# succes, rvecs, tvecs = cv2.solvePnP(np.array(box3d), np.array(pixel2d), self.intrinsic_matrix, self.distortion_matrix, flags=cv2.SOLVEPNP_ITERATIVE)
		# self.rot_mat, _ = cv2.Rodrigues(rvecs) 
		# R_ = np.concatenate((self.rot_mat,tvecs), axis=1 )
		# H_b0_c = np.concatenate((R_,np.array([[0,0,0,1]])), axis = 0)





# if(self.base_frame == "corner"):
# 	H_c_w = np.linalg.inv(H_w_c)
# 	quat_c_w = quaternion_from_matrix(H_c_w)
# elif(self.base_frame == "center"):
# 	a=1
# elif(self.base_frame == "robot"):
# 	print "\33[93mbase_frame as the robot is not implemeted yet\33[0m"
# else:
# 	print "\33[41mUnknown base_frame\33[0m"






########### MAIN #####################
if __name__ == '__main__':

	try:
		Detect_box = aruco_detection()		
	except rospy.ROSInterruptException:
		pass



