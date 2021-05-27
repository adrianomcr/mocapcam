#!/usr/bin/env python
import rospy
import cv2
from cv2 import aruco
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose

from tf.transformations import rotation_matrix, quaternion_from_matrix
import tf

def print_matrix(H):
	print("%.3f  %.3f  %.3f    %.3f" % (H[0,0],H[0,1],H[0,2], H[0,3]))
	print("%.3f  %.3f  %.3f    %.3f" % (H[1,0],H[1,1],H[1,2], H[1,3]))
	print("%.3f  %.3f  %.3f    %.3f\n" % (H[2,0],H[2,1],H[2,2], H[2,3])) 



class aruco_detection:
	def __init__(self):
		self.image = 0
		self.image_marker = 0
		self.gray_image = 0
		self.dictionary = aruco.Dictionary_get(aruco.DICT_4X4_50)
		self.parameters =  aruco.DetectorParameters_create()
		self.intrinsic_matrix = 0
		self.distortion_matrix = 0
		self.corners = 0
		self.rot_mat = 0
		# self.objp = np.array([[-W,-H,0.0], [W,-H,0.0], [W,H,0.0], [-W,H,0.0]])

		nodename = "detection"
		rospy.init_node(nodename, anonymous=True)



		#Load parameters
		try:
			aruco_size = rospy.get_param("/"+nodename+"/aruco_size")
			#self.aruco_pose = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
			image_topic = rospy.get_param("/"+nodename+"/image_topic")
			pose_topic = rospy.get_param("/"+nodename+"/pose_topic")
			print "\n\33[92mParameters loaded\33[0m"
			print "\33[94maruco_size: ", aruco_size,"\33[0m"
			#print "\33[94maruco_pose: ", self.aruco_pose,"\33[0m"
			print "\33[94mimage_topic: ", image_topic,"\33[0m"
			print "\33[94mpose_topic: ", pose_topic,"\33[0m"
		except:
			print "\33[41mProblem occurred when trying to read the parameters!\33[0m"
			print "\33[41mNode detection.py\33[0m"

		W = aruco_size/2.0
		H = aruco_size/2.0

		rospy.Subscriber(image_topic+"image_raw", Image, self.callback_image)
		rospy.Subscriber(image_topic+"camera_info", CameraInfo, self.callback_camera_info)
		self.pub_draw = rospy.Publisher("/aruco_image", Image, queue_size=1)
		self.pub_pose = rospy.Publisher(pose_topic, Pose, queue_size=1)
		self.pub_aruco_pose = rospy.Publisher(pose_topic+"/raw", Pose, queue_size=1)

		rospy.spin()




	def callback_image(self, data):
		bridge = CvBridge()
		self.image = bridge.imgmsg_to_cv2(data, desired_encoding='rgb8')
		self.gray_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
		self.corners, ids, rejectedImgPoints = aruco.detectMarkers(self.gray_image, self.dictionary, parameters=self.parameters)

		# print "corners: \n", self.corners[0].tolist()[0], "\n\n"
		# print "ids: \n", ids
		# if ids is not None:
		# 	for i in ids:
		# 		print "--- " + str(i)
		# # print "rejectedImgPoints: \n", rejectedImgPoints, "\n\n"
		# print "\n\n"
		
		

		frame_markers = aruco.drawDetectedMarkers(self.image.copy(), self.corners, ids)
		#Check corners ids
		if ids is not None:
			for i in range(ids.size):
				for k in [1,2,3,4]:
					# k = 4
					pixel2d = self.corners[i].tolist()[0]
					cv2.putText(frame_markers, str(k), (int(pixel2d[k-1][0]), int(pixel2d[k-1][1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
		pub_image = bridge.cv2_to_imgmsg(frame_markers, encoding="rgb8")

		if ids is not None:
			# self.pose_from_markers(ids) # old Victor
			self.compute_pose(ids) # old Victor
		

		self.pub_draw.publish(pub_image)




	def callback_camera_info(self, data):
		self.intrinsic_matrix = np.array([[data.K[0],data.K[1],data.K[2]],[data.K[3],data.K[4],data.K[5]],[data.K[6],data.K[7],data.K[8]]], dtype = "double")
		self.distortion_matrix = np.array([[data.D[0]],[data.D[1]],[data.D[2]],[data.D[3]],[data.D[4]]], dtype = "double")



	def compute_pose(self, ids):

		box3d = []
		pixel2d = []
		for k in range(0,ids.size):
			# print "id now: ", ids[k]
			pixel2d = pixel2d + self.corners[k].tolist()[0]

			if (ids[k] == 1):
				# pts3d = [[6.7,0.0,80.7],[7.8,0.0,8.3],[77.5,0.0,6.7],[78.7,0.0,80.1]]
				pts3d = [[0.0,6.7,80.7],[0.0,7.8,8.3],[0.0,77.5,6.7],[0.0,78.7,80.1]]
				pts3d = (np.array(pts3d)/1000.0).tolist()
			elif (ids[k] == 2):
				# pts3d = [[79.8,9.5,0.0],[9.8,10.1,0.0],[8.7,83.6,0.0],[80.4,84,0.0]]
				pts3d = [[9.5,79.8,0.0],[10.1,9.8,0.0],[83.6,8.7,0.0],[84,80.4,0.0]]
				pts3d = (np.array(pts3d)/1000.0).tolist()
				# continue
			elif (ids[k] == 3):
				continue
			else:
				continue

			box3d = box3d + pts3d


		succes, rvecs, tvecs = cv2.solvePnP(np.array(box3d), np.array(pixel2d), self.intrinsic_matrix, self.distortion_matrix, flags=cv2.SOLVEPNP_ITERATIVE)
		self.rot_mat, _ = cv2.Rodrigues(rvecs) 
		R_ = np.concatenate((self.rot_mat,tvecs), axis=1 )
		H_a_c = np.concatenate((R_,np.array([[0,0,0,1]])), axis = 0)

		print "H_a_c"
		print_matrix(H_a_c)

		# print "pixel2d: ", pixel2d
		# print "box3d: ", box3d

		# Computes aruco pose
		pose_aruco = Pose()
		quat_aruco = quaternion_from_matrix(H_a_c)
		pose_aruco.position.x = H_a_c[0][3]
		pose_aruco.position.y = H_a_c[1][3]
		pose_aruco.position.z = H_a_c[2][3]
		pose_aruco.orientation.x = quat_aruco[0]
		pose_aruco.orientation.y = quat_aruco[1]
		pose_aruco.orientation.z = quat_aruco[2]
		pose_aruco.orientation.w = quat_aruco[3]

		self.pub_aruco_pose.publish(pose_aruco)

		#Publish tf
		br = tf.TransformBroadcaster()
		br.sendTransform((H_a_c[0][3], H_a_c[1][3], H_a_c[2][3]),
			quat_aruco,
			rospy.Time.now(),
			"box",
			"camera")



	def pose_from_markers(self, ids):
		for i in range(0,ids.size):
			#print("H_a_c:")
			#print_matrix(H_a_c)
			
			if (ids[i] == 0):
				W = 0.8/2.0
				H = 0.8/2.0
			elif(ids[i] == 1):
				W = 0.8/2.0
				H = 0.8/2.0
			elif(ids[i] == 2):
				W = 0.09/2.0
				H = 0.09/2.0

			self.objp = np.array([[-W,-H,0.0], [W,-H,0.0], [W,H,0.0], [-W,H,0.0]])		

			succes, rvecs, tvecs = cv2.solvePnP(self.objp, self.corners[i][0], self.intrinsic_matrix, self.distortion_matrix, flags=cv2.SOLVEPNP_ITERATIVE)
			self.rot_mat, _ = cv2.Rodrigues(rvecs) 
			R_ = np.concatenate((self.rot_mat,tvecs), axis=1 )

			H_a_c = np.concatenate((R_,np.array([[0,0,0,1]])), axis = 0)
			H_c_d = np.array([[0,-1,0,0.1],[-1,0,0,0],[0,0,-1,-0.1],[0,0,0,1]])
			H_a2_w = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
			H_a_a2 = np.array([[0, -1, 0, 0],   [-1, 0, 0, 0],   [0, 0, -1, 0],   [0, 0, 0, 1]])
			H_a_w = H_a2_w.dot(H_a_a2)
			H_c_w = H_a_w.dot(np.linalg.inv(H_a_c))
			H_d_w = H_c_w.dot(np.linalg.inv(H_c_d))
				



			## FOra do if

			if (ids[i] <= np.min(ids)):
				# print("ID: %d" % ids[i])

				# Drone pose
				pose = Pose()
				quat = quaternion_from_matrix(H_d_w)
				pose.position.x = H_d_w[0][3]
				pose.position.y = H_d_w[1][3]
				pose.position.z = H_d_w[2][3]
				pose.orientation.x = quat[0]
				pose.orientation.y = quat[1]
				pose.orientation.z = quat[2]
				pose.orientation.w = quat[3]

				# Computes aruco pose
				pose_aruco = Pose()
				quat_aruco = quaternion_from_matrix(H_a_c)
				pose_aruco.position.x = H_a_c[0][3]
				pose_aruco.position.y = H_a_c[1][3]
				pose_aruco.position.z = H_a_c[2][3]
				pose_aruco.orientation.x = quat_aruco[0]
				pose_aruco.orientation.y = quat_aruco[1]
				pose_aruco.orientation.z = quat_aruco[2]
				pose_aruco.orientation.w = quat_aruco[3]
				
				self.pub_pose.publish(pose)
				self.pub_aruco_pose.publish(pose_aruco)

				




########### MAIN #####################
if __name__ == '__main__':

	try:
		Detect_markers = aruco_detection()		
	except rospy.ROSInterruptException:
		pass



