#!/usr/bin/env python
import rospy
import cv2
from cv2 import aruco
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_matrix, quaternion_from_matrix
import tf


#Auxiliar function to print a homogeneous matrix
def print_matrix(H):
	print("%.3f  %.3f  %.3f    %.3f" % (H[0,0],H[0,1],H[0,2], H[0,3]))
	print("%.3f  %.3f  %.3f    %.3f" % (H[1,0],H[1,1],H[1,2], H[1,3]))
	print("%.3f  %.3f  %.3f    %.3f\n" % (H[2,0],H[2,1],H[2,2], H[2,3])) 


# Box detection class
class box_detection:
	def __init__(self):
		self.dictionary = aruco.Dictionary_get(aruco.DICT_4X4_50)
		self.parameters =  aruco.DetectorParameters_create()
		self.parameters.doCornerRefinement = True # default: False (check version of cv2)
		self.parameters.cornerRefinementMinAccuracy = 0.05 # default: 0.1 (check version of cv2)
		self.parameters.cornerRefinementWinSize = 4 # default: 5 (check version of cv2)
		#Reference: https://docs.opencv.org/3.1.0/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html#accaae2e7f4bf01483e3432adbc5ffa83
		self.intrinsic_matrix = 0
		self.distortion_matrix = 0
		self.corners = 0
		# Position and quaternion (pq - [x, y, z,  qx, qy, qz, qw])
		self.pq_b_r = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0] # box center (b) with respect to robot (r)
		self.pq_c_w = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0] # box ccamera (c) with respect to world (w)
		# Equivalent homogeneous matrices
		self.H_b_r = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
		self.H_c_w = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]

		# Init ros node
		nodename = "detection"
		rospy.init_node(nodename, anonymous=True)

		#Load parameters
		try:
			aruco_size = rospy.get_param("/"+nodename+"/aruco_size")
			box_size = rospy.get_param("/"+nodename+"/box_size")
			image_topic = rospy.get_param("/"+nodename+"/image_topic")
			pose_topic = rospy.get_param("/"+nodename+"/pose_topic")

			self.pq_b_r = rospy.get_param("/"+nodename+"/pq_b_r")
			self.pq_c_w = rospy.get_param("/"+nodename+"/pq_c_w")
			self.pq_b_b0 = rospy.get_param("/"+nodename+"/pq_b_b0")
			
			# Compute homognious transformations from the positions and quaternions given
			self.H_b_r = quaternion_matrix(self.pq_b_r[3:7])
			self.H_b_r[0][3] = self.pq_b_r[0]
			self.H_b_r[1][3] = self.pq_b_r[1]
			self.H_b_r[2][3] = self.pq_b_r[2]
			self.H_b_r = np.array(self.H_b_r)
			self.H_c_w = quaternion_matrix(self.pq_c_w[3:7])
			self.H_c_w[0][3] = self.pq_c_w[0]
			self.H_c_w[1][3] = self.pq_c_w[1]
			self.H_c_w[2][3] = self.pq_c_w[2]
			self.H_c_w = np.array(self.H_c_w)
			self.H_b_b0 = quaternion_matrix(self.pq_b_b0[3:7])
			self.H_b_b0[0][3] = self.pq_b_b0[0]
			self.H_b_b0[1][3] = self.pq_b_b0[1]
			self.H_b_b0[2][3] = self.pq_b_b0[2]
			self.H_b_b0 = np.array(self.H_b_b0)

			# Print the read parameters
			print "\n\33[92mParameters loaded\33[0m"
			print "\33[94maruco_size: ", aruco_size,"\33[0m"
			print "\33[94mbox_size: ", box_size,"\33[0m"
			print "\33[94mimage_topic: ", image_topic,"\33[0m"
			print "\33[94mpose_topic: ", pose_topic,"\33[0m"
			print "\33[94mpq_b_r: ", self.pq_b_r,"\33[0m"
			print "\33[94mpq_c_w: ", self.pq_c_w,"\33[0m"
			print "\33[94mpq_b_b0: ", self.pq_b_b0,"\33[0m"
		except:
			print "\33[41mProblem occurred when trying to read the parameters!\33[0m"
			print "\33[41mNode detect.py\33[0m"

		# Subscribe to image
		rospy.Subscriber(image_topic+"image_raw", Image, self.callback_image)
		rospy.Subscriber(image_topic+"camera_info", CameraInfo, self.callback_camera_info)
		# Publisher for image with detected markers
		self.pub_draw = rospy.Publisher("/aruco_image", Image, queue_size=1)
		# Publishers for some poses
		self.pub_box_pose = rospy.Publisher(pose_topic+"/center", Pose, queue_size=1)
		self.pub_box_pose_raw = rospy.Publisher(pose_topic+"/raw", Pose, queue_size=1)
		self.pub_robot_pose = rospy.Publisher(pose_topic+"/robot", Pose, queue_size=1)
		# Publisher for a marker
		self.pub_marker = rospy.Publisher("box_marker", Marker, queue_size=1)


		# Keep a loop only to send astatic transform
		rate = rospy.Rate(30)
		while not rospy.is_shutdown():

			# Publish a transform between the camera and the world
			br = tf.TransformBroadcaster()
			br.sendTransform((self.pq_c_w[0], self.pq_c_w[1], self.pq_c_w[2]),
				(self.pq_c_w[3], self.pq_c_w[4], self.pq_c_w[5], self.pq_c_w[6]),
				rospy.Time.now(),
				"camera",
				"world")

			rate.sleep()
			
		#rospy.spin()



	# Callback to get the image
	def callback_image(self, data):

		# Convert image to opencv
		bridge = CvBridge()
		image = bridge.imgmsg_to_cv2(data, desired_encoding='rgb8')

		# # Filter the image
		# kernel = np.array([[1.0, 1.4, 1.0], 
		#                    [1.4, 2.0, 1.4], 
		#                    [1.0, 1.4, 1.0]])
		# kernel = kernel/sum(sum(kernel))
		# filtered_image = cv2.filter2D(image,-1,kernel,borderType=cv2.BORDER_CONSTANT)
		filtered_image = image

		# Get a gray scale image
		gray_image = cv2.cvtColor(filtered_image, cv2.COLOR_BGR2GRAY)

		# Detect the corners of the markers
		self.corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_image, self.dictionary, parameters=self.parameters)

		#Create and publish an image with the deteted corners
		frame_markers = aruco.drawDetectedMarkers(filtered_image.copy(), self.corners, ids)
		#Put corner ids
		if ids is not None:
			for i in range(ids.size):
				for k in [1,2,3,4]:
					pixel2d = self.corners[i].tolist()[0]
					cv2.putText(frame_markers, str(k), (int(pixel2d[k-1][0]), int(pixel2d[k-1][1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 120, 0), 1)
		pub_image = bridge.cv2_to_imgmsg(frame_markers, encoding="rgb8")
		self.pub_draw.publish(pub_image)

		# If at leas one ArUco is detected, compute the pose
		if ids is not None:
			self.compute_pose(ids)
		

		



	# Callback to get the camera calibration info
	def callback_camera_info(self, data):
		self.intrinsic_matrix = np.array([[data.K[0],data.K[1],data.K[2]],[data.K[3],data.K[4],data.K[5]],[data.K[6],data.K[7],data.K[8]]], dtype = "double")
		self.distortion_matrix = np.array([[data.D[0]],[data.D[1]],[data.D[2]],[data.D[3]],[data.D[4]]], dtype = "double")


	# Function to compute the pose of the box
	def compute_pose(self, ids):


		box3d = [] #list of physical points in 3d
		pixel2d = [] #list of corner image points in 2d

		# For each detected ArUco
		for k in range(0,ids.size):

			# Get the associated 3d points in the "corner frame" according to the aruco id
			if (ids[k] == 1):
				# pts3d = [[0.0,6.7,80.7],[0.0,7.8,8.3],[0.0,77.5,6.7],[0.0,78.7,80.1]] #small wood
				pts3d = [[16.1,15.7,0.0],[2.5,15.8,0.0],[2.5,2.2,0.0],[16.0,2.1,0.0]] #big polystyrene
				pts3d = (np.array(pts3d)/100.0).tolist()
			elif (ids[k] == 2):
				# pts3d = [[9.5,79.8,0.0],[10.1,9.8,0.0],[83.6,8.7,0.0],[84,80.4,0.0]]  #small wood
				pts3d = [[16.0,0.0,2.4],[2.5,0.0,2.5],[2.2,0.0,16.1],[15.9,0.0,15.9]] #big polystyrene
				pts3d = (np.array(pts3d)/100.0).tolist()
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
				print "\33[93mAruco out of range 1-6 detected\33[0m"
				continue

			# Concatenate the points of all detected ArUcos
			pixel2d = pixel2d + self.corners[k].tolist()[0]
			box3d = box3d + pts3d

		# Execute PnP and get the position of the corner frame (b0) with respect to the camera frame (c)
		succes, rvecs, tvecs = cv2.solvePnP(np.array(box3d), np.array(pixel2d), self.intrinsic_matrix, self.distortion_matrix, flags=cv2.SOLVEPNP_ITERATIVE)
		# Transform the result into a homogeneous matrix H_b0_c
		rot_mat, _ = cv2.Rodrigues(rvecs) 
		R_ = np.concatenate((rot_mat,tvecs), axis=1 )
		H_b0_c = np.concatenate((R_,np.array([[0,0,0,1]])), axis = 0) # box corner (b0) with respect to camera (c)

		#Compute pose of the box center (b) with respect to the world (w)
		#H_b_w = H_c_w * H_b0_c * H_b_b0
		H_b_c = H_b0_c.dot(self.H_b_b0) # box center (b) with respect to camera (c)
		H_b_w = self.H_c_w.dot(H_b_c) # box center (b) with respect to world (w)

		#Compute pose of the robot with respect to the center
		H_r_b = np.linalg.inv(self.H_b_r) # robot (r) with respect to box center (b)
		H_r_w = H_b_w.dot(H_r_b) # robot (r) with respect to world (w)

		# print "H_b0_c"
		# print_matrix(H_b0_c)
		# print "H_b_b0"
		# print_matrix(self.H_b_b0)
		# print "H_c_w"
		# print_matrix(H_c_w)
		# print "H_b_w"
		# print_matrix(H_b_w)


		# Starting publication of messages ...

		# Publish box pose with respect to the camera (corner frame)
		pose_raw = Pose()
		quat_raw = quaternion_from_matrix(H_b0_c)
		pose_raw.position.x = H_b0_c[0][3]
		pose_raw.position.y = H_b0_c[1][3]
		pose_raw.position.z = H_b0_c[2][3]
		pose_raw.orientation.x = quat_raw[0]
		pose_raw.orientation.y = quat_raw[1]
		pose_raw.orientation.z = quat_raw[2]
		pose_raw.orientation.w = quat_raw[3]
		self.pub_box_pose_raw.publish(pose_raw)

		# Publish box pose with respect to the world (center frame)
		pose_w = Pose()
		quat_w = quaternion_from_matrix(H_b_w)
		pose_w.position.x = H_b_w[0][3]
		pose_w.position.y = H_b_w[1][3]
		pose_w.position.z = H_b_w[2][3]
		pose_w.orientation.x = quat_w[0]
		pose_w.orientation.y = quat_w[1]
		pose_w.orientation.z = quat_w[2]
		pose_w.orientation.w = quat_w[3]
		self.pub_box_pose.publish(pose_w)

		# Publish robot pose with respect to the world
		pose_rob = Pose()
		quat_rob = quaternion_from_matrix(H_r_w)
		pose_rob.position.x = H_r_w[0][3]
		pose_rob.position.y = H_r_w[1][3]
		pose_rob.position.z = H_r_w[2][3]
		pose_rob.orientation.x = quat_rob[0]
		pose_rob.orientation.y = quat_rob[1]
		pose_rob.orientation.z = quat_rob[2]
		pose_rob.orientation.w = quat_rob[3]
		self.pub_robot_pose.publish(pose_rob)


		# Piblish cube marker to be visualized on rviz
		marker = Marker()
		marker.header.frame_id = "/world"
		marker.header.stamp = rospy.Time.now()
		marker.id = 1
		marker.type = marker.CUBE
		marker.action = marker.ADD
		# Size of sphere
		marker.scale.x = 0.18
		marker.scale.y = 0.18
		marker.scale.z = 0.18
		# Color and transparency
		marker.color.a = 0.7
		marker.color.r = 0.99
		marker.color.g = 0.99
		marker.color.b = 0.99
		# Pose
		marker.pose.orientation.x = quat_w[0]
		marker.pose.orientation.y = quat_w[1]
		marker.pose.orientation.z = quat_w[2]
		marker.pose.orientation.w = quat_w[3]
		marker.pose.position.x = H_b_w[0][3]
		marker.pose.position.y = H_b_w[1][3]
		marker.pose.position.z = H_b_w[2][3]
		self.pub_marker.publish(marker)


		#Publish transformations
		br = tf.TransformBroadcaster()
		br.sendTransform((H_b0_c[0][3], H_b0_c[1][3], H_b0_c[2][3]),
			quat_raw,
			rospy.Time.now(),
			"box",
			"camera")
		br.sendTransform((0.09, 0.09, 0.09),
			(0.0, 0.0, 0.0, 1.0,),
			rospy.Time.now(),
			"center",
			"box")
		br.sendTransform((H_r_w[0][3], H_r_w[1][3], H_r_w[2][3]),
			(quat_rob[0], quat_rob[1], quat_rob[2], quat_rob[3]),
			rospy.Time.now(),
			"robot",
			"world")




# Main
if __name__ == '__main__':

	try:
		Detect_box = box_detection()		
	except rospy.ROSInterruptException:
		pass



