#!/usr/bin/env python
import rospy
import cv2
from cv2 import aruco
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
import copy
import time
from tf.transformations import quaternion_matrix, quaternion_from_matrix
import tf


#Auxiliar function to print a homogeneous matrix
def print_matrix(H):
	print("%.3f  %.3f  %.3f    %.3f" % (H[0,0],H[0,1],H[0,2], H[0,3]))
	print("%.3f  %.3f  %.3f    %.3f" % (H[1,0],H[1,1],H[1,2], H[1,3]))
	print("%.3f  %.3f  %.3f    %.3f" % (H[2,0],H[2,1],H[2,2], H[2,3]))
	print("%.3f  %.3f  %.3f    %.3f\n" % (0.0,0.0,0.0,1.0))


# Camera calibrator class (pose calibration)
class camera_calibrator:
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
		self.pq_b_r = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
		self.pq_c_w = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
		self.H_b_r = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
		self.H_c_w = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
		self.do_computation = 0
		self.H_all = [];
		self.base_frame = ""

		self.points2d_acumulated = []
		self.points3d_acumulated = []

		# Init ros node
		nodename = "calibrator"
		rospy.init_node(nodename, anonymous=True)

		time.sleep(1.0)

		#Load parameters
		try:
			n_poses = rospy.get_param("/"+nodename+"/n_poses")
			n_poses = int(n_poses)

			# Read all homogeneous transformations
			for k in range(n_poses):
				try:
					H_now = rospy.get_param("/"+nodename+"/H_"+str(k+1))
					self.H_all.append(np.array(copy.deepcopy(H_now)))
				except:
					print "\33[41mParameter /"+nodename+"/H_"+str(k+1)+" was not provided\33[0m"
			
			self.pq_b_r = rospy.get_param("/"+nodename+"/pq_b_r")
			self.pq_b_b0 = rospy.get_param("/"+nodename+"/pq_b_b0")

			# Compute homognious transformations from the positions and quaternions given
			self.H_b_r = quaternion_matrix(self.pq_b_r[3:7])
			self.H_b_r[0][3] = self.pq_b_r[0]
			self.H_b_r[1][3] = self.pq_b_r[1]
			self.H_b_r[2][3] = self.pq_b_r[2]
			self.H_b_r = np.array(self.H_b_r)
			self.H_b_b0 = quaternion_matrix(self.pq_b_b0[3:7])
			self.H_b_b0[0][3] = self.pq_b_b0[0]
			self.H_b_b0[1][3] = self.pq_b_b0[1]
			self.H_b_b0[2][3] = self.pq_b_b0[2]
			self.H_b_b0 = np.array(self.H_b_b0)

			self.base_frame = rospy.get_param("/"+nodename+"/base_frame")

			aruco_size = rospy.get_param("/"+nodename+"/aruco_size")
			box_size = rospy.get_param("/"+nodename+"/box_size")
			image_topic = rospy.get_param("/"+nodename+"/image_topic")

			# Print the read parameters
			print "\n\33[92mParameters loaded\33[0m"
			print "\33[94maruco_size: ", aruco_size,"\33[0m"
			print "\33[94mbox_size: ", box_size,"\33[0m"
			print "\33[94mimage_topic: ", image_topic,"\33[0m"
			print "\33[94mn_poses: ", n_poses,"\33[0m"
			print "\33[94mbase_frame: ", self.base_frame,"\33[0m"
			print "\33[94mpq_b_r: ", self.pq_c_w,"\33[0m"
			print "\33[94mpq_b_b0: ", self.pq_b_b0,"\33[0m"

		except:
			print "\33[41mProblem occurred when trying to read the parameters!\33[0m"
			print "\33[41mNode detect.py\33[0m"

			
		# Subscribe to image
		rospy.Subscriber(image_topic+"image_raw", Image, self.callback_image)
		rospy.Subscriber(image_topic+"camera_info", CameraInfo, self.callback_camera_info)
		# Publisher for image with detected markers
		self.pub_draw = rospy.Publisher("/aruco_image", Image, queue_size=1)

		# Start the proccess of placing the box in the proper poses
		print "\n\33[92mYou will be asked to place your box/robot in a series of places informed in \33[42mconfig/calibration.yaml\33[0m"
		rate = rospy.Rate(30)
		# Start with the first pose
		current_pose = 1
		while not rospy.is_shutdown():

			# Current position where the box must be placed
			H_now = self.H_all[current_pose-1]
			# Set flag to get the data
			self.do_computation = 0
			# Wait the user to place the box (or the robot)
			print ""
			print "\33[94mPlace the robot at H_"+str(current_pose)+":\33[0m"
			print_matrix(H_now)
			raw_input('Then press enter to continue ...')
			print ""
			# Set the flag to enable the data to be captured in the next image
			self.do_computation = current_pose

			# Increment the pose id
			current_pose = current_pose + 1

			# Wait a bit
			time.sleep(1)

			# Check if all poses have been captured
			if current_pose > n_poses:
				break

			rate.sleep()


		
		# Now, all points are captured and the PnP can be solved by using all of them
		print ("Solving PnP with a total of %d points" % (len(self.points2d_acumulated)))
		print "\n----------  ----------\n"
		self.solve_concatenated_PnP(self.points2d_acumulated, self.points3d_acumulated)
		
		
		rospy.spin()



	# Function to solve the PnP with all captured points
	def solve_concatenated_PnP(self, p2d, p3d):


		p3d = np.array(self.points3d_acumulated) # 3d physical points written in the world frame
		p2d = np.array(self.points2d_acumulated) # 2d image points

		# Execute PnP and get the position of the world frame (w) with respect to the camera frame (c)
		succes, rvecs, tvecs = cv2.solvePnP(p3d, p2d, self.intrinsic_matrix, self.distortion_matrix, flags=cv2.SOLVEPNP_ITERATIVE)
		# Transform the result into a homogeneous matrix H_w_c
		rot_mat, _ = cv2.Rodrigues(rvecs) 
		R_ = np.concatenate((rot_mat,tvecs), axis=1 )
		H_w_c = np.concatenate((R_,np.array([[0,0,0,1]])), axis = 0)

		# Compute pose of the camera (c) with respect to the world (w)
		H_c_w = np.linalg.inv(H_w_c)
		# Get a quaternion representation of the orientation
		quat_c_w = quaternion_from_matrix(H_c_w)


		# Print the computed result
		print "\33[92m\nResults:\33m"
		print "H_c_w:\33[0m"
		print_matrix(H_c_w)
		print ""
		print "\33[92mpq_c_w:\33[0m"
		print ("[%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]" % (H_c_w[0,3], H_c_w[1,3], H_c_w[2,3], quat_c_w[0], quat_c_w[1], quat_c_w[2], quat_c_w[3]))
		print "x, y, z,   qx, qy, qz, qw"
		print ""
		print "\33[92mPlace this data in the file config/transforms.yaml\33[0m"
		print ("pq_c_w: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]" % (H_c_w[0,3], H_c_w[1,3], H_c_w[2,3], quat_c_w[0], quat_c_w[1], quat_c_w[2], quat_c_w[3]))





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


		#Pass if the user has not commanded to make the capture yet
		if (self.do_computation == 0):
			return

		#Get the number of the pose where the box is supposed to be
		pose_id = self.do_computation
		#Reset flag
		self.do_computation = 0

		print "Capturing pose ", pose_id, "..."

		# If at least one marker was detedcted, proceed and stode this data in a buffer
		H_now = self.H_all[pose_id-1]
		if ids is not None:
			#Get the associated 3d physical points
			pixel2d, box3d = self.get_points(ids, H_now)
			#Print the number of captured points
			print ("\33[92mCaptured %d points\33[0m" % (len(box3d)))
			# Apend to the the buffer
			self.points2d_acumulated = self.points2d_acumulated + pixel2d
			self.points3d_acumulated = self.points3d_acumulated + box3d
		else:
			print "\33[93mNo points captured\33[0m"


		print "\n----------  ----------\n"

		



	# Callback to get the camera calibration info
	def callback_camera_info(self, data):
		self.intrinsic_matrix = np.array([[data.K[0],data.K[1],data.K[2]],[data.K[3],data.K[4],data.K[5]],[data.K[6],data.K[7],data.K[8]]], dtype = "double")
		self.distortion_matrix = np.array([[data.D[0]],[data.D[1]],[data.D[2]],[data.D[3]],[data.D[4]]], dtype = "double")



	# Function to compute the physical corners of the arucos associated to each detected corner on the image
	def get_points(self, ids, H_base_w):

		box3d = [] #list of physical points in 3d
		pixel2d = [] #list of corner image points in 2d

		# For each detected ArUco
		for k in range(0,ids.size):

			# Get the associated 3d points in the "corner frame" according to the aruco id
			if (ids[k] == 1):
				pts3d = [[16.1,15.7,0.0],[2.5,15.8,0.0],[2.5,2.2,0.0],[16.0,2.1,0.0]] #big polystyrene
				pts3d = (np.array(pts3d)/100.0).tolist()
			elif (ids[k] == 2):
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
				continue

			
			# Concatenate the points of all detected ArUcos
			pixel2d = pixel2d + self.corners[k].tolist()[0]
			box3d = box3d + pts3d


		# The points box3d are in the corner frame (b0)
		# They need to be written in the world frame (w) by using the transformations provided in calibration.yaml
		box3d_world = []
		#For each point in box3d
		for k in range(len(box3d)):
			# Get the point written in the corner frame (b0)
			p_b0 = np.array( [[box3d[k][0]], [box3d[k][1]], [box3d[k][2]], [1]] )

			if(self.base_frame == "corner"):
				p_w = H_base_w.dot(p_b0) # point in the world frame (w)
			elif(self.base_frame == "center"):
				p_b = np.linalg.inv(self.H_b_b0).dot(p_b0) # point in the box center frame (b)
				p_w = H_base_w.dot(p_b) # point in the world frame (w)
			elif(self.base_frame == "robot"):
				p_b = np.linalg.inv(self.H_b_b0).dot(p_b0) # point in the box center frame (b)
				p_r = self.H_b_r.dot(p_b) # point in the robot frame (r)
				p_w = H_base_w.dot(p_r) # point in the world frame (w)
			else:
				print "\33[41mUnknown base_frame\33[0m"

			# append the points in the world frame
			box3d_world.append([p_w[0,0], p_w[1,0], p_w[2,0]])

		return (pixel2d, box3d_world)



# Main
if __name__ == '__main__':

	try:
		Detect_box = camera_calibrator()		
	except rospy.ROSInterruptException:
		pass



