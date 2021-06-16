# mocapcam
Motion capture system based on a single camera and a box of arucos

---------------------

This repository uses a single camera to detect the pose of a box (a cube) that has one ArUco on each of its faces. The ArUco corners are detected and used in a PnP algorithm to estimate the box's pose.
The camera should be fixed somewhere, so that the workspace of interest is on the image. The box will be easily detected since it has ArUcos in all faces. A calibrator code is provided to easily compute the pose of the fixed camera.

To use the, clone it to your catkin_ws/src folder:

```bash
$ git clone https://github.com/adrianomcr/mocapcam.git
```



### Requirements

To use a webcam, I recommend the usb_cam package (see `http://wiki.ros.org/usb_cam`). You can install it with:

```bash
$ sudo apt install ros-$ROS_DISTRO-usb-cam
```

You will need to get the parameters of your camera. I recommend to the package camera_calibration (see `http://wiki.ros.org/camera_calibration`). You can install it with:

```bash
$ sudo apt install ros-$ROS_DISTRO-camera-calibration
```

You will also need to construct an ArUco box with your preferred size.



### Box construction

The position of the ArUcos in the faces of the cube should be in agreement with the following image:

![image](https://github.com/adrianomcr/mocapcam/blob/main/images/box_pattern.png)

The parameter `B` is the size of the box and `A` the size of the ArUco. These parameters `aruco_size` (equivalent to `A`) and `box_size` (equivalent to `B`) are defined in the the launch files. The values should be passed in centimeters.

Currently, the code used the 4x4 ArUco dictionary (see `https://chev.me/arucogen/`). This can be modified in the script `detect.py`.



### Detection

First, it is necessary to configure the file `config/transforms.yaml`. In this file, the transforms are represented by lists with the 3 position and a quaternion in the form [x, y, z, qx, qy, qz, qw].

- `pq_b_b0`: Pose of a frame `F_b` with respect to the frame `F_b0`. Frame `F_b0` is placed in the corner of the box, as shown in the image above. Frame `F_b` is aligned with `F_b0` but is placed at the center of the box.

- `pq_b_r`: Pose of the center of the box with respect to the robot. This is useful when the box is attached to a robot and the pose of the robot is the desired measurement.

- `pq_c_w`: Pose of the camera with respect to the world. This data can be obtained with the calibrator code.

Finaly, to run the detection code use:

```bash
$ roslaunch mocapcam start_detection.launch
```



### Camera pose calibration

This package has a simple-to-use code to calibrate the pose of the camera with respect to the world frame. This world frame is chosen according to the user's wish. For instance, the corner of the floor of your room. In order to calibrate the camera position, it is necessary to place the box in some known poses and let the code know which poses are these. 

To configure the calibrator code, consider the file `config/calibration.yaml`. 

- `n_poses`: Set the number of poses (where the box will be placed) you will use to calibrate the camera pose. You can use a single pose, however, it will reduce the precision of the calibration. I recommend at least 3 not aligned poses.

- `base_frame`: Select the frame that that will be considered for calibration. The options are: `corner` to use the frame depicted in the image above; `center` to use the frame in the box's center; and `robot` to use the frame of the robot to which the box is attached to. The poses informed in the next parameters are the poses of the selected frame with respect to the world frame.

- `H_1`: Homogeneous transformation matrix that represents the pose of the frame "`base_frame`" with respect to the world in the first pose used in the calibration proccess.

- 'H_2': Keep adding `H_i` for each pose you will place the robot.

Finally, to run the calibrator code use:

```bash
$ roslaunch mocapcam start_detection.launch
``` 

It will ask you to place the box (or the robot) in the informed places and then hit enter. On the screen, it will print the calibration result. Copy it and place it on the file `config/transforms.yaml`.



### Illustration

If everything is ok, you should see something like this:

![image](https://github.com/adrianomcr/mocapcam/blob/main/images/illustration.png)


