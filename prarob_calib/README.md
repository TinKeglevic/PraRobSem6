# prarob_calib
Camera intrinsic and extrinsic calibration package for Robotics Practicum

## Prerequisites
To calibrate camera using ROS, you need to install image_pipeline package, which you can do using:

```bash
apt install ros-jazzy-image-pipeline
```

Next, you will need a calibration pattern. You can use whichever you like, the instructions written here are aimed towards the calibration file (_prarob_calib_sheet.pdf_) in the files folder. It is a checkerboard pattern, with dimensions _**7x9**_ and with square size (default print on A4 paper) of _**0.0186m**_.

If you are using a different pattern, make sure to provide correct dimensions (only the inner square connections count) and correct size of the square. 

## Instrinsic calibration
To calibrate the camera instrinsic parameters, we will use cameracalibrator node from camera_calibration package. We have provided a sample launch file that runs the camera and calibration simultaneously, so you can use:

```bash
ros2 launch prarob_calib camera_calib.launch.py
```

It will launch the cameracalibrator app (OpenCV GUI), showing the image and some additional info. If you have set up the checkerboard dimensions correctly, you will get colored circles depicting detected square corners. 

![calibration start](https://github.com/larics/prarob_calib/blob/main/files/calib_start.png "Calibration start")

Next, you need to move the calibration sheet until all progress bars (X, Y, size, skew) are green (the calibrate button will also light up).

![calibration ready](https://github.com/larics/prarob_calib/blob/main/files/calib_ready.png "Calibration ready")

When you press calibrate, you need to wait for other buttons to appear, signaling that the calibration is done. 

![calibration done](https://github.com/larics/prarob_calib/blob/main/files/calib_done.png "Calibration done")

Pressing save button will save the calibration data to `/tmp/calibrationdata.tar.gz`, pressing _Commit_ button should insert the calibration data into the _camera_info_ topic. 

Next, go to _tmp_ folder and extract the _calibrationdata.tar.gz_ file. There, you will find the images used to calibrate the camera and _ost.yaml_ file, that contains the calibration parameters. You should copy these into _camera_calibration_params.yaml_. That should be the final step in the calibration of the intrinsic parameters of your camera.

## Extrinsic calibration
To calculate the pose of the camera with respect to the checkerboard, you can launch _camera_world.launch.py_:

```bash
ros2 launch prarob_calib camera_world.launch.py
```

If the checkerboard is visible in the image, you should get a frame drawn on the top left corner, corresponding to the axes denoted on the sheet (x is red, y is green, z is blue).

![chessboard frame](https://github.com/larics/prarob_calib/blob/main/files/chessboard_frame.png "Chessboard frame")

This works as follows. We first generate world_points which are coordinates of the inner square corners in the coordinate frame of the sheet. Then we detect those in the image, match image points to world points and solve PnP (Perspective-N-Point) problem. You should get an output in the terminal:

```bash
[cam_to_world-2] [INFO] [1747993397.306976804] [cam2world]: Got matrix
[cam_to_world-2] [[ 0.99505, 0.09874, 0.01103,-0.12349],
[cam_to_world-2]  [ 0.09931,-0.98546,-0.13789, 0.04024],
[cam_to_world-2]  [-0.00275, 0.1383 ,-0.99039, 0.42894],
[cam_to_world-2]  [ 0.     , 0.     , 0.     , 1.     ]]
```

which is the transformation matrix describing the pose of the world (checkerboard) frame in the frame of the camera. This can also work with arbitrary world frame, as long as you first transform the points to that frame. We have implemented transforming of these points to the **R** frame shown on the calibration sheet (check _generate_world_points_R_ function in _camera_to_world.py_).

![R frame](https://github.com/larics/prarob_calib/blob/main/files/R_frame.png "R frame")

The matrix used in the function describes the pose of the checkerboard with respect to the **R** frame, so if you do something like that with respect to your robot, you should get a good result.

## Image points to world points
Once you have the matrix that describes the pose of some world frame in the camera frame, you can use _image2world_ function from _camera_to_world.py_ to extract 3D coordinates in the world frame of a point in the image. The function requires the above mentioned matrix denoting world pose in camera frame, camera intrinsic matrix and _z_ coordinate of the object (in world frame, this should in our case always be zero). We have implemented the detection of the red circle on the sheet and reconstruction of its position in the **R** frame. If you set detection parameters correctly, you should get output in terminal:

```bash
[cam_to_world-2] [INFO] [1747993397.254379941] [cam2world]: Circle location:
[cam_to_world-2] [0.22367 0.03057 0.     ]
```

which is the correct location of the center of the circle in the **R** frame. 

## IMPORTANT NOTE
Since checkerboard is symmetric, sometimes the solution of PnP problem can be flipped. Please always check before saving the extrinsic parameters matrix. 
