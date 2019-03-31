# Dash Robotics Perception (Assignment 3)

## Setup

1. Clone this repository to your `ws/src/` folder.
2. Run `catkin_make` and `source devel/setup.bash` from your `ws/` folder.
3. Note that this repository `perception` by itself is not a ROS package.

## Usage

### Package: `cuboid_detection`

The package was created with the following command.

```
catkin_create_pkg cuboid_detection pcl_ros roscpp sensor_msgs pcl_conversions mage_transport opencv3 cv_bridge
```

For Assignment 3, we are use the `cuboid_detection` package.

Place the downloaded bag files in `ws/src/perception/cuboid_detection/bags`.

More details on launch files yet to come.

## Notes

### Camera Info Topics (Intel RealSense D435)

#### /camera/color/camera_info

```
header: 
  seq: 431
  stamp: 
    secs: 1553391107
    nsecs: 738674179
  frame_id: "camera_color_optical_frame"
height: 480
width: 640
distortion_model: "plumb_bob"
D: [0.0, 0.0, 0.0, 0.0, 0.0]
K: [616.8246459960938, 0.0, 321.81976318359375, 0.0, 616.609375, 239.91116333007812, 0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [616.8246459960938, 0.0, 321.81976318359375, 0.0, 0.0, 616.609375, 239.91116333007812, 0.0, 0.0, 0.0, 1.0, 0.0]
binning_x: 0
binning_y: 0
roi: 
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: False
```

#### /camera/depth/camera_info

```
header: 
  seq: 1529
  stamp: 
    secs: 1553391287
    nsecs: 182639969
  frame_id: "camera_depth_optical_frame"
height: 480
width: 640
distortion_model: "plumb_bob"
D: [0.0, 0.0, 0.0, 0.0, 0.0]
K: [384.0898742675781, 0.0, 322.4656677246094, 0.0, 384.0898742675781, 240.64073181152344, 0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [384.0898742675781, 0.0, 322.4656677246094, 0.0, 0.0, 384.0898742675781, 240.64073181152344, 0.0, 0.0, 0.0, 1.0, 0.0]
binning_x: 0
binning_y: 0
roi: 
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: False
```
