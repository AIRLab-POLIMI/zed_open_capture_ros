# zed_open_capture_ros

A ROS node to use Zed cameras on machines with no GPU. It wraps [zed_open_capture](https://github.com/stereolabs/zed-open-capture)

## Usage:
1. Clone the repository in your ROS catkin workspace and compile it.

```
cd catkin_ws/src
git clone git@github.com:AIRLab-POLIMI/zed_open_capture_ros.git
cd ..
catkin_make
```

2. Add the calibration files.

Calibration files can be downloaded from the Stereolabs website at <http://calib.stereolabs.com/?SN=XXXX>.
Note: Replace XXXX with the last four digit of the serial number of your camera.

By default, the node will look for calibration files in the `zed_open_capture_ros/conf` folder. This location can be changed using the `config_file_location` parameter.

3. Start the node with the provided launch file

```
roslaunch zed_open_capture_ros zed2.launch
```



## Launch file parameters

 Parameter                    |           Description                                       |              Value
------------------------------|-------------------------------------------------------------|-------------------------
 device_id                  | `/dev/video` id to open (use -1 to open the first available camera) | int
 resolution                   | ZED Camera resolution                                       | '0': HD2K
 _                            | _                                                           | '1': HD1080
 _                            | _                                                           | '2': HD720
 _                            | _                                                           | '3': VGA
 frame_rate                   | Rate at which images are published                          | int
 left_frame_id                | Left Frame ID                                               | string
 right_frame_id               | Right Frame ID                                              | string
 use_zed_config              | Use ZED calibration file                         | bool
 config_file_location         | The location of ZED calibration file                        | string
 serial                   | Serial number of the camera to open (use 0 to retrieve the serial from the camera) | int
