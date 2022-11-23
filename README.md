# ROS wrapper (cpp) for the Fourier Scan Matcher ([FSM](https://github.com/li9i/fsm)) in the guise of lidar odometry.

Functionally `fsm_lidom_ros` assumes a 2D LIDAR sensor with a field of view of
360 degrees and executing it while the sensor is on the move will output an estimate
of its trajectory.


## Installation

### Via Docker

```
docker pull li9i/fsm_lidom_ros

docker run -it \
    --name=fsm_lidom_ros_container \
    --env="DISPLAY=$DISPLAY" \
    --net=host \
    --rm \
    li9i/fsm_lidom_ros:latest
```

or build the image with the most recent code of this repository

```
cd ~/catkin_ws/src
git clone git@github.com:li9i/fsm_lidom_ros.git
cd fsm_lidom_ros/docker
./build_fsm_lidom_image.sh
./run_fsm_lidom_container.sh

```

### Via the traditional method

Tested in Ubuntu 16.04 and ROS kinetic

#### Dependencies: `CGAL 4.7` `FFTW3` `boost/random`

#### Building

```
cd ~/catkin_ws/src
git clone git@github.com:li9i/fsm_lidom_ros.git
cd ~/catkin_ws
catkin build fsm_lidom_ros
```

#### Executing

```
roslaunch fsm_lidom_ros avanti_fsm_lidom.launch
```

## Nodes

### `fsm_lidom_node`

#### Parameters

Found in `config/params.yaml`:

| Node-specific parameters | What is this?                                                                    |
| ------------------------ | -------------------------------------------------------------------------------- |
| `scan_topic`             | 2d panoramic scans are published here                                            |
| `initial_pose_topic`     | (optional) the topic where an initial pose estimate may be provided              |
| `pose_estimate_topic`    | `fsm_lidar_ros`'s pose estimates are published here                              |
| `path_estimate_topic`    | `fsm_lidar_ros`'s total trajectory estimate is published here                    |
| `scan_size`              | for resizing the input scans' size (execution time is proportional to scan size) |

| FSM-specific parameters  | And this?                                                                                         |
| ------------------------ | ------------------------------------------------------------------------------------------------- |
| `min_magnification_size` | base angular oversampling                                                                         |
| `max_magnification_size` | maximum angular oversampling                                                                      |
| `num_iterations`         | Greater sensor velocity requires higher values                                                    |
| `xy_bound`               | Axiswise radius for randomly generating a new initial position estimate in case of recovery       |
| `t_bound`                | Angularwise radius for randomly generating a new initial orientation estimate in case of recovery |
| `max_counter`            | Lower values decrease execution time                                                              |
| `max_recoveries`         | Ditto                                                                                             |

---


#### Subscribed topics

| Topic                | Type                                     | Utility                                                                                |
| -------------------- | ---------------------------------------- | ---------------------------------------------------------------------------------------|
| `scan_topic`         | `sensor_msgs/LaserScan`                  | 2d panoramic scans are published here                                                  |
| `initial_pose_topic` | `geometry_msgs/PoseWithCovarianceStamped`| optional---for setting the very first pose estimate to something other than the origin |

---

#### Published topics

| Topic                 | Type                        | Utility                                          |
| --------------------- | ----------------------------| ------------------------------------------------ |
| `pose_estimate_topic` | `geometry_msgs/PoseStamped` | the current pose estimate is published here      |
| `path_estimate_topic` | `nav_msgs/Path`             | the total estimated trajectory is published here |

---

#### Services offered

| Service                              | Type             | Utility                                                                                                                                          |
| ------------------------------------ | ---------------- | ------------------------------------------------------------------------------------------------------------------------------------------------ |
| `fsm_lidom/service_start`            | `std_srvs/Empty` | commences node functionality                                                                                                                     |
| `fsm_lidom/service_stop`             | `std_srvs/Empty` | halts node functionality (node remains alive)                                                                                                    |
| `fsm_lidom/set_initial_pose_service` | `std_srvs/Empty` | calling this service means: node subscribes to `initial_pose_topic`, obtains the latest pose estimate, sets fsm's initial pose, and unsubscribes |

---

## Motivation and Under the hood

### 1 min summary video
[![IMAGE ALT TEXT](http://img.youtube.com/vi/hB4qsHCEXGI/0.jpg)](http://www.youtube.com/watch?v=hB4qsHCEXGI "1 min summary video")

### IROS 2022 presentation slides
[PDF link](https://raw.githubusercontent.com/li9i/fsm_presentation_iros22/master/main.pdf)

### IROS 2022 paper

(Incomplete)

```bibtex
@INPROCEEDINGS{fsm_iros_2022,
  author    = {Alexandros Filotheou, Georgios D. Sergiadis, Antonis G. Dimitriou},
  booktitle = {2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  title     = {FSM: Correspondenceless scan-matching of panoramic 2D range scans},
  year      = {2022},
  doi       = {TBA}
}
