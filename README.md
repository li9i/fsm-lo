# ROS wrapper (cpp) for the Fourier Scan Matcher ([FSM](https://github.com/li9i/fsm)) in the guise of lidar odometry.

[![ieeexplore.ieee.org](https://img.shields.io/badge/IEEE/RSJ_IROS_2022_paper-00629B)](https://ieeexplore.ieee.org/abstract/document/9981228)
[![youtube.com](https://img.shields.io/badge/1'_presentation-YouTube-FF0000)](https://www.youtube.com/watch?v=hB4qsHCEXGI)
[![github.com](https://img.shields.io/badge/pdf_presentation-333333)](https://github.com/phd-li9i/fsm_presentation_iros22/blob/master/main.pdf)


Functionally `fsm_lidom_ros` assumes a 2D LIDAR sensor with a field of view of 360 degrees. Executing `fsm_lidom_ros` while the sensor is on the move will output an estimate of its trajectory. Lidar odometry is achieved via scan-matching without using correspondences, based on properties of the Discrete Fourier Transform.


Table of Contents
=================
* [Installation](#installation)
  * [Via Docker](#via-docker)
  * [Via the traditional method](#via-the-traditional-method)
    * [Dependencies](#dependencies-cgal-47-fftw3-boostrandom)
    * [Building](#building)
    * [Executing](#executing)
* [Nodes](#nodes)
  * [`fsm_lidom_node`](#fsm_lidom_node)
    * [Subscribed topics](#subscribed-topics)
    * [Published topics](#published-topics)
    * [Services offered](#services-offered)
    * [Parameters](#parameters)
    * [Transforms published](#transforms-published)
* [Motivation and Under the hood](#motivation-and-under-the-hood)
  * [1 min summary video](#1-min-summary-video)
  * [IROS 2022 paper](#iros-2022-paper)



## Installation

### Via Docker

```console
docker pull li9i/fsm_lidom_ros

docker run -it \
    --name=fsm_lidom_ros_container \
    --env="DISPLAY=$DISPLAY" \
    --net=host \
    --rm \
    li9i/fsm_lidom_ros:latest
```

or build the image with the most recent code of this repository

```console
cd ~/catkin_ws/src
git clone git@github.com:li9i/fsm_lidom_ros.git
cd fsm_lidom_ros/docker
docker compose build
docker compose up
```

### Via the traditional method

Tested in Ubuntu 16.04 and ROS kinetic

#### Dependencies: `CGAL 4.7` `FFTW3` `boost/random`

#### Building

```console
cd ~/catkin_ws/src
git clone git@github.com:li9i/fsm_lidom_ros.git
cd ~/catkin_ws
catkin build fsm_lidom_ros
```

#### Executing

```console
roslaunch fsm_lidom_ros avanti_fsm_lidom.launch
```

## Nodes

### `fsm_lidom_node`

#### Subscribed topics

| Topic                | Type                                     | Utility                                                                                |
| -------------------- | ---------------------------------------- | ---------------------------------------------------------------------------------------|
| `scan_topic`         | `sensor_msgs/LaserScan`                  | 2d panoramic scans are published here                                                  |
| `initial_pose_topic` | `geometry_msgs/PoseWithCovarianceStamped`| optional---for setting the very first pose estimate to something other than the origin |

---

#### Published topics

| Topic                 | Type                        | Utility                                                                       |
| --------------------- | ----------------------------| ------------------------------------------------                              |
| `pose_estimate_topic` | `geometry_msgs/PoseStamped` | the current pose estimate relative to the global frame is published here      |
| `path_estimate_topic` | `nav_msgs/Path`             | the total estimated trajectory relative to the global frame is published here |
| `lidom_topic`         | `nav_msgs/Odometry`         | the odometry is published here                                                |

---

#### Services offered

| Service                                | Type             | Utility                                                                                                                                          |
| -------------------------------------- | ---------------- | ------------------------------------------------------------------------------------------------------------------------------------------------ |
| `fsm_lidom/clear_estimated_trajectory` | `std_srvs/Empty` | clears the vector of estimated poses                                                                                                             |
| `fsm_lidom/set_initial_pose`           | `std_srvs/Empty` | calling this service means: node subscribes to `initial_pose_topic`, obtains the latest pose estimate, sets fsm's initial pose, and unsubscribes |
| `fsm_lidom/start`                      | `std_srvs/Empty` | commences node functionality                                                                                                                     |
| `fsm_lidom/stop`                       | `std_srvs/Empty` | halts node functionality (node remains alive)                                                                                                    |

---

#### Parameters

Found in `config/params.yaml`:

| IO Topics                | Description                                                         |
| ------------------------ | ------------------------------------------------------------------- |
| `scan_topic`             | 2d panoramic scans are published here                               |
| `initial_pose_topic`     | (optional) the topic where an initial pose estimate may be provided |
| `pose_estimate_topic`    | `fsm_lidom_ros`'s pose estimates are published here                 |
| `path_estimate_topic`    | `fsm_lidom_ros`'s total trajectory estimate is published here       |
| `lidom_topic`            | `fsm_lidom_ros`'s odometry estimate is published here               |

| Frame ids         | Description                                                     |
| ----------------- | -------------------------------------                           |
| `global_frame_id` | the global frame id (e.g. `/map`)                               |
| `base_frame_id`   | the lidar sensor's reference frame id (e.g. `/base_laser_link`) |
| `lidom_frame_id`  | the (lidar) odometry's frame id                                 |

| FSM-specific parameters  | Description                                                                                                       |
| ------------------------ | ----------------------------------------------------------------------------------------------------------------- |
| `size_scan`              | the size of scans that are matched (execution time is proportional to scan size, hence subsampling may be needed) |
| `min_magnification_size` | base angular oversampling                                                                                         |
| `max_magnification_size` | maximum angular oversampling                                                                                      |
| `num_iterations`         | Greater sensor velocity requires higher values                                                                    |
| `xy_bound`               | Axiswise radius for randomly generating a new initial position estimate in case of recovery                       |
| `t_bound`                | Angularwise radius for randomly generating a new initial orientation estimate in case of recovery                 |
| `max_counter`            | Lower values decrease execution time                                                                              |
| `max_recoveries`         | Ditto                                                                                                             |

---

#### Transforms published

```
lidom_frame_id <- base_frame_id
```

in other words `fsm_lidom_node` publishes the transform from `/base_laser_link`
(or equivalent) to the equivalent of `/odom` (in this case `lidom_frame_id`)

---

## Motivation and Under the hood

### 1 min summary video
[![IMAGE ALT TEXT](http://img.youtube.com/vi/hB4qsHCEXGI/0.jpg)](http://www.youtube.com/watch?v=hB4qsHCEXGI "1 min summary video")

<!-- ### IROS 2022 presentation slides -->
<!-- [PDF link](https://raw.githubusercontent.com/li9i/fsm_presentation_iros22/master/main.pdf) -->

### IROS 2022 paper

```bibtex
@INPROCEEDINGS{9981228,
  author={Filotheou, Alexandros and Sergiadis, Georgios D. and Dimitriou, Antonis G.}
  booktitle={2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  title={FSM: Correspondenceless scan-matching of panoramic 2D range scans},
  year={2022},
  pages={6968-6975},
  doi={10.1109/IROS47612.2022.9981228}}
