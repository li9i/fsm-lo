# ROS wrapper (cpp) for the Fourier Scan Matcher ([FSM](https://github.com/li9i/fsm)) in the guise of lidar odometry.

Functionally `fsm_lidom_ros` assumes a 2D LIDAR sensor with a field of view of
360 deg and executing it while the sensor is on the move will output an estimate
of its trajectory.

## Dependencies
`CGAL 4.7`
`FFTW3`
`boost/random`

## Building

As always
```
$ cd ~/catkin_ws/src
$ git clone git@github.com:li9i/fsm_lidom_ros.git
$ catkin build fsm_lidom_ros
```

# Nodes

## `fsm_lidar_ros`

### Parameters

Found in `config/params.yaml`:

| Node-specific parameters | What is this?                                                                    |
| ------------------------ | -------------------------------------------------------------------------------- |
| `scan_topic`             | 2d panoramic scans are published here                                            |
| `initial_pose_estimate`  | (optional) the topic where an initial pose estimate may be provided              |
| `pose_estimate_topic`    | `fsm_lidar_ros`'s pose estimates are published here                              |
| `path_estimate_topic`    | `fsm_lidar_ros`'s total trajectory estimate is published here                    |
| `scan_size`              | for resizing the input scans' size (execution time is proportional to scan size) |

| FSM-specific parameters  | And this?                                                                                        |
| ------------------------ | ------------------------------------------------------------------------------------------------ |
| `min_magnification_size` | base angular oversampling                                                                        |
| `max_magnification_size` | maximum angular oversampling                                                                     |
| `num_iterations`         | Greater sensor velocity requires higher values                                                   |
| `xy_bound`               | Axiswise radius for randomly generating a new initial position estimate in case of recovery       |
| `t_bound`                | Angularwise radius for randomly generating a new initial orientation estimate in case of recovery |
| `max_counter`            | Lower values decrease execution time                                                             |
| `max_recoveries`         | Ditto                                                                                            |


### Subscribed topics

| Topic                | Type                                     |                                                                                        |
| -------------------- | ---------------------------------------- | ---------------------------------------------------------------------------------------|
| `scan_topic`         | `sensor_msgs/LaserScan`                  | 2d panoramic scans are published here                                                  |
| `initial_pose_topic` | `geometry_msgs/PoseWithCovarianceStamped`| optional---for setting the very first pose estimate to something other than the origin |

### Published topics
`pose_estimate_topic` (`geometry_msgs/PoseStamped`)\
is where the current pose estimate is published

`path_estimate_topic` (`nav_msgs/Path`)\
is where the total estimated trajectory is published


### Services offered
`fsm_lidar_ros/service_start` (`std_srvs/Empty`)\
commences node functionality

`fsm_lidar_ros/service_stop` (`std_srvs/Empty`)\
halts node functionality (node remains alive) calling `fsm_lidar_ros/service_start` revives it

`fsm_lidom/set_initial_pose_service` (`std_srvs/Empty`)\
calling this service means: node subscribes to `initial_pose_topic`, obtains the latest pose estimate, sets fsm initial pose, and unsubribes


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
