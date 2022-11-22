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

## Motivation and Under the hood

### Summary
[![IMAGE ALT TEXT](http://img.youtube.com/vi/hB4qsHCEXGI/0.jpg)](http://www.youtube.com/watch?v=hB4qsHCEXGI "1 min summary video")

### IROS 2022 presentation slides
[PDF link](https://raw.githubusercontent.com/li9i/fsm_presentation_iros22/master/main.pdf?token=GHSAT0AAAAAAB2XKFD62TOKYO2JNXWVDFAMY34RBTQ)

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
