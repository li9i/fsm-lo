# Install and run natively

This package was tested and works under Ubuntu 16.04 and ROS kinetic. You will need `CGAL 4.7`, `fftw3`, and `boost/random` as dependencies.

## Install

```sh
cd ~/catkin_ws/src
git clone git@github.com:li9i/fsm-lo.git
cd fsm-lo; mv fsm_lo/* $PWD; rmdir fsm_lo; cd ../..
catkin build fsm_lo
```

## Run

### Launch


```sh
roslaunch fsm_lo avanti.launch
```

### Call

Launching `fsm-lo` simply makes it go into stand-by mode and does not actually execute anything. To do so simply call the provided service

```sh
docker exec -it fsm_lo_container sh -c "source ~/catkin_ws/devel/setup.bash; rosservice call /fsm_lo/start"
```
