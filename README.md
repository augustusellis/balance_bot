# balanceBot

## Overview

Operates a balance bot (wheeled inverted pendulum) made by Blake Leiker, Anthony Hresko, and Augustus Ellis in a graduate mechatronics class (TAMU MEEN 667). The master branch is done directly in python, the ros_development branch is the same code reformated into a ros package.

**Keywords:** inverted pendulum, ROS, python

### License

The source code is released under a [BSD 3-Clause license](ros_package_template/LICENSE).

**Author: Augustus Ellis<br />
Maintainer: Augustus Ellis, augususellis@gmail.com**

The balanceBot package has been tested under [ROS] Kinetic and Ubuntu 16.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

<!---[![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=ros_best_practices)](http://rsl-ci.ethz.ch/job/ros_best_practices/)


![Example image](doc/example.jpg)--->

<!---
### Publications

If you use this work in an academic context, please cite the following publication(s):

* P. Fankhauser, M. Bloesch, C. Gehring, M. Hutter, and R. Siegwart: **PAPER TITLE**. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2015. ([PDF](http://dx.doi.org/10.3929/ethz-a-010173654))

        @inproceedings{Fankhauser2015,
            author = {Fankhauser, P\'{e}ter and Hutter, Marco},
            booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
            title = {{PAPER TITLE}},
            publisher = {IEEE},
            year = {2015}
--->        }

<!---
## Installation

### Building from Source
git clone https://github.com/augustusellis/mechatronics.git
--->

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [pigpio](http://abyz.me.uk/rpi/pigpio/python.html) (Raspberry Pi GPIO interface library)



#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/augustusellis/mechatronics.git
	cd ../
	catkin_make

<!---
### Unit Tests

Run the unit tests with

	catkin_make run_tests_ros_package_template
--->

## Usage

Describe the quickest way to run this software, for example:

Run the main node with

	roslaunch balanceBot balanceBot.launch

<!---
## Config files

Config file folder/set 1

* **config_file_1.yaml** Shortly explain the content of this config file

Config file folder/set 2

* **...**

## Launch files

* **launch_file_1.launch:** shortly explain what is launched (e.g standard simulation, simulation with gdb,...)

     Argument set 1

     - **`argument_1`** Short description (e.g. as commented in launch file). Default: `default_value`.

    Argument set 2

    - **`...`**

* **...**
--->
## Nodes

### ros_package_template

Reads temperature measurements and computed the average.


#### Subscribed Topics

* **`/temperature`** ([sensor_msgs/Temperature])

	The temperature measurements from which the average is computed.


#### Published Topics

...


#### Services

* **`get_average`** ([std_srvs/Trigger])

	Returns information about the current average. For example, you can trigger the computation from the console with

		rosservice call /ros_package_template/get_average


#### Parameters

* **`subscriber_topic`** (string, default: "/temperature")

	The name of the input topic.

* **`cache_size`** (int, default: 200, min: 0, max: 1000)

	The size of the cache.


### NODE_B_NAME

...


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-asl/ros_best_practices/issues).


[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[Eigen]: http://eigen.tuxfamily.org
[std_srvs/Trigger]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html
[sensor_msgs/Temperature]: http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html
