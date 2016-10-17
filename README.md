# laser_random_walk

Installation
============

We follow
[this link](https://www.openrobots.org/morse/doc/stable/user/installation/mw/ros.html):

```bash
$ rosdep install morseslam  --ignore-src
$ sudo add-apt-repository ppa:morse-simulator/daily
$ sudo apt-get update
$ sudo apt-get dist-upgrade
The following packages will be upgraded:
  morse-simulator morse-simulator-data morse-simulator-doc python3-morse-simulator
```

***Problem***:
Error
```bash
class YAMLObject(metaclass=YAMLObjectMetaclass)
```

***Solution***:
clear the ```PYTHONPATH``` variable.

***Problem***: ```No module named 'error'```

***Solution*** [from here](https://www.openrobots.org/morse/doc/stable/user/faq.html):

```bash
$ nano .bashrc
export PYTHONPATH="$PYTHONPATH:/usr/lib/python3/dist-packages"
$ source .bashrc
```

Dependencies (if needed)
========================


```bash
$ sudo apt-get install python3-dev
$ sudo apt install python3-yaml  python3-catkin-tools
```

Morse requires python3-rospkg:

```bash
cd ~/src
git clone git://github.com/ros-infrastructure/catkin_pkg.git
cd catkin_pkg
sudo python3 setup.py install

cd ~/src
git clone git://github.com/ros/catkin.git
cd catkin
sudo python3 setup.py install
```

gazebo and rviz on eeepc
========================

put in ```~/.bashrc```:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

# wireframe
```bash
env = Environment('sandbox', fastmode = True)
```

compile from sources
====================

Error:

```bash
"Automatic installation of middlewares’ support requires MORSE >=1.3! Ubuntu
  =< 15.04 and Debian Wheezy/Jessie only ship MORSE 1.2.2: in that case, you
  still need to install MORSE manually if you want to use MORSE with
  ROS/YARP/MOOS/pocolibs. Read on."
  "No module named 'morse.middleware.ros_datastream'"
Could not import modules required for the desired datastream interface
```

=> Need to install from source
Download the latest stable version
and do ```make install``` (not ```make``` only)


List of robots
==============

https://www.openrobots.org/morse/doc/stable/components_library.html
https://www.openrobots.org/morse/doc/stable/user/integration.html
