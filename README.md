# parrot_ros

ROS driver for [Parrot Anafi](https://www.parrot.com/en/) drones. Based on Official [Parrot Olympe](https://developer.parrot.com/docs/olympe/index.html) (Python SDK) and [Parrot Sphinx](https://developer.parrot.com/docs/sphinx/index.html) (UE4 simulation tool).

<!-- ![GIF](./repo/frame/test_pursuit_z.gif) -->

* Author: [Amir Ebrahimnezhad](https://amirtronics.com)
* Maintainer: [Amir Ebrahimnezhad](https://amirtronics.com)


## Table of Contents

- [Ubuntu](#ubuntu)
- [Nvidia Driver](#nvidia-driver)
- [CUDA](#cuda)
- [Python Env](#python-env)
- [Olympe](#olympe)
- [Sphinx](#sphinx)
- [ROS Noetic](#ros-noetic)
- [Python Requirements](#python-requirements)


Please follow the following instructions carefully for setting up your development environment. If there are any issues with the installations or the codes, simply open a new issue request on the  [github repo](https://github.com/amirtronics/parrot_ros).


<div align="center">
  <p>
    <a href="" target="_blank">
      <img width="100%" src="./media/PID.png"></a>
    <!--
    <a align="center" href="https://ultralytics.com/yolov5" target="_blank">
      <img width="100%" src="https://raw.githubusercontent.com/ultralytics/assets/main/yolov5/v70/splash.png"></a>
    -->
  </p>
</div>


# Installation Guide

## Ubuntu

Install Ubuntu 20.04 dekstop image. Follow instructions on [ubuntu.com](https://releases.ubuntu.com/focal/). You can install it as dual-boot along your Windows but you can't use VM Ware as the GPU is inaccessible over virtual machine.

## Nvidia Driver

After installing Ubuntu, go ahead with installing the graphic card driver:

```
sudo apt-get install nvidia-driver-530
```

Reboot and verify the installation by:

```
sudo reboot 
nvidia-smi
```

## CUDA

We'll use CUDA 11.1 for this project. Download the runfile and install it. Remember to uncheck the Nvidia Driver on runfile:


```
cd ~/Downloads

wget https://developer.download.nvidia.com/compute/cuda/11.1.0/local_installers/cuda_11.1.0_455.23.05_linux.run
sudo sh cuda_11.1.0_455.23.05_linux.run
```

![Alt Text](./media/cuda.png)


You need to add cuda path to your .bashrc after installation. Simply:

```
echo 'export PATH="/usr/local/cuda-11.1/bin:$PATH"' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH="/usr/local/cuda-11.1/lib64:$LD_LIBRARY_PATH"' >> ~/.bashrc
```

Now you can check your CUDA version:

```
source ~/.bashrc
nvcc --version
```


## Python Env

First things first, you need to change teh default python command:

```
sudo apt install python-is-python3
```

Now install pip as:

```
sudo apt install python3-pip
```

Type python on terminal and double check the default python env which should be 3.8:

```
python
```


Before installing Parrot Olympe and other Python pacakges, it's needed to setup a virtual Python environment (let's call it parrot) for using a newer version of pip (20.3 or higher).

```
sudo apt install python3.8-venv
python -m venv parrot
source parrot/bin/activate
```

You'll see an indicator (parrot) on your terminal. Now you need to upgrade your pip on parrot env:

```
python -m pip install --upgrade pip
```

Double check you pip version, it should be 23.3:

```
pip --version
```

## Olympe


Now it's time or Olympe, make sure you're installing it on parrot env:

```
pip install parrot-olympe
```

Double check installation by importing it on parrot:

```
python
>>> import olympe
>>> quit()
```


## Sphinx

For simulating a drone and sending commands, we need to Parrot Sphinx simulator which is based on UE4 (Unreal Engine). 


```
curl --fail --silent --show-error --location https://debian.parrot.com/gpg | gpg --dearmor | sudo tee /usr/share/keyrings/debian.parrot.com.gpg > /dev/null
echo "deb [signed-by=/usr/share/keyrings/debian.parrot.com.gpg] https://debian.parrot.com/ $(lsb_release -cs) main generic" | sudo tee /etc/apt/sources.list.d/debian.parrot.com.list
sudo apt update
```

```
sudo apt install parrot-sphinx
```


Now it's time to install Parrot UE4:

```
sudo apt update
sudo apt install parrot-ue4-empty
```

To have access to pysphinx library run: (add to bash file)

```
. /opt/parrot-sphinx/usr/bin/parrot-sphinx-setenv.sh
```


### Wifi Fix

If your Wifi is disabled by running Sphinx, simply add wifi_iface="" to command line as follows:

```
sphinx "/opt/parrot-sphinx/usr/share/sphinx/drones/anafi_ai.drone"::firmware="https://firmware.parrot.com/Versions/anafi2/pc/%23latest/images/anafi2-pc.ext2.zip"::wifi_iface=""
```

## ROS Noetic

ROS or Robot Operating System is a middleware which allows developing roboitcs packages and runtimes. The greatest advantage of ROS is that it comes with predeveloped pipelines for establshing communication between different Python and C++ nodes. To install ROS simply follow the instructions below:



Configure Ubuntu repositories:


```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

Installation:

```
sudo apt update
sudo apt install ros-noetic-desktop-full
source /opt/ros/noetic/setup.bash
```

Rosdep:

```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

Verifying installation:

```
printenv | grep ROS
source /opt/ros/noetic/setup.bash
```

Catkin workspace setup:

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make

```


Before running the next command, replace {user} with your own Ubuntu username:


```
echo 'source /home/{user}/catkin_ws/devel/setup.bash' >> ~/.bashrc
source ~/.bashrc
```


Now simply run roscore in a terminal to run ROS:

```
roscore
```

## Python Requirements

```
pip install -r requirements.txt
```


## Pytorch and Yolov5

There are several neural networks libraries out there, most prominently PyTorch and Tensorflow. These libraries provide you with the necessary tools for developing neural networks from scratch. However, developing an optimised acrhictecture from scratch takes time and is a painstaking task, so instead of developing the architecture ourselves, we can simply use fine-tuned networks. Yolov5 is one of the fastest object detection models which is based on Torch. Therefore, PyTorch 1.8 is already in the requirements of Yolov5 which is also compatible with CUDA 11.1 as well. To do so, make a new folder first:

```
mkdir ~/code
cd ~/code
```

Make sure parrot environment is activated:

```
source ~/parrot/bin/activate
```


Now clone and install yolov5:


```
git clone https://github.com/ultralytics/yolov5  # clone
cd yolov5
pip install -r requirements.txt  # install
```

## Xterm


```
sudo apt-get -y install xterm

```

