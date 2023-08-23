# Unicycle Motion Control, Prediction, and Navigation
![Project Status](https://img.shields.io/badge/Project_Status-maintained-yellowgreen) &nbsp;
![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-darkgray) &nbsp;
![ROS](https://img.shields.io/badge/ROS-Noetic-darkgreen)\
This repository provides the MATLAB code used to prepare numerical simulations of the study "Adaptive Headway Motion Control and Motion Prediction for Safe Unicycle Motion Design".

 [İşleyen, Aykut, Nathan van de Wouw, and Ömür Arslan. "Adaptive Headway Motion Control and Motion Prediction for Safe Unicycle Motion Design." arXiv preprint arXiv:2209.12648 (2022).](https://arxiv.org/pdf/2304.02760.pdf)

---

## Table of Contents
- [Introduction](#introduction)
- [Dependencies](#dependencies)
- [Usage](#usage)
- [Roadmap](#roadmap)
- [Authors](#authors)
- [License](#license)

---

### Introduction
As part of the ongoing research in safe unicycle robot navigation, this repository houses the MATLAB code that facilitates numerical simulations outlined in the paper "Adaptive Headway Motion Control and Motion Prediction for Safe Unicycle Motion Design." The provided code serves as a foundation for studying motion control strategies and navigation techniques for unicycles in cluttered environments. The research aims to enhance the capabilities of unicycle robots, leading to their successful integration into real-world applications, such as autonomous delivery systems, surveillance, and exploration missions.

We hope this repository and its code inspire your research and applications. Feel free to explore the provided simulations, experiment with various configurations, and adapt the code to suit your specific needs. For any questions, feedback, or collaboration opportunities, please reach out to us!


---

### Dependencies

#### <ins>Python Module Dependency</ins>

The following Python modules need to be installed using `pip3`:
- numpy 1.24.4
- scipy 1.9.1
- scikit-image (skimage) 0.20.0
- matplotlib 3.1.2
- shapely 2.0.1

You can install these packages using the command line as:
```
pip3 install numpy scipy scikit-image matplotlib shapely
```

If any of these packages has an earlier version installed. You can upgrade using:
```
pip3 install module_name --upgrade
```

#### <ins>ROS Package Dependency</ins>

The following additional ROS Noetic packages are also needed:
- map server

You can install these packages using the command line as:
```
sudo apt-get install ros-noetic-map-server
```


---

### Usage
To run the simulations, follow these steps:

1. On your local machine that runs on Ubuntu 20.04 and has ROS Noetic installed, open a terminal and create a catkin workspace on your home directory as:
```
mkdir -p ~/sample_ws/src
cd ~/sample_ws/src
catkin_init_workspace
cd ~/sample_ws
catkin_make
```
2. Download or clone this repository to the `src` folder under your workspace.
```
cd ~/sample_ws/src
git clone [url_of_the_repository]
```
3. Compile the Catkin Workspace
```
cd ~/sample_ws
catkin_make
source ~/sample_ws/devel/setup.bash
```
4. Add your workspace to your `~/.bashrc` file
```
echo "source ~/sample_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
5. Launch example demo files:
- Demo for safe unicycle path following via feedback motion prediction and safety assessment.
```
roslaunch time_governed_unicycle_navigation demo_time_governed_navigation.launch
```
- Demo for unicycle feedback motion predictions that bound the closed-loop unicycle motion trajectory under adaptive headway control towards a given goal position: circular and triangular motion predictions.
```
roslaunch unicycle_motion_control demo_ahc_motion_prediction_rviz.launch
```



 --- 


### Roadmap

Our ongoing efforts include exploring new unicycle control methods and motion design techniques to enhance the capabilities of autonomous systems. These studies revolve around various aspects of unicycle robot navigation, predictive control, and advanced motion planning strategies. For more information on these studies, please refer to the following links:

- [İşleyen, Aykut, Nathan van de Wouw, and Ömür Arslan. "From low to high order motion planners: Safe robot navigation using motion prediction and reference governor." IEEE Robotics and Automation Letters 7.4 (2022): 9715-9722.](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=9832477)
- [Arslan, Ömür, and Aykut İşleyen. "Vandermonde Trajectory Bounds for Linear Companion Systems." arXiv preprint arXiv:2302.10995 (2023).](https://arxiv.org/pdf/2302.10995.pdf)
- [İşleyen, Aykut, Nathan van de Wouw, and Ömür Arslan. "Feedback Motion Prediction for Safe Unicycle Robot Navigation." arXiv preprint arXiv:2209.12648 (2022).](https://arxiv.org/pdf/2209.12648.pdf)


---

### Authors
The code is authored by Aykut Isleyen and [Dr. Ömür Arslan](https://omurarslan.github.io/).


---

### License
This project is licensed under the MIT License. See the LICENSE file for details.


