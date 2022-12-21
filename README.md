<h1 align="center">
  <br>
F1Tenth Environment
  <br>
</h1>


## General Overview
**F1tenth Environment** is a repository that holds the **FSAE reinforcement learning environment**; which is responsible for the training and deployment of **RL methods** in both a **real** and **simulated** environment.


## Prerequisites
|Library         | Version (TESTED) |
|----------------------|----|
| Ubuntu | 22.04|
| ROS2| [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)|
| Gazebo |[Garden](https://gazebosim.org/docs/garden/install_ubuntu) |
| ros_gz | [Humble, Garden](https://github.com/gazebosim/ros_gz/tree/ros2#from-source)|


## How to run this Repository

Once cloned, `cd` into the **project root** and run the following

``` bash
# Build the project
colcon build

# Source the Environment
. install/setup.bash

# Start the Simulation
ros2 launch traxxas_4x4_zoomer_bringup start_simulation.launch.py 

```

## Citation
If you use either the code, data or the step from the tutorial-blog in your paper or project, please kindly star this repo and cite our webpage

## Contact
Please feel free to contact me or open an issue if you have questions or need additional explanations.

######  The released codes are only allowed for non-commercial use.
