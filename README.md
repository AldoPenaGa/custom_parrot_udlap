<h2 align="center">Intelligent Drone Competition: MICAI 2024</h2>

<p align="justify">
  This repository hosts the ROS package that earned the third place :3rd_place_medal: in the Intelligent Drones Competition. This event was organized by the National Institute of Astrophysics, Optics, and Electronics (INAOE, by its Spanish acronym) as part of the 23rd Mexican International Conference on Artificial Intelligence (MICAI), held in Puebla, Mexico.
</p>

---

![Competition GIF](https://github.com/AldoPenaGa/custom_parrot_udlap/blob/main/pictures/Gif_.gif)

## Table of Contents
- [Introduction](#introduction)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Contributors](#contributors)

<div align="justify">

### Introduction
This ROS package was developed to provide a robust framework featuring two primary nodes: one for camera handling and another for control/teleoperation. Designed for efficiency, the system enabled the drone to autonomously navigate through a series of size fixed orange-colored windows. These windows varied in location and orientation, which settled as a difficult scenario to test the drone's autonomous navigation capabilities.

To face this challenge, the vision system used YOLOv8, custom-trained for the competition's specific tasks, while the teleoperation/control system was built from scratch. Due to the high publication rate of the camera feed, a throttle node was introduced to manage data flow, which is intended to be fixed in the future.

### Prerequisites
1. **Drone**: Bebop2 Parrot.
2. **Operating System**: Ubuntu 20.04 with ROS Noetic.

### Installation

To set up the system, follow these steps:

1. **Install Bebop Autonomy (drone driver) and Parrot ARSDK wrapper**:
   - Follow the instructions at this [repository link](https://github.com/antonellabarisic/parrot_arsdk/tree/noetic_dev).

2. **Download this repository**:
   - Navigate to your ROS workspace's `src` folder:
     ```bash
     cd path-to-ROS-ws/src
     git clone https://github.com/AldoPenaGa/custom_parrot_udlap.git
     ```
3. **Install remaining ROS dependencies:**:
     ```bash
     `cd ..
     rosdep install --from-paths src --ignore-src -r
     ```
4. **Locate into the ROS workspace and build it:**:
`catkin build` or alternatively `catkin_make`

### Usage
The system has the following options to use it:

1. **Launch the entire system:**

`roslaunch custom_parrot_udlap start.launch`

2. **Each module can be launched separately:**

- For the cam node: `rosrun custom_parrot_udlap camera_node.py`
- For the throttle node: `rosrun custom_parrot_udlap camera_node.py`
- For the teleop node: `rosrun custom_parrot_udlap camera_node.py`

**Map used for controlling the system:**

```
---------------------------
   q    w   e       +: up
   a        d       -: down
   z    s   c
---------------------------
Take off: 1
Land: 2
---------------------------
Teleop mode: t
Automatic mode: y
---------------------------
Cam control:
---------------------------
   j    i    l       
        k    
---------------------------
CTRL-C to exit and land.
```


### Contributors

| Name                          | Github                               |
|-------------------------------|--------------------------------------|
| Aldo Oziel Peña Gamboa        | https://github.com/AldoPenaGa        |
| Charbel Breydy Torres         | https://github.com/Buly1601          |
| Pablo Emilio Peredo Vega      | https://github.com/PEREGOOOD         |
| Jose Miguel Zúñiga Juarez     | https://github.com/mike130201        |
| Diego de Jesús Pastrana Blanco| null                                 |

