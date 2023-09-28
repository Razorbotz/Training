# Welcome to the Razorbotz ROS2 Training Page
This page is intended to provide a starting point for beginning to progam in ROS2. 

## Overview
* [Prerequisites](https://github.com/Razorbotz/ROS2_Training/tree/master#prerequisites)
* [Introduction](https://github.com/Razorbotz/ROS2_Training/tree/master#introduction)
* [Beginner](https://github.com/Razorbotz/ROS2_Training/tree/master/#beginner)
* [Intermediate](https://github.com/Razorbotz/ROS2_Training/tree/master#intermediate)
* [Advanced](https://github.com/Razorbotz/ROS2_Training/tree/master#advanced)

## Prerequisites
Before beginning this project, there are several key prerequisites required to ensure success. To fully understand the codebase, you will need a functional understanding of both Linux and ROS2. The tutorials linked below will help ensure that you have a good foundation to proceed with the project.

### Linux Tutorials
To ensure that you are able to effectively use Linux, go through these [Linux tutorials](https://www.hostinger.com/tutorials/linux-commands). The key objective of these tutorials is to teach how to navigate through the file structure via the terminal, as well manipulating files using commands. Because the robot is designed to be operated remotely on the lunar surface, understanding these commands is an essential skill for this project. 

### ROS2 Tutorials
To gain a better understanding of ROS2, please refer to the following [tutorials](https://docs.ros.org/en/foxy/Tutorials.html).
* [Configuring Your ROS 2 Environment](https://docs.ros.org/en/foxy/Tutorials/Configuring-ROS2-Environment.html)
* [Understanding ROS 2 Nodes](https://docs.ros.org/en/foxy/Tutorials/Understanding-ROS2-Nodes.html)
* [Understanding ROS 2 Topics](https://docs.ros.org/en/foxy/Tutorials/Topics/Understanding-ROS2-Topics.html)
* [Understanding ROS 2 Parameters](https://docs.ros.org/en/foxy/Tutorials/Parameters/Understanding-ROS2-Parameters.html)
* [Creating a Launch File](https://docs.ros.org/en/foxy/Tutorials/Launch-Files/Creating-Launch-Files.html)
* [Creating a Workspace](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html)
* [Creating a Package](https://docs.ros.org/en/foxy/Tutorials/Creating-Your-First-ROS2-Package.html)
* [Writing a Simple Publisher and Subscriber (C++)](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
* [Writing a Simple Publisher and Subscriber (Python)](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
* [Writing Custom ROS2 msg Files](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
* [Using Parameters in a Class (C++)](https://docs.ros.org/en/foxy/Tutorials/Using-Parameters-In-A-Class-CPP.html)
* [Using Parameters in a Class (Python)](https://docs.ros.org/en/foxy/Tutorials/Using-Parameters-In-A-Class-Python.html)
* [Using ROS2 Launch](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)

## Introduction
This tutorial will apply the concepts of ROS2 development to a physical system and allow you to control motors in the real world. To allow for a more personalized experience, we have created three levels of difficulty for this tutorial. 
All three levels will be given several important nodes, including the logic and communication nodes. These nodes are very complex and rewriting them would be beyond the scope of this training.

### Building The Code
To build the code, use the colcon build command. If you have written everything correctly, the code will build without any errors.

### Launching The Nodes
After successfully compiling the ROS2 code, you will launch the nodes using the ros2 launch command. If the launch file is in the current working directory, the nodes can be launched using the following commands:
```
source install/setup.bash
ros2 launch launch.py
```

## Beginner
The [Beginner folder](https://github.com/Razorbotz/ROS2_Training/tree/master/ROS2_Beginner) contains the launch file and the various nodes. The Falcon, Neo, and Talon nodes are missing some key lines of code and need to be completed before the motors will run.

## Intermediate
The [Intermediate folder](https://github.com/Razorbotz/ROS2_Training/tree/master/ROS2_Intermediate) contains the launch file and several other key files. The Falcon, Neo, and Talon nodes have the CMakeLists.txt and package.xml files needed to build, but lack the actual node file. You will need to create the C++ node file.

## Advanced
The [Advanced folder](https://github.com/Razorbotz/ROS2_Training/tree/master/ROS2_Advanced/src) contains only the logic and communication nodes. You will need to create a launch file and the node to control each of the motors attached to the system.