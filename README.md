## IntelliCar
#### Application of Computer Vision Techniques for Object Detection and Recognition in Urban Environment Simulation oriented to Autonomous Driving Vehicles
---
##### INTRODUCTION
This project is focus on image object detection by training a model using YOLO. Simulations are done on CARLA Simulator and a integration with ROS has been developed.

![A](imgs/portada-edit.jpg)

##### TOOLS

+ **Dataset**: We create our custom dataset, using Roboflow Platform, by the integration of some existing datasets and the addition of multiple images of scenarios with different environmental and weather conditions.
[Dataset on Roboflow Platform](https://universe.roboflow.com/carla-awmfg/carladataset/model/5)

+ **Simulator**: We use the open-source simulator for autonomous driving systems CARLA.
[CARLA web page](https://carla.org/)
[CARLA Github](https://github.com/carla-simulator/carla)

+ **Model Training**: Model based on newest version of YOLO (YOLOv8) from Ultralytics. 
[YOLOv8 Github](https://github.com/ultralytics/ultralytics)

+ **ROS**: Running model inference for prediction and extra decission tasks modules.

+ **ROS-Bridge**: CARLA - ROS communication.
[ROS-Bridge Github](https://github.com/carla-simulator/ros-bridge)

![A](imgs/global_diagram.png)

---

##### CAPABILITIES


---

##### GETTING STARTED
Software development is divided into several ROS packages:
+ *descentralized_bringup*: Placeholder for programs launchfiles.
+ *cnn_image_processing*: Augmented reality via ArUco detection + fire recognition model + fire location estimation
+ *gamepad_controller*: Allow to control rosbot movement using a Xbox/PS4 controller device.
+ *map_marker*: fire marker placement on map
+ *robot_gui_bridge*: Graphical interface webserver
+ *ros_astra_camera*: Low level controller for camera activation
+ *rosbot_ekf*: Low level motors sensor and extended Kalman Filter for fusion sensor
+ *rplidar_ros*: Low level controller of LiDAR sensor
+ *tutorial_pkg*: Tutorial packages provided by manufacturer, modified for testing purposes.

Prerequisites: Ubuntu 20.04 or derivated flavours & ROS Noetic

Main functionality can be launched by executing next commands:
1. ``bash dependencies/setup.sh``
2. ``catkin_make``
3. ``roslaunch descentralized_bringup hardware.launch`` (ROSbot)
4. ``roslaunch descentralized_bringup software.launch`` (ROS slave device)

Sofware launcher has several arguments to select what to run:
| Argument | Description | Default value |
|---|---|---|
| localization | Map Server + AMCL (!) Not compatible with slam argument | True (&#9745;) |
| map_name | Yaml file loader for map Server.  | pasillo_patio.yaml |
| slam | Gmapping . (!) Not compatible with localization argument | False (&#x2612;) |
| CNN | YOLO Model + AR node  | True (&#9745;) |
| markers | Fire marker on map | True (&#9745;) |

---
##### PROJECT REPORT:
More info about the project can be found on Memory.pdf

##### FUNCTIONAL VIDEO: 
[![DEMO-Video](https://img.youtube.com/vi/4nHfei47I14/0.jpg)](https://www.youtube.com/watch?v=4nHfei47I14)
