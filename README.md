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
##### MODEL PERFORMANCE:
![A](imgs/confusion_matrix_normalized.png)
![A](imgs/pr-curve.png)
![A](imgs/results.png)
---
##### EXPERIMENTS RESULTS:
![A](imgs/inferencia.png)
![A](imgs/collage-demo.png)

##### TRAFFIC LIGHT STATE AND EMERGENCY BRAKE SUPERVISOR
![A](imgs/traffic_light_task_diagram_.png)
![A](imgs/classic_color_segmentation.png)


---
##### PROJECT REPORT:
More info about the project can be found on Memory.pdf

##### FUNCTIONAL VIDEO: 
[![DEMO-Video](https://img.youtube.com/vi/4nHfei47I14/0.jpg)](https://www.youtube.com/watch?v=4nHfei47I14)
