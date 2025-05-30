# API Reference

## Introduction

This document provides detailed information about the ROS2 interface for the MARC(Meta-Sejong AI Robotics Challenge) 2025. It includes message formats, topic descriptions, and implementation examples to help participants develop their solutions effectively.

## ROS2 Topic List

The following is a list of all ROS2 topics provided by the Meta-Sejong platform:

**Publish** means Meta-Sejong publish message throuth this topic to provide some information to competitor's application
**Subscribe** means Meta-Sejong subscribe this topic to receive some message from competitor's application

| Topic Name | Message Type | Publish/Subscribe | Description |
|------------|--------------|-------------------|-------------|
| /metasejong2025/competitor_request | std_msgs/msg/String |  Subscribe | Participants send competition-related requests to the Meta-Sejong Platform |
| /metasejong2025/competitor_notification  | std_msgs/msg/String |  Publish | Response messages are sent in reply to the participant’s requests |
| /metasejong2025/competitor_response | std_msgs/msg/String |  Publish | The platform delivers notification messages to the participant |
<br />
| /metasejong2025/cameras/*field_name*/camera_info | sensor_msgs/msg/CameraInfo | Publish | The camera’s installation position and orientation information |
| /metasejong2025/cameras/*field_name*/image_raw | sensor_msgs/msg/Image | Publish | Streaming images from the camera |
<br />
| /metasejong2025/map | nav_msgs/msg/OccupancyGrid |  Publish | Map data for autonomous navigation provided via the ROS 2 Nav2 Map Server |
<br />
| /metasejong2025/robot/center_camera_depth | sensor_msgs/msg/Image | Publish | Depth video stream captured by the RGB+Depth camera mounted at the front center of the robot |
| /metasejong2025/robot/center_camera_image | sensor_msgs/msg/Image | Publish | RGB video stream captured by the RGB+Depth camera mounted at the front center of the robot |
| /metasejong2025/robot/center_camera_info | sensor_msgs/msg/CameraInfo | Publish | Position and orientation information of the RGB+Depth camera mounted at the front center of the robot |
| /metasejong2025/robot/left_camera_image | sensor_msgs/msg/Image | Publish | RGB video stream captured by the RGB+Depth camera mounted at the front left of the robot |
| /metasejong2025/robot/left_camera_info | sensor_msgs/msg/CameraInfo | Publish | Position and orientation information of the RGB camera mounted at the front left of the robot |
| /metasejong2025/robot/right_camera_image | sensor_msgs/msg/Image | Publish | RGB video stream captured by the RGB camera mounted at the front right of the robot |
| /metasejong2025/robot/right_camera_info | sensor_msgs/msg/CameraInfo | Publish | RGB video stream captured by the RGB+Depth camera mounted at the front right of the robot |
| /metasejong2025/robot/scan | sensor_msgs/msg/LaserScan | Publish | Sensor data collected from the LiDAR sensor mounted on the robot |
<br />
| /metasejong2025/robot/cmd_vel | geometry_msgs/msg/Twist | Subscribe | Movement command for autonomous navigation of the robot |
<br />
| /metasejong2025/robot/ppcmd | std_msgs/msg/String | Subscribe | Command for controlling the robot arm mounted on the robot |
| /metasejong2025/robot/odom | nav_msgs/msg/Odometry | Publish | Required for tracking the robot's movement path during autonomous navigation |


