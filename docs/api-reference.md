# API Reference Documentation

## Overview

This document provides a detailed description of the ROS2 interface used in the MetaSejong AI Robotics Challenge 2025. Participants can understand the communication methods with the platform and implement necessary functionalities through this document.

## ROS2 Topic List

The following is a list of all ROS2 topics provided by the MetaSejong platform:

| Topic Name | Message Type | Publisher → Subscriber | Description |
|----------|------------|----------|------|
| **Competition Protocol** |  |  | 
| `/metasejong2025/competitor_request` | std_msgs/msg/String (JSON) | Participant → Platform | Request message sent by participant to platform |
| `/metasejong2025/competitor_response` | std_msgs/msg/String (JSON) | Platform → Participant | Response message sent by platform to participant's request |
| `/metasejong2025/competitor_notification` | std_msgs/msg/String (JSON) | Platform → Participant | Notification message sent by platform to participant |
|  **Sensor Data** |  |  | 
| `/metasejong2025/cameras/<field_name>/image_raw` | sensor_msgs/Image | Platform → Participant | RGB image captured by fixed camera |
| `/metasejong2025/cameras/<field_name>/camera_info` | sensor_msgs/msg/CameraInfo | Platform → Participant | Internal/external parameters of fixed camera |
| `/metasejong2025/robot_camera/camera_info` | sensor_msgs/msg/CameraInfo | Platform → Participant | Internal/external parameters of robot camera |
| `/metasejong2025/robot_camera/color` | sensor_msgs/msg/Image | Platform → Participant | RGB image captured by robot camera |
| `/metasejong2025/robot_camera/depth` | sensor_msgs/msg/Image | Platform → Participant | Depth image captured by robot camera |
| `/metasejong2025/scan` | sensor_msgs/msg/LaserScan | Platform → Participant | Scan data measured by LiDAR sensor |
|  **Robot State, Navigation and Control** |  |  | 
| `/metasejong2025/odom` | nav_msgs/Odometry | Platform → Participant | Current position and orientation of robot |
| `/metasejong2025/tf` | tf2_msgs/TFMessage | Platform → Participant | Coordinate system transformation information |
| `/metasejong2025/cmd_vel` | geometry_msgs/Twist | Participant → Platform | Robot movement velocity control command |
| `/metasejong2025/ppcmd` | std_msgs/msg/String | Participant → Platform | Robot arm motion control command |

## Competition Protocol

### 1. Participant Request Message
- **Topic**: `/metasejong2025/competitor_request`
- **Type**: std_msgs/msg/String (JSON)
- **Description**: Request message sent by participant to platform
- **Basic Data Structure**:
  ```json
  {
      "msg": <message type>,
      "session": <null or session string>,
      "payload": {
          <payload contents>
      }
  }
  ```

- **Message Types and Payload Format**:
  - **COMPETITOR_APP_STARTED (101)**: Participant Application Start Request
    - **Description**: Request message to notify platform that participant application is ready to perform stage tasks
    - **Notes**: 
      - session value is ignored (called before session key issuance)
      - team and token values are obtained from competition registration
      - stage is a selection value for whether to perform up to stage 1 or stage 2
    - **Data Structure**:
      ```json
      {
          "msg": 101,
          "session": "",
          "payload": {
            "team": "your_team_id",
            "token": "your_auth_token",
            "stage": 2
          }
      }
      ```

  - **REPORT_STAGE1_COMPLETED (102)**: Stage 1 Completion Report Request
    - **Description**: Request message to report completion of Stage 1 tasks to platform
    - **Notes**: 
      - Use session key received from COMPETITOR_APP_STARTED response
    - **Data Structure**:
      ```json
      {
          "msg": 102,
          "session": "<session key>",
          "payload": {
              "object_detections": [
                 {"class_name": "master_shelf_can", "position": [x, y, z]},
                 ... 
              ],
          }
      }
      ```

  - **REPORT_STAGE2_COMPLETED (103)**: Stage 2 Completion Report Request
    - **Description**: Request message to report completion of Stage 2 tasks to platform
    - **Notes**: 
      - Use session key received from COMPETITOR_APP_STARTED response
      - payload is ignored
    - **Data Structure**:
      ```json
      {
          "msg": 103,
          "session": "<session key>",
          "payload": {
          }
      }
      ```

### 2. Platform Response Message
- **Topic**: `/metasejong2025/competitor_response`
- **Type**: std_msgs/msg/String (JSON)
- **Description**: Platform's response message to participant's request
- **Basic Data Structure**:
  ```json
  {
      "msg": <message type>,
      "status": <status code>,
      "status_message": <status message>,
      "result": {
          <result contents>
      }
  }
  ```
  - **msg**: Response message type in 20X format for 10X format request messages
  - **status**: 1(success) or 0(failed)
  - **status_message**: "OK" when status is 1, error cause when 0
  - **result**: Response content according to message type

- **Message Types and Payload Format**:
  - **COMPETITOR_APP_STARTED_RESPONSE (201)**: Participant Application Start Response
    - **Description**: Response to COMPETITOR_APP_STARTED(101) request
    - **Notes**: 
      - Session key is issued upon successful team authentication
      - Status is set to 0 and error cause is included in status_message upon authentication failure
    - **Data Structure**:
      ```json
      {
          "msg": 201,
          "status": 1,
          "status_message": "OK",
          "result": {
              "session": "<session key>"
          }
      }
      ```

  - **REPORT_STAGE1_COMPLETED_RESPONSE (202)**: Stage 1 Completion Report Response
    - **Description**: Response to REPORT_STAGE1_COMPLETED(102) request
    - **Notes**: 
      - Next action is determined based on ENV_METASEJONG_TEAM_TARGET_LEVEL value after receiving response
      - Status is set to 0 and error cause is included in status_message upon processing error
    - **Data Structure**:
      ```json
      {
          "msg": 202,
          "status": 1,
          "status_message": "OK",
          "result": {
              "dummy": ""
          }
      }
      ```

  - **REPORT_STAGE2_COMPLETED_RESPONSE (203)**: Stage 2 Completion Report Response
    - **Description**: Response to REPORT_STAGE2_COMPLETED(103) request
    - **Notes**: 
      - Application can be terminated after receiving response
      - Status is set to 0 and error cause is included in status_message upon processing error
    - **Data Structure**:
      ```json
      {
          "msg": 203,
          "status": 1,
          "status_message": "OK",
          "result": {
              "dummy": ""
          }
      }
      ```

### 3. Platform Notification Message
- **Topic**: `/metasejong2025/competitor_notification`
- **Type**: std_msgs/msg/String (JSON)
- **Description**: Notification message sent by platform to participant
- **Basic Data Structure**:
  ```json
  {
      "msg": <message type>,
      "session": "<session key>",
      "payload": {
          <payload contents>
      }
  }
  ```

- **Message Types and Payload Format**:
  - **TIME_CONSTRAINT_EXPIRED (301)**: Time Limit Expiration Notification
    - **Description**: Notification message indicating expiration of competition scenario time limit
    - **Notes**: 
      - Application must be terminated immediately after receiving message
      - Platform stops receiving request messages after time limit expiration
    - **Data Structure**:
      ```json
      {
          "msg": 301,
          "session": "<session key>",
          "payload": {
              "dummy": ""
          }
      }
      ```

  - **COMPETITOR_REQUEST_ERROR (302)**: Participant Request Error Notification
    - **Description**: Notification message for errors occurring during platform operation
    - **Data Structure**:
      ```json
      {
          "msg": 302,
          "session": "<session key>",
          "payload": {
              "error_message": "Error message"
          }
      }
      ```

## Sensor Data

### 1. Fixed Camera Data
- **Topic**: `/metasejong2025/cameras/<field_name>/image_raw`
- **Type**: sensor_msgs/Image
- **Description**: RGB image captured by fixed camera
- **Data Structure**:
  ```json
  {
      "header": {
          "stamp": {"sec": 0, "nanosec": 0},
          "frame_id": "camera_link"
      },
      "height": 480,
      "width": 640,
      "encoding": "rgb8",
      "is_bigendian": 0,
      "step": 1920,
      "data": [/* Image data */]
  }
  ```

### 2. Camera Information
- **Topic**: `/metasejong2025/cameras/<field_name>/camera_info`
- **Type**: sensor_msgs/msg/CameraInfo
- **Description**: Internal/external parameters of camera
- **Data Structure**:
  ```json
  {
      "header": {
          "stamp": {"sec": 0, "nanosec": 0},
          "frame_id": "camera_link"
      },
      "height": 480,
      "width": 640,
      "distortion_model": "plumb_bob",
      "d": [0.0, 0.0, 0.0, 0.0, 0.0],
      "k": [525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0],
      "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
      "p": [525.0, 0.0, 319.5, 0.0, 0.0, 525.0, 239.5, 0.0, 0.0, 0.0, 1.0, 0.0]
  }
  ```

### 3. Robot Camera Data
- **Topic**: `/metasejong2025/robot_camera/color`
- **Type**: sensor_msgs/msg/Image
- **Description**: RGB image captured by robot camera
- **Data Structure**: Same as fixed camera data

### 4. Robot Camera Depth Data
- **Topic**: `/metasejong2025/robot_camera/depth`
- **Type**: sensor_msgs/msg/Image
- **Description**: Depth image captured by robot camera
- **Data Structure**:
  ```json
  {
      "header": {
          "stamp": {"sec": 0, "nanosec": 0},
          "frame_id": "robot_camera_link"
      },
      "height": 480,
      "width": 640,
      "encoding": "32FC1",
      "is_bigendian": 0,
      "step": 2560,
      "data": [/* Depth data */]
  }
  ```

### 5. LiDAR Scan Data
- **Topic**: `/metasejong2025/scan`
- **Type**: sensor_msgs/msg/LaserScan
- **Description**: Scan data measured by LiDAR sensor
- **Data Structure**:
  ```json
  {
      "header": {
          "stamp": {"sec": 0, "nanosec": 0},
          "frame_id": "laser_link"
      },
      "angle_min": -3.14159,
      "angle_max": 3.14159,
      "angle_increment": 0.0174533,
      "time_increment": 0.0,
      "scan_time": 0.1,
      "range_min": 0.1,
      "range_max": 30.0,
      "ranges": [/* Distance data */],
      "intensities": [/* Intensity data */]
  }
  ```

## Robot State, Navigation and Control

### 1. Robot Position Information
- **Topic**: `/metasejong2025/odom`
- **Type**: nav_msgs/Odometry
- **Description**: Current position and orientation of robot
- **Data Structure**:
  ```json
  {
      "header": {
          "stamp": {"sec": 0, "nanosec": 0},
          "frame_id": "odom"
      },
      "child_frame_id": "base_link",
      "pose": {
          "position": {
              "x": 0.0,
              "y": 0.0,
              "z": 0.0
          },
          "orientation": {
              "x": 0.0,
              "y": 0.0,
              "z": 0.0,
              "w": 1.0
          }
      },
      "twist": {
          "linear": {
              "x": 0.0,
              "y": 0.0,
              "z": 0.0
          },
          "angular": {
              "x": 0.0,
              "y": 0.0,
              "z": 0.0
          }
      }
  }
  ```

### 2. Coordinate System Transformation Information
- **Topic**: `/metasejong2025/tf`
- **Type**: tf2_msgs/TFMessage
- **Description**: Coordinate system transformation information
- **Data Structure**:
  ```json
  {
      "transforms": [
          {
              "header": {
                  "stamp": {"sec": 0, "nanosec": 0},
                  "frame_id": "odom"
              },
              "child_frame_id": "base_link",
              "transform": {
                  "translation": {
                      "x": 0.0,
                      "y": 0.0,
                      "z": 0.0
                  },
                  "rotation": {
                      "x": 0.0,
                      "y": 0.0,
                      "z": 0.0,
                      "w": 1.0
                  }
              }
          }
      ]
  }
  ```

### 3. Robot Movement Command
- **Topic**: `/metasejong2025/cmd_vel`
- **Type**: geometry_msgs/Twist
- **Description**: Robot movement velocity control command
- **Data Structure**:
  ```json
  {
      "linear": {
          "x": 0.0,  // Forward/backward velocity (m/s)
          "y": 0.0,  // Left/right velocity (m/s)
          "z": 0.0   // Up/down velocity (m/s)
      },
      "angular": {
          "x": 0.0,  // Roll rotation velocity (rad/s)
          "y": 0.0,  // Pitch rotation velocity (rad/s)
          "z": 0.0   // Yaw rotation velocity (rad/s)
      }
  }
  ```

### 4. Robot Arm Control Command
- **Topic**: `/metasejong2025/ppcmd`
- **Type**: std_msgs/msg/String
- **Description**: Robot arm motion control command
- **Data Structure**:
  `<picking_quatenion_angle_for_gripper> <picking_endpoint_for_gripper> <placing_quatenion_angle_for_gripper> <placing_endpoint_for_gripper>`
  
  <picking_quatenion_angle_for_gripper>: Gripper entry angle for picking up waste, 4 real numbers (quaternion) separated by spaces
  <picking_endpoint_for_gripper>: Gripper endpoint position for picking up waste, 3 real numbers (x, y, z) separated by spaces
  <placing_quatenion_angle_for_gripper>: Gripper entry angle for disposing waste, 4 real numbers (quaternion) separated by spaces
  <placing_endpoint_for_gripper>: Gripper endpoint position for disposing waste, 3 real numbers (x, y, z) separated by spaces
