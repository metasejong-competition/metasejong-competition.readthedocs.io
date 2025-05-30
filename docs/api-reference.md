# API Reference

## Introduction

This document provides detailed information about the ROS2 interface for the MARC(Meta-Sejong AI Robotics Challenge) 2025. It includes message formats, topic descriptions, and implementation examples to help participants develop their solutions effectively.

## ROS2 Topic List

The following is a list of all ROS2 topics provided by the Meta-Sejong platform:

| Topic Name | Message Type | Publish/Subscribe | Description |
|------------|--------------|-------------------|-------------|
| `/metasejong2025/camera/image_raw` | `sensor_msgs/Image` | Subscribe | Raw camera image data |
| `/metasejong2025/camera/depth` | `sensor_msgs/Image` | Subscribe | Depth image data |
| `/metasejong2025/odom` | `nav_msgs/Odometry` | Subscribe | Robot odometry data |
| `/metasejong2025/tf` | `tf2_msgs/TFMessage` | Subscribe | Robot transformation data |
| `/metasejong2025/cmd_vel` | `geometry_msgs/Twist` | Publish | Robot velocity commands |
| `/metasejong2025/arm_control` | `metasejong_msgs/ArmControl` | Publish | Robot arm control commands |
| `/metasejong2025/object_info` | `metasejong_msgs/ObjectInfo` | Subscribe | Object detection information |
| `/metasejong2025/task_status` | `metasejong_msgs/TaskStatus` | Subscribe | Task execution status |

### Topic Descriptions

#### Camera Data Topics
- **`/metasejong2025/camera/image_raw`**
  ```json
  {
    "header": {
      "stamp": {
        "sec": 0,
        "nanosec": 0
      },
      "frame_id": "camera"
    },
    "height": 480,
    "width": 640,
    "encoding": "rgb8",
    "is_bigendian": 0,
    "step": 1920,
    "data": [0, 0, 0, ...]
  }
  ```

- **`/metasejong2025/camera/depth`**
  ```json
  {
    "header": {
      "stamp": {
        "sec": 0,
        "nanosec": 0
      },
      "frame_id": "camera"
    },
    "height": 480,
    "width": 640,
    "encoding": "32FC1",
    "is_bigendian": 0,
    "step": 2560,
    "data": [0.0, 0.0, 0.0, ...]
  }
  ```

#### Robot State Topics
- **`/metasejong2025/odom`**
  ```json
  {
    "header": {
      "stamp": {
        "sec": 0,
        "nanosec": 0
      },
      "frame_id": "odom"
    },
    "child_frame_id": "base_link",
    "pose": {
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
      "covariance": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ...]
    },
    "twist": {
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
      },
      "covariance": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ...]
    }
  }
  ```

#### Control Command Topics
- **`/metasejong2025/cmd_vel`**
  ```json
  {
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
  ```

- **`/metasejong2025/arm_control`**
  ```json
  {
    "joint_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "gripper_position": 0.0
  }
  ```

#### Object Information Topics
- **`/metasejong2025/object_info`**
  ```json
  {
    "header": {
      "stamp": {
        "sec": 0,
        "nanosec": 0
      },
      "frame_id": "camera"
    },
    "objects": [
      {
        "id": 1,
        "class": "object_class",
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
        },
        "confidence": 0.0
      }
    ]
  }
  ```

#### Task Status Topics
- **`/metasejong2025/task_status`**
  ```json
  {
    "header": {
      "stamp": {
        "sec": 0,
        "nanosec": 0
      },
      "frame_id": "task"
    },
    "task_id": 1,
    "status": "RUNNING",
    "progress": 0.0,
    "message": "Task in progress"
  }
  ```

## Implementation Example

### Basic ROS2 Node Implementation
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from metasejong_msgs.msg import ArmControl

class CompetitionNode(Node):
    def __init__(self):
        super().__init__('competition_node')
        
        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/metasejong2025/camera/image_raw',
            self.image_callback,
            10)
            
        self.depth_sub = self.create_subscription(
            Image,
            '/metasejong2025/camera/depth',
            self.depth_callback,
            10)
            
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/metasejong2025/cmd_vel',
            10)
            
        self.arm_control_pub = self.create_publisher(
            ArmControl,
            '/metasejong2025/arm_control',
            10)
            
    def image_callback(self, msg):
        # Process image data
        pass
        
    def depth_callback(self, msg):
        # Process depth data
        pass
        
    def move_robot(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)
        
    def control_arm(self, joint_positions, gripper_position):
        msg = ArmControl()
        msg.joint_positions = joint_positions
        msg.gripper_position = gripper_position
        self.arm_control_pub.publish(msg)

def main():
    rclpy.init()
    node = CompetitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Important Notes

### Message Frequency
- Camera topics: 30Hz
- Odometry topic: 100Hz
- Control command topics: 50Hz
- Object information topic: 10Hz
- Task status topic: 1Hz

### Error Handling
- Implement proper error handling for all callbacks
- Log errors using ROS2 logging system
- Handle message deserialization errors
- Implement timeout mechanisms for critical operations

### Performance Considerations
- Minimize message processing time
- Use efficient data structures
- Implement proper cleanup
- Monitor system resources

### Debugging
- Use ROS2 logging for debugging
- Monitor topic messages using `ros2 topic echo`
- Check node information using `ros2 node info`
- Use visualization tools for debugging
