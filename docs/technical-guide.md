# Technical Guide

## Introduction

This technical guide provides detailed information about the development environment setup, coding style, and implementation methods for the MARC (Meta-Sejong AI Robotics Challenge) 2025. It is designed to help participants understand the technical aspects of the competition and develop their solutions effectively.

## Development Environment Setup

### 1. System Requirements

#### 1.1 Hardware Requirements
- **CPU**: Intel Core i7-8700K or AMD Ryzen 7 3700X or higher
- **RAM**: 32GB or more
- **GPU**: NVIDIA RTX 3080 or NVIDIA RTX A5000 or higher
  - CUDA 11.7 or higher support
  - Minimum 8GB VRAM
- **Storage**: 100GB or more free space on SSD
- **Network**: 1Gbps or higher network connection

#### 1.2 Software Requirements
- **Operating System**: Ubuntu 22.04 LTS
- **Python**: 3.10 or higher
- **Docker**: 20.10 or higher
- **Docker Compose**: 2.0 or higher
- **NVIDIA Driver**: 525.60.13 or higher
- **CUDA**: 11.7 or higher

### 2. Development Environment Setup

#### 2.1 Docker Installation
```bash
# Install Docker
sudo apt-get update
sudo apt-get install docker.io docker-compose

# Add user to docker group
sudo usermod -aG docker $USER

# Verify installation
docker --version
docker-compose --version
```

#### 2.2 NVIDIA Container Toolkit Installation
```bash
# Add NVIDIA package repositories
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

# Install nvidia-docker2
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

#### 2.3 ROS2 Installation
```bash
# Set up sources
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2
sudo apt update
sudo apt install ros-humble-desktop
```

## Coding Style Guide

### 1. Python Coding Style

#### 1.1 General Rules
- Follow PEP 8 style guide
- Use 4 spaces for indentation
- Maximum line length: 79 characters
- Use meaningful variable and function names
- Add docstrings for all public modules, functions, classes, and methods

#### 1.2 Code Organization
```python
# Import standard library
import os
import sys

# Import third-party libraries
import numpy as np
import rospy2

# Import local modules
from . import utils
from . import config
```

#### 1.3 Documentation
```python
def process_image(image: np.ndarray) -> np.ndarray:
    """
    Process the input image for object detection.
    
    Args:
        image (np.ndarray): Input image in BGR format
        
    Returns:
        np.ndarray: Processed image in RGB format
    """
    # Implementation
    pass
```

### 2. ROS2 Node Implementation

#### 2.1 Node Structure
```python
import rclpy
from rclpy.node import Node

class CompetitionNode(Node):
    def __init__(self):
        super().__init__('competition_node')
        # Initialize publishers and subscribers
        self.publisher = self.create_publisher(
            String, 'topic_name', 10)
        self.subscription = self.create_subscription(
            String, 'topic_name', self.callback, 10)
        
    def callback(self, msg):
        # Process received message
        pass
```

#### 2.2 Message Handling
```python
from std_msgs.msg import String

def process_message(self, msg: String) -> None:
    """
    Process incoming ROS2 message.
    
    Args:
        msg (String): ROS2 message to process
    """
    try:
        # Message processing logic
        pass
    except Exception as e:
        self.get_logger().error(f'Error processing message: {str(e)}')
```

## Implementation Guide

### 1. Competition Protocol Implementation

#### 1.1 Message Types
- **COMPETITOR_APP_STARTED (101)**
- **REPORT_STAGE1_COMPLETED (102)**
- **REPORT_STAGE2_COMPLETED (103)**

#### 1.2 Response Message Types
- **COMPETITOR_APP_STARTED_RESPONSE (201)**
- **REPORT_STAGE1_COMPLETED_RESPONSE (202)**
- **REPORT_STAGE2_COMPLETED_RESPONSE (203)**

#### 1.3 Notification Message Types
- **TIME_CONSTRAINT_EXPIRED (301)**
- **COMPETITOR_REQUEST_ERROR (302)**

### 2. Robot Control Implementation

#### 2.1 Movement Control
```python
def move_robot(self, x: float, y: float, theta: float) -> None:
    """
    Move robot to specified position.
    
    Args:
        x (float): Target x coordinate
        y (float): Target y coordinate
        theta (float): Target orientation
    """
    # Implementation
    pass
```

#### 2.2 Arm Control
```python
def control_arm(self, joint_positions: List[float]) -> None:
    """
    Control robot arm joints.
    
    Args:
        joint_positions (List[float]): Target joint positions
    """
    # Implementation
    pass
```

### 3. Object Detection Implementation

#### 3.1 Image Processing
```python
def process_image(self, image: np.ndarray) -> List[Dict]:
    """
    Process image for object detection.
    
    Args:
        image (np.ndarray): Input image
        
    Returns:
        List[Dict]: List of detected objects with positions
    """
    # Implementation
    pass
```

#### 3.2 Pose Estimation
```python
def estimate_pose(self, object_detection: Dict) -> Dict:
    """
    Estimate object pose.
    
    Args:
        object_detection (Dict): Object detection result
        
    Returns:
        Dict: Estimated pose information
    """
    # Implementation
    pass
```

## Testing and Debugging

### 1. Unit Testing
```python
import unittest

class TestCompetitionNode(unittest.TestCase):
    def setUp(self):
        # Setup test environment
        pass
        
    def test_object_detection(self):
        # Test object detection
        pass
        
    def test_robot_control(self):
        # Test robot control
        pass
```

### 2. ROS2 Testing
```python
import launch
import launch_ros.actions

def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='airobotics_app',
            executable='competition_node',
            name='test_node'
        ),
        # Add test nodes
    ])
```

### 3. Debugging Tools
- **rqt**: ROS visualization and debugging tools
- **ros2 topic echo**: Monitor topic messages
- **ros2 node info**: Get node information
- **ros2 service call**: Call ROS services

## Performance Optimization

### 1. Code Optimization
- Use efficient data structures
- Minimize memory allocations
- Optimize loops and conditionals
- Use vectorized operations when possible

### 2. ROS2 Optimization
- Use appropriate QoS settings
- Optimize message serialization
- Minimize topic publishing frequency
- Use efficient message types

### 3. Resource Management
- Monitor CPU and memory usage
- Implement proper cleanup
- Handle exceptions gracefully
- Log important events

## Next Steps

- Read the [API Reference](api-reference.md) for detailed API information
- Check the [Submission Guide](submit-guide.md) for submission requirements
- Review the [Getting Started Guide](getting-started.md) for basic setup
