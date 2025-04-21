# Technical Guide

## Overview

This guide provides detailed technical information about the development environment setup and demo application for the MetaSejong AI Robotics Challenge 2025.

## Development Environment Setup

### 1. System Requirements

#### 1.1 Hardware Requirements
- **CPU**: Intel Core i5 or AMD Ryzen 5 or higher
- **RAM**: 8GB or more
- **GPU**: NVIDIA GPU (optional, recommended for AI algorithm implementation)
- **Storage**: 20GB or more free space
- **Network**: 100Mbps or higher network connection

#### 1.2 Software Requirements
- **Operating System**: Ubuntu 22.04 LTS
- **Python**: 3.10 or higher
- **ROS2**: Humble Hawksbill
- **Docker**: 20.10 or higher
- **Docker Compose**: 2.0 or higher

### 2. Development Tools Installation

#### 2.1 Essential Development Tools
- **Git**: Version control and collaboration
  ```bash
  sudo apt update
  sudo apt install git
  ```
- **Docker**: Container-based development environment management
  ```bash
  # Install Docker
  sudo apt install docker.io
  sudo systemctl enable docker
  sudo systemctl start docker
  
  # Install Docker Compose
  sudo apt install docker-compose
  ```
- **ROS2**: Robot control system development
  ```bash
  # Install ROS2 Humble
  sudo apt install software-properties-common
  sudo add-apt-repository universe
  sudo apt update && sudo apt install curl
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  sudo apt update
  sudo apt install ros-humble-desktop
  ```

#### 2.2 Development Environment Setup
- **Python Virtual Environment Setup**
  ```bash
  # Create Python virtual environment
  python3 -m venv venv
  source venv/bin/activate
  
  # Install required packages
  pip install -r requirements.txt
  ```
- **ROS2 Workspace Setup**
  ```bash
  # Create ROS2 workspace
  mkdir -p ~/metasejong_ws/src
  cd ~/metasejong_ws
  colcon build
  ```

## Demo Application

### 1. Application Structure

```
.
|   # Project guide document
├── README.md       
|   # Docker related files
├── Dockerfile      
├── Dockerfile.dev
├── docker-compose.yaml
├── entrypoint.sh
|   # Commands for development, score calculation, etc.
├── Makefile
|   # Participant application workspace (ROS2 Workspace)
└── metasejong_competitor_ws
    └── src
        |   
        └── airobotics_app
            |   # ROS2 package definition and module dependencies
            ├── package.xml
            ├── requirements.txt
            |   # Participant ROS Node implementation
            ├── airobotics_node
            │   ├── __init__.py
            |   |   # ROS Node execution entry point
            │   ├── airobotics_node.py
            |   |   # Basic competition participant application template example implementation (abstract implementation)
            │   ├── competition_task_base.py
            |   |   # Basic competition participant application template example implementation
            │   ├── competition_task_implementation.py
            │   ├── competitor_request_message.py
            |   |   # Utility function example implementation
            │   ├── robot_node.py
            │   ├── robot_util.py
            │   └── world_coordinates_util.py
            ├── resource
            │   ├── airobotics_app
            │   └── metasejong
            ├── setup.cfg
            └── setup.py
```

### 2. Key Components Description

#### 2.1 ROS2 Nodes
- **airobotics_node.py**: Main execution file
  - Entry point for competition application
  - ROS2 node initialization and execution
  - Message publishing/subscribing setup

- **competition_task_base.py**: Abstract base class
  - Basic interface definition for competition task implementation
  - Required method declarations
  - Common utility functions

- **competition_task_implementation.py**: Actual implementation class
  - Concrete implementation of competition tasks
  - Object detection and pose estimation logic
  - Robot control logic

#### 2.2 Utility Modules
- **robot_node.py**: Robot control related functions
  - Robot movement control
  - Robot arm control
  - Sensor data processing

- **robot_util.py**: Robot related utility functions
  - Coordinate transformation
  - Path planning
  - Collision avoidance

- **world_coordinates_util.py**: World coordinate system utilities
  - Coordinate system transformation
  - Position estimation
  - Orientation calculation

### 3. Development Guide

#### 3.1 Development Environment Setup
1. **Repository Cloning**
   ```bash
   git clone https://github.com/<your_team_account>/metasejong-airobotics
   cd metasejong-airobotics
   ```

2. **Environment Variable Setup**
   ```bash
   export ENV_METASEJONG_TEAM_NAME="your_team_name"
   export ENV_METASEJONG_TEAM_TOKEN="your_team_token"
   export ENV_METASEJONG_TEAM_TARGET_STAGE="your_target_stage"
   ```

3. **Docker Image Build**
   ```bash
   make build-dev
   ```

#### 3.2 Development and Testing
1. **Development Environment Execution**
   ```bash
   make up-dev
   ```

2. **Code Modification and Testing**
   - Modify files in the `airobotics_node` directory
   - Restart ROS2 nodes to apply changes
   - Debug using logs

3. **Test Execution**
   ```bash
   # Run unit tests
   python -m pytest tests/
   
   # Run full system tests
   make test
   ```

### 4. Troubleshooting

#### 4.1 Common Issues
1. **ROS2 Node Execution Failure**
   - Check ROS2 environment setup
   - Verify required package installation
   - Check log files

2. **Docker Container Execution Failure**
   - Check Docker service status
   - Verify image build errors
   - Check container logs

3. **Performance Issues**
   - Check system resource usage
   - Verify GPU acceleration settings
   - Check network bandwidth

#### 4.2 Debugging Guide
1. **Log Checking**
   ```bash
   # Check ROS2 logs
   ros2 topic echo /rosout
   
   # Check Docker logs
   docker logs <container_id>
   ```

2. **Performance Monitoring**
   ```bash
   # Monitor system resources
   htop
   
   # Check GPU usage
   nvidia-smi
   ```

3. **Network Diagnostics**
   ```bash
   # Check network connection
   ping <target_ip>
   
   # Check ports
   netstat -tulpn
   ```
