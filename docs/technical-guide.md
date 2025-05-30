# Technical Guide

## Introduction

  This technical guide provides detailed information about the development environment setup and demo application for the MARC (Meta-Sejong AI Robotics Challenge) 2025.

## Development Environment Setup

  ### System Requirements

  - Hardware Requirements

    - **CPU**: Intel Core i5 or AMD Ryzen 5 or higher
    - **RAM**: 8GB or more
    - **GPU**: NVIDIA GPU (Optional, recommended for AI algorithm implementation)
    - **Storage**: 20GB or more free space
    - **Network**: 100Mbps or higher network connection

    <br/>

  - Software Requirements
    
    - **Operating System**: Ubuntu 22.04 LTS
    - **Python**: 3.10 or higher
    - **ROS2**: Humble Hawksbill
    - **Docker**: 20.10 or higher
    - **Docker Compose**: 2.0 or higher

  ### Development Tools Installation

  - Essential Development Tools

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

      [Optional] You can develop using the provided Dockerfile and docker-compose.yml files without installing ROS2
      
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

  - Development Environment Setup

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

  ### Application Structure

  ```
    .
    |   # Project guide documents
    ├── README.md       
    |   # Docker related files
    ├── Dockerfile      
    ├── Dockerfile.dev
    ├── docker-compose.yaml
    ├── entrypoint.sh
    |   # Commands for development and score calculation
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

  ### Key Components Description

  - ROS2 Nodes

    - **airobotics_node.py**: Main execution file
      - Entry point for the competition application
      - ROS2 node initialization and execution
      - Message publishing/subscription setup

    - **competition_task_base.py**: Abstract base class
      - Basic interface definition for competition task implementation
      - Required method declarations
      - Common utility function provision

    - **competition_task_implementation.py**: Actual implementation class
      - Specific implementation of competition tasks
      - Object detection and pose estimation logic
      - Robot control logic

    <br />

  - Utility Modules

    - **robot_node.py**: Robot control related functions

      - Robot movement control
      - Robot arm control
      - Sensor data processing

    - **robot_util.py**: Robot related utility functions
    
      - Coordinate transformation
      - Path planning
      - Collision avoidance

    - **world_coordinates_util.py**: World coordinate system related utilities
    
      - Coordinate system transformation
      - Position estimation
      - Orientation calculation


  ### Development Guide

  - Development Environment Setup

    - **Clone Repository**
      ```bash
      git clone https://github.com/<your_team_account>/metasejong-airobotics
      cd metasejong-airobotics
      ```

    - **Set Environment Variables**
      ```bash
      export ENV_METASEJONG_TEAM_NAME="your_team_name"
      export ENV_METASEJONG_TEAM_TOKEN="your_team_token"
      export ENV_METASEJONG_TEAM_TARGET_STAGE="your_target_stage"
      ```

    - **Build Docker Image**
      ```bash
      make build-dev
      ```

  - Development and Testing

    - **Demo Application Operation Flow**

    ![Demo Application Operation Flow](./_static/images/task_and_evaluation_protocol.jpg)

    The demo application operation flow consists of the following stages:

    1 Participant Application Start Request

      - **Request Message**: COMPETITOR_APP_STARTED
      - **Required Information**:
        - team ID: Participant team identifier
        - authentication token: Authentication token
        - target stage: Supported stage number
      - **Stage Goals**:
        - Stage 1: Object detection and pose estimation
        - Stage 2: Object collection and classification

    2 Participant Task Preparation Stage
    - **Authentication Verification**:
      - team ID and authentication token validation
      - Participation qualification check
    - **Environment Configuration**:
      - Virtual environment initialization
      - Robot and sensor setup
    - **Data Streaming**:
      - Virtual environment data transmission start
      - ROS2 topic setup and activation

    3 Start Response Reception
    - **Response Message**: COMPETITOR_APP_STARTED_RESPONSE
    - **Response Content**:
      - session ID
      - preparation status
      - error message (if any)
    - **Next Steps**:
      - Start main task after response confirmation
      - Retry or problem resolution in case of errors

    4 Stage 1 Task
    - **Image Analysis**:
      - Fixed camera image reception
      - Object detection algorithm application
      - Pose estimation execution
    - **Data Processing**:
      - Object position calculation
      - Orientation information extraction
      - Confidence score calculation

    5 Stage 1 Result Report and Evaluation
    - **Result Transmission**:
      - REPORT_STAGE1_RESULT request
      - Result data in JSON format

      ```json
      {
          "msg": 102,
          "session": <session id>,
          "payload": {
              "`object_detections`": [
                  {
                      "class_name": "master_shelf_can",
                      "position": [x, y, z]
                  }
              ]
          }
      }
      ```
    - **Evaluation Criteria**:
      - Accuracy of object classification
      - Precision of position estimation

    6 Stage 2 Task
    - **Robot Control**:
      - Path planning and autonomous navigation
      - Robotic arm control
      - Gripper operation
      
    - **Sorting recyclables**:
      - Recyclable type recognition with accurate pose estimation
      - Accurate sorting based on recyclability classification

## Next Steps

- Read the [API Reference](api-reference.md) for detailed API information
- Check the [Submission Guide](submit-guide.md) for submission requirements
- Review the [Getting Started Guide](getting-started.md) for basic setup
