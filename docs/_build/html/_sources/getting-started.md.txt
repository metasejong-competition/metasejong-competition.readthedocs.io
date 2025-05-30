# Getting Started Guide

## Introduction

Welcome to the MARC(Meta-Sejong AI Robotics Challenge) 2025! This competition is a co-located event of IEEE MetaCom 2025 and part of the MetaCom 2025 Student Challenge Programs. It provides an innovative platform that combines metaverse and AI robotics technologies.

The MARC(Meta-Sejong AI Robotics Challenge) 2025 takes place in a metaverse space modeled after Sejong University. Participating teams will solve given tasks using Embodied AI technology in this virtual environment. Participants will control robots in the virtual Sejong University campus and perform missions by applying various AI technologies.

This guide provides step-by-step instructions for the entire process, from setting up the development environment to mission execution. It details how to interact with the virtual environment through the ROS2 standard interface, helping participants prepare smoothly for the competition.

## System Architecture

![Competition Development Environment](./_static/images/interface_architecture.jpg)

The competition system consists of the following main components:

1. **Meta-Sejong Platform**
The Meta-Sejong Platform is a metaverse virtual environment of Sejong University built on NVIDIA IsaacSim. It provides the following key features:

    - Real-time 3D virtual environment simulation
    - Physics engine-based precise robot motion simulation
    - Various sensor data streaming
    - Real-time robot control interface

2. **Participant Development Environment**
    - ROS2-based application development environment
    - AI algorithm implementation and testing environment
    - Docker container-based independent development space

3. **Communication Interface**
The API required for participants to perform missions is provided through the ROS2 interface. Detailed API documentation is available in a separate document, including the following main features:

    - Robot control interface
    - Sensor data collection
    - Environment information access
    - Mission progress monitoring

The Meta-Sejong Platform serves as a learning environment for the AI Robotics applications developed by participants. This allows testing and optimization of various scenarios without using actual robots.

## Prerequisites

To participate in the Meta-Sejong Platform, you must meet the following requirements. Each requirement is essential for competition participation and development:
For detailed information about the prerequisites, please refer to the [Technical Guide](technical-guide.md).

### 1. Hardware Requirements

#### Meta-Sejong Platform Execution Environment
- **CPU**: Intel Core i7-8700K or AMD Ryzen 7 3700X or higher
- **RAM**: 32GB or more
- **GPU**: NVIDIA RTX 3080 or NVIDIA RTX A5000 or higher
  - CUDA 11.7 or higher support
  - Minimum 8GB VRAM
- **Storage**: 100GB or more free space on SSD
- **Network**: 1Gbps or higher network connection

#### Participant Development Environment
- **CPU**: Intel Core i5 or AMD Ryzen 5 or higher
- **RAM**: 8GB or more
- **GPU**: NVIDIA GPU (optional, recommended for AI algorithm implementation)
- **Storage**: 20GB or more free space
- **Network**: 100Mbps or higher network connection

### 2. Software Requirements

#### Meta-Sejong Platform
- **Operating System**: Ubuntu 22.04 LTS
- **Python**: 3.10 or higher
- **Docker**: 20.10 or higher
- **Docker Compose**: 2.0 or higher
- **NVIDIA Driver**: 525.60.13 or higher
- **CUDA**: 11.7 or higher

#### Participant Development Environment
- **Operating System**: Ubuntu 22.04 LTS
- **Python**: 3.10 or higher
- **ROS2**: Humble Hawksbill
- **Docker**: 20.10 or higher
- **Docker Compose**: 2.0 or higher

### 3. Development Tools and Knowledge Requirements

#### Required Development Tools
- **Git**: Version control and collaboration
- **Docker**: Container-based development environment management
- **ROS2**: Robot control system development
- **Python IDE**: Code writing and debugging

#### Required Knowledge
- **Python Programming**: Algorithm implementation and debugging
- **ROS2 Basic Concepts**: Topics, services, actions, etc.
- **Docker Basic Usage**: Image building and container execution
- **Basic AI/ML Knowledge**: Object detection, pose estimation, etc.

### 4. Competition Participation Requirements

#### Required Information
- **Team Information**: Team name, team member information
- **Application Form**: Form provided by the competition organizers
- **Team ID**: Issued after application approval
- **Authentication Token**: Issued after application approval

#### Submission Requirements
- **GitHub Repository**: Private repository recommended
- **Code Submission**: Final code in main/master branch
- **Documentation**: HOWTORUN.md file included
- **Execution Method**: Must be executable with make up-prod command

## Installation Method

### 1. Meta-Sejong Platform Installation

#### 1.1 System Prerequisites Setup

The Meta-Sejong Platform is an IsaacSim simulation application distributed via Docker, including a GUI. X11 setup is required to run GUI applications in Docker.

1. **X11 Setup Verification**
   ```bash
   # Check X11 setup
   echo $DISPLAY
   ```
   - Output should be in the format `:1` or `localhost:1`
   - If X11 is not set up, refer to [Ubuntu X Architecture](https://wiki.ubuntu.com/X/Architecture) for setup instructions

#### 1.2 Competition Repository Clone

1. **Repository Clone**
   ```bash
   # Clone Meta-Sejong Platform repository
   git clone https://github.com/metasejong-competition/metacom2025-metasejong
   cd metacom2025-metasejong
   ```

2. **Repository Structure**
   - `docker-compose.yml`: Environment configuration file
   - `Makefile`: Build and execution commands
   - Example code and documentation

#### 1.3 Docker Image Download

1. **Image Download**
   ```bash
   # Download via Makefile
   make download
   ```
   - Or direct download: [metasejong-metacom2025-with-playground-r06.tar](https://drive.google.com/file/d/1fWGCelUpPuHxYX9ECWaa8SxL5NXOrL72/view?usp=drive_link)
   - Do not change the downloaded filename

2. **Docker Image Load**
   ```bash
   # Load downloaded image
   make load
   ```

#### 1.4 Platform Execution

1. **X11 Activation**
   ```bash
   # Activate X11 (run once)
   make setup
   ```

2. **Platform Execution**
   ```bash
   # Execute Meta-Sejong Platform
   make run
   ```
   - Note that there may be performance limitations during execution

### 2. Participant Development Environment Setup

#### 2.1 Preparation

1. **Application**
   - Submit application at [Application Submission](https://metasejong-competition.github.io/)
   - Team ID and authentication token issued after application review
   - Team information received via email with confirmation message

#### 2.2 Development Environment Repository Setup

1. **Repository Fork**
   - Fork the [Development Environment Repository](https://github.com/metasejong-competition/metasejong-airobotics)
   - Private repository recommended

2. **Repository Clone**
   ```bash
   # Clone development environment repository
   git clone https://github.com/<your_team_account>/metasejong-airobotics
   cd metasejong-airobotics
   ```

#### 2.3 Environment Variable Setup

1. **Required Environment Variables**
   ```bash
   # Set environment variables
   export ENV_METASEJONG_TEAM_NAME="your_team_name"
   export ENV_METASEJONG_TEAM_TOKEN="your_team_token"
   export ENV_METASEJONG_TEAM_TARGET_STAGE="your_target_stage"

   # Permanent environment variable setup (optional)
   echo "export ENV_METASEJONG_TEAM_NAME=\"your_team_name\"" >> ~/.bashrc
   ```

2. **Demo Environment Variable Values**
   |Environment Variable|Value|
   |---|---|
   |ENV_METASEJONG_TEAM_NAME|demo_team|
   |ENV_METASEJONG_TEAM_TOKEN|18471a11421511d3c3a9f56c53bc8d57|
   |ENV_METASEJONG_TEAM_TARGET_STAGE|2|

#### 2.4 Execution

1. **Execution Command**
   ```bash
   make run
   ```

### 3. Submission

#### 3.1 Submission Preparation

1. **Code Organization**
   - Push final code to 'main' or 'master' branch
   - Document execution method (HOWTORUN.md)

2. **Repository Sharing**
   - Register organizer's GitHub account as collaborator
   - Verify private repository settings

#### 3.2 Submission Requirements

1. **Required Files**
   - Modified Makefile
   - Modified Dockerfile
   - Modified docker-compose.yml
   - HOWTORUN.md (if needed)

2. **Execution Method**
   - Must be executable with `make run` command
   - Document any special settings in detail

## Demo Application

For detailed structure and development guide of the demo application, please refer to the [Technical Guide](technical-guide.md).

### 1. Demo Application Execution

The demo application operates using answer sheets generated by the Meta-Sejong Platform running on your local computer for demonstration and learning purposes. The answer sheets provide information about the types and locations of randomly generated trash objects based on the competition scenario definition. The demo application implements 물체 탐지 및 위치 추정 technologies that participants need to develop in a mockup form, utilizing the answer sheets for competition protocol implementation and robot control.

#### 1.1 Answer Sheet File Connection
- **Answer Sheet File Path**: `<Meta-Sejong Platform execution path>/scenario-data/answer-sheets/<scenario name>_answer_sheet.yaml`
- **Demo Application Answer Sheet Location**: `<Development environment project>/demo/demo_answer_sheet.yaml`
- **Note**: Since answer sheets are newly generated each time the Meta-Sejong Platform is executed, it is recommended to set up a symbolic link rather than copying the file.

#### 1.2 Team Information Environment Setup
According to the competition protocol, the participant application must present team ID, authentication token, and supported Stage information to the Meta-Sejong Platform. This information must be set as environment variables.

```yaml
environment:
  - ...
  - ENV_METASEJONG_TEAM_NAME=team_passion_for_challenges
  - ENV_METASEJONG_TEAM_TOKEN=87cef2059293b764451516c5e632e8b5
  - ENV_METASEJONG_TEAM_TARGET_STAGE=2
```

**Note**: When submitting competition results, you must modify this information with the values issued during the application process.

### 2. Demo Application Operation Flow

![Demo Application Operation Flow](./_static/images/task_and_evaluation_protocol.jpg)

The above diagram shows the complete operation flow of the demo application. The main steps are as follows:

1. **Participant Application Start Request**
   - Send COMPETITOR_APP_STARTED request
   - Include team ID, authentication token, supported stage
   - Stage 1: Object detection and pose estimation
   - Stage 2: Object collection and classification

2. **Participant Work Preparation Phase**
   - Platform verifies team ID and authentication token
   - Competition environment configuration
   - Virtual environment data streaming begins
   - Data provision through ROS Topics begins

3. **Start Response Reception**
   - Receive COMPETITOR_APP_STARTED_RESPONSE
   - Confirm all preparations complete
   - Begin actual competition work

4. **Stage 1 Work**
   - Fixed camera image analysis
   - Object detection and pose estimation
   - Trash object location and orientation determination

5. **Stage 1 Result Report and Evaluation**
   - Send REPORT_STAGE1_RESULT request
   - Platform responds with evaluation results
   - Analysis data format:
   ```json
   {
       "msg": 102,
       "session": <session id>,
       "payload": {
           "object_detections": [
               {"class_name": "master_shelf_can", "position": [x, y, z]},
               ... 
           ]
       }
   }
   ```
   - Score Calculation:
      Stage 1 score is calculated by comparing the ground truth stored in the Meta-Sejong Platform with the 'object_detections' field in the participant's Stage 1 result submission. The evaluation assesses the accuracy of the detected object's type and position. The final score is determined by the number of correctly recognized objects.

6. **Stage 2 Work**
   - Actual work execution using robot
   - Robot movement and arm control
   - Trash collection and classification

7. **Robot Control Interface**
   - Robot control command
      - `/metasejong2025/robot/cmd_vel`: Robot movement command
      - `/metasejong2025/robot/ppcmd`: Robot arm control command

   - Robot position information
      - `/metasejong2025/robot/odom`: Robot position(odometry) information
      - `/tf`: Coordinate transformation information

   - Map information
      - `/metasejong2025/map`: Global occupancy map

8. **Stage 2 Work Evaluation**
   - Real-time work execution evaluation
   - Collection success and classification success evaluation
   - Score calculation:
   ![Stage 2 Score Calculation Formula](./_static/images/math_formation_stage2_score.jpg)

For detailed implementation methods and API usage for each step, please refer to the [Technical Guide](technical-guide.md).

## Next Steps

- Read the [Technical Guide](technical-guide.md): You can check detailed technical information.
- Check the [Submission Guide](submit-guide.md): You can find guidance on how to submit your results.