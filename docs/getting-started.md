# Getting Started Guide

## Introduction

Welcome to the META-SEJONG AI Robotics Challenge 2025! This event is a sub-event of IEEE MetaCom 2025 and corresponds to the MetaCom 2025 Student Challenge Programs.

The META-Sejong AI Robotics Challenge provides a metaverse space modeled after Sejong University, where challenging students can form teams and fully utilize Embodied AI technologies.

Follow the guide below to set up your development environment and complete the given missions by interacting with the virtual environment through the ROS2 standard interface.

## Prerequisites

Before you begin, make sure you have:

### MetaSejong (Metaverse Service Platform)
- Python 3.10 or higher installed
- Git installed
- Basic understanding of Python programming
- Familiarity with ROS2
- Familiarity with NVidia IsaacSim 4.0.2
- Docker and Docker Compose

### Development Environment for Competition Participants
- Python 3.10 or higher installed
- Git installed
- Basic understanding of Python programming
- Familiarity with ROS2
- Docker and Docker Compose

## Installation

### Meta-Sejong Platform

The Meta-Sejong platform is a metaverse virtual environment of Sejong University built using NVidia IsaacSim. It also provides APIs necessary for participants to perform their missions in the competition. Detailed information about the provided APIs will be explained on a separate page.

The Meta-Sejong platform serves as a counterpart for training the AI Robotics applications developed by participants.

Follow the steps below to run IsaacSim based on Docker and check the metaverse-implemented Sejong University. This project provides an environment that can be run through Docker Compose.

1. Clone the competition repository:
```bash
git clone https://github.com/metasejong-competition/metacom2025-metasejong
cd metacom2025-metasejong
```

This repository provides a kind of environment for developing competition applications. There is no need to fork it.

2. Load Docker image
The files downloaded via git consist of a Docker image tar file, docker-compose.yml, and Makefile.

You can perform this using Docker commands directly or through the commands provided in the Makefile.

2.1. Using Docker commands:

```
cd <git clone folder>
docker load -i <image_name>.tar
```

2.2. Using Makefile commands:
```
make load-image
```

3. Run

3.1. Using Docker commands:

Using Docker Compose:
```bash
docker compose up
```

3.2. Using Makefile:
```bash
make run
```

### Demo Application for Competition Participants

The demo application for competition participants provides example code for utilizing data provided by the metasejong platform and controlling robots operating on the platform.

The application is in the form of a ROS2 application package and includes message exchange between the participant's application and the metasejong platform according to the competition procedure, so participants should carefully analyze it and follow the required procedures.

Competition participants must follow the following procedures:

1. Application

|Participant|   |Organizer|
|---|---|---|
|Submit application form| --> | |
| | <-- | Issue team ID and authentication token (email)|

2. Check the demo application for participants
