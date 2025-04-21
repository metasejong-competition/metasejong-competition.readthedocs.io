# API 참조 문서

## 개요

본 문서는 MetaSejong AI Robotics Challenge 2025에서 사용되는 ROS2 인터페이스에 대한 상세한 설명을 제공합니다. 대회 참가자들은 이 문서를 통해 플랫폼과의 통신 방법을 이해하고, 필요한 기능을 구현할 수 있습니다.

## ROS2 토픽 목록

다음은 MetaSejong 플랫폼에서 제공하는 모든 ROS2 토픽의 목록입니다:

| 토픽 이름 | 메시지 타입 | 발행 → 구독 | 설명 |
|----------|------------|----------|------|
| **경진대회 프로토콜** |  |  | 
| `/metasejong2025/competitor_request` | std_msgs/msg/String (JSON) | 참가자 → 플랫폼 | 참가자가 플랫폼에 전송하는 요청 메시지 |
| `/metasejong2025/competitor_response` | std_msgs/msg/String (JSON) | 플랫폼 → 참가자 | 플랫폼이 참가자의 요청에 대해 전송하는 응답 메시지 |
| `/metasejong2025/competitor_notification` | std_msgs/msg/String (JSON) | 플랫폼 → 참가자 | 플랫폼이 참가자에게 전송하는 통지 메시지 |
|  **센서 데이터** |  |  | 
| `/metasejong2025/cameras/<field_name>/image_raw` | sensor_msgs/Image | 플랫폼 → 참가자 | 고정 카메라에서 촬영한 RGB 이미지 |
| `/metasejong2025/cameras/<field_name>/camera_info` | sensor_msgs/msg/CameraInfo | 플랫폼 → 참가자 | 고정 카메라의 내부/외부 파라미터 정보 |
| `/metasejong2025/robot_camera/camera_info` | sensor_msgs/msg/CameraInfo | 플랫폼 → 참가자 | 로봇 카메라의 내부/외부 파라미터 정보 |
| `/metasejong2025/robot_camera/color` | sensor_msgs/msg/Image | 플랫폼 → 참가자 | 로봇 카메라에서 촬영한 RGB 이미지 |
| `/metasejong2025/robot_camera/depth` | sensor_msgs/msg/Image | 플랫폼 → 참가자 | 로봇 카메라에서 촬영한 깊이 이미지 |
| `/metasejong2025/scan` | sensor_msgs/msg/LaserScan | 플랫폼 → 참가자 | 라이다 센서에서 측정한 스캔 데이터 |
|  **로봇 상태, 네비게이션 및 제어** |  |  | 
| `/metasejong2025/odom` | nav_msgs/Odometry | 플랫폼 → 참가자 | 로봇의 현재 위치 및 방향 정보 |
| `/metasejong2025/tf` | tf2_msgs/TFMessage | 플랫폼 → 참가자 | 좌표계 간 변환 정보 |
| `/metasejong2025/cmd_vel` | geometry_msgs/Twist | 참가자 → 플랫폼 | 로봇의 이동 속도 제어 명령 |
| `/metasejong2025/ppcmd` | std_msgs/msg/String | 참가자 → 플랫폼 | 로봇팔의 동작 제어 명령 |

## 경진대회 프로토콜

### 1. 참가자 요청 메시지
- **토픽**: `/metasejong2025/competitor_request`
- **타입**: std_msgs/msg/String (JSON)
- **설명**: 참가자가 플랫폼에 전송하는 요청 메시지
- **기본 데이터 구조**:
  ```json
  {
      "msg": <message type>,
      "session": <null or session string>,
      "payload": {
          <payload contents>
      }
  }
  ```

- **메시지 타입 및 payload 포맷**:
  - **COMPETITOR_APP_STARTED (101)**: 참가자 애플리케이션 시작 요청
    - **설명**: 참가자 애플리케이션이 실행되어 stage task를 수행할 준비가 되었음을 플랫폼에 알리는 요청 메시지
    - **주의사항**: 
      - session 값은 무시됨 (session key 발급 전 호출)
      - team과 token은 경진대회 참가 신청서 제출 시 받은 값을 사용
      - stage는 참가자 어플리케이션이 stage 1까지 수행할지 stage 2까지 수행할지 선택값
    - **데이터 구조**:
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

  - **REPORT_STAGE1_COMPLETED (102)**: Stage 1 완료 보고 요청
    - **설명**: 참가자 애플리케이션이 Stage 1 임무를 완료하고 결과를 플랫폼에 보고하는 요청 메시지
    - **주의사항**: 
      - session key는 COMPETITOR_APP_STARTED 응답에서 받은 값을 사용
    - **데이터 구조**:
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

  - **REPORT_STAGE2_COMPLETED (103)**: Stage 2 완료 보고 요청
    - **설명**: 참가자 애플리케이션이 Stage 2 임무를 완료하고 결과를 플랫폼에 보고하는 요청 메시지
    - **주의사항**: 
      - session key는 COMPETITOR_APP_STARTED 응답에서 받은 값을 사용
      - payload는 무시됨
    - **데이터 구조**:
      ```json
      {
          "msg": 103,
          "session": "<session key>",
          "payload": {
          }
      }
      ```

### 2. 플랫폼 응답 메시지
- **토픽**: `/metasejong2025/competitor_response`
- **타입**: std_msgs/msg/String (JSON)
- **설명**: 참가자의 요청에 대한 플랫폼의 응답 메시지
- **기본 데이터 구조**:
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
  - **msg**: 10X 형식의 요청 메시지에 대해 20X 형식의 응답 메시지 유형
  - **status**: 1(success) 또는 0(failed)
  - **status_message**: status가 1일 경우 "OK", 0일 경우 오류 원인
  - **result**: 메시지 유형에 따른 응답 내용

- **메시지 타입 및 payload 포맷**:
  - **COMPETITOR_APP_STARTED_RESPONSE (201)**: 참가자 애플리케이션 시작 응답
    - **설명**: COMPETITOR_APP_STARTED(101) 요청에 대한 응답
    - **주의사항**: 
      - team 인증 성공 시 session key 발급
      - 인증 실패 시 status가 0으로 설정되고 오류 원인이 status_message에 포함
    - **데이터 구조**:
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

  - **REPORT_STAGE1_COMPLETED_RESPONSE (202)**: Stage 1 완료 보고 응답
    - **설명**: REPORT_STAGE1_COMPLETED(102) 요청에 대한 응답
    - **주의사항**: 
      - 응답 수신 후 ENV_METASEJONG_TEAM_TARGET_LEVEL 값에 따라 다음 동작 결정
      - 처리 오류 시 status가 0으로 설정되고 오류 원인이 status_message에 포함
    - **데이터 구조**:
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

  - **REPORT_STAGE2_COMPLETED_RESPONSE (203)**: Stage 2 완료 보고 응답
    - **설명**: REPORT_STAGE2_COMPLETED(103) 요청에 대한 응답
    - **주의사항**: 
      - 응답 수신 후 애플리케이션 종료 가능
      - 처리 오류 시 status가 0으로 설정되고 오류 원인이 status_message에 포함
    - **데이터 구조**:
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

### 3. 플랫폼 통지 메시지
- **토픽**: `/metasejong2025/competitor_notification`
- **타입**: std_msgs/msg/String (JSON)
- **설명**: 플랫폼이 참가자에게 전송하는 통지 메시지
- **기본 데이터 구조**:
  ```json
  {
      "msg": <message type>,
      "session": "<session key>",
      "payload": {
          <payload contents>
      }
  }
  ```

- **메시지 타입 및 payload 포맷**:
  - **TIME_CONSTRAINT_EXPIRED (301)**: 제한 시간 만료 통지
    - **설명**: 경연 시나리오의 제한 시간이 만료되었음을 알리는 통지 메시지
    - **주의사항**: 
      - 메시지 수신 후 즉시 애플리케이션 종료 필요
      - 플랫폼은 제한 시간 만료 후 요청 메시지 수신 중단
    - **데이터 구조**:
      ```json
      {
          "msg": 301,
          "session": "<session key>",
          "payload": {
              "dummy": ""
          }
      }
      ```

  - **COMPETITOR_REQUEST_ERROR (302)**: 참가자 요청 오류 통지
    - **설명**: 플랫폼 동작 중 발생한 오류를 참가자에게 알리는 통지 메시지
    - **데이터 구조**:
      ```json
      {
          "msg": 302,
          "session": "<session key>",
          "payload": {
              "error_message": "오류 메시지"
          }
      }
      ```

## 센서 데이터

### 1. 고정 카메라 데이터
- **토픽**: `/metasejong2025/cameras/<field_name>/image_raw`
- **타입**: sensor_msgs/Image
- **설명**: 고정 카메라에서 촬영한 RGB 이미지
- **데이터 구조**:
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
      "data": [/* 이미지 데이터 */]
  }
  ```

### 2. 카메라 정보
- **토픽**: `/metasejong2025/cameras/<field_name>/camera_info`
- **타입**: sensor_msgs/msg/CameraInfo
- **설명**: 카메라의 내부/외부 파라미터 정보
- **데이터 구조**:
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

### 3. 로봇 카메라 데이터
- **토픽**: `/metasejong2025/robot_camera/color`
- **타입**: sensor_msgs/msg/Image
- **설명**: 로봇 카메라에서 촬영한 RGB 이미지
- **데이터 구조**: 고정 카메라 데이터와 동일

### 4. 로봇 카메라 깊이 데이터
- **토픽**: `/metasejong2025/robot_camera/depth`
- **타입**: sensor_msgs/msg/Image
- **설명**: 로봇 카메라에서 촬영한 깊이 이미지
- **데이터 구조**:
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
      "data": [/* 깊이 데이터 */]
  }
  ```

### 5. 라이다 스캔 데이터
- **토픽**: `/metasejong2025/scan`
- **타입**: sensor_msgs/msg/LaserScan
- **설명**: 라이다 센서에서 측정한 스캔 데이터
- **데이터 구조**:
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
      "ranges": [/* 거리 데이터 */],
      "intensities": [/* 강도 데이터 */]
  }
  ```

## 로봇 상태, 네비게이션 및 제어

### 1. 로봇 위치 정보
- **토픽**: `/metasejong2025/odom`
- **타입**: nav_msgs/Odometry
- **설명**: 로봇의 현재 위치 및 방향 정보
- **데이터 구조**:
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

### 2. 좌표계 변환 정보
- **토픽**: `/metasejong2025/tf`
- **타입**: tf2_msgs/TFMessage
- **설명**: 좌표계 간 변환 정보
- **데이터 구조**:
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

### 3. 로봇 이동 명령
- **토픽**: `/metasejong2025/cmd_vel`
- **타입**: geometry_msgs/Twist
- **설명**: 로봇의 이동 속도 제어 명령
- **데이터 구조**:
  ```json
  {
      "linear": {
          "x": 0.0,  // 전진/후진 속도 (m/s)
          "y": 0.0,  // 좌우 이동 속도 (m/s)
          "z": 0.0   // 상하 이동 속도 (m/s)
      },
      "angular": {
          "x": 0.0,  // 롤 회전 속도 (rad/s)
          "y": 0.0,  // 피치 회전 속도 (rad/s)
          "z": 0.0   // 요 회전 속도 (rad/s)
      }
  }
  ```

### 4. 로봇팔 제어 명령
- **토픽**: `/metasejong2025/ppcmd`
- **타입**: std_msgs/msg/String
- **설명**: 로봇팔의 동작 제어 명령
- **데이터 구조**:
  `<picking_quatenion_angle_for_gripper> <picking_endpoint_for_gripper> <placing_quatenion_angle_for_gripper> <placing_endpoint_for_gripper>`
  
  <picking_quatenion_angle_for_gripper>: 쓰레기를 집기 위한 gripper의 진입 각도, 공백문자로 구분되는 4개의 실수(quatenion)
  <picking_endpoint_for_gripper>: 쓰레기를 집기 위한 gripper의 endpoint 위치, 공백문자로 구분되는 3개의 실수(x, y, z)
  <placing_quatenion_angle_for_gripper>: 쓰레기를 수거하기 위한 gripper의 진입 각도, 공백문자로 구분되는 4개의 실수(quatenion)
  <placing_endpoint_for_gripper>: 쓰레기를 수거하기 위한 gripper의 endpoint 위치, 공백문자로 구분되는 3개의 실수(x, y, z)
