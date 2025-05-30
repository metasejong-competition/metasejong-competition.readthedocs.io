# API Reference

## Introduction

이 문서는 Meta-Sejong 플랫폼에서 제공하는 ROS2 인터페이스에 대한 상세한 설명을 제공합니다. 참가자는 이 문서를 통해 플랫폼과의 통신 방법을 이해하고 구현할 수 있습니다.

## ROS2 Topic List

다음은 Meta-Sejong 플랫폼에서 제공하는 모든 ROS2 토픽 목록입니다:

| 토픽 이름 | 메시지 타입 | 발행/구독 | 설명 |
|----------|------------|----------|------|
| /metasejong2025/camera/image_raw | sensor_msgs/Image | 발행 | RGB 카메라 이미지 |
| /metasejong2025/camera/depth | sensor_msgs/Image | 발행 | 깊이 카메라 이미지 |
| /metasejong2025/odom | nav_msgs/Odometry | 발행 | 로봇의 위치 및 자세 정보 |
| /metasejong2025/tf | tf2_msgs/TFMessage | 발행 | 좌표계 변환 정보 |
| /metasejong2025/cmd_vel | geometry_msgs/Twist | 구독 | 로봇 이동 명령 |
| /metasejong2025/arm_control | metasejong_msgs/ArmControl | 구독 | 로봇 팔 제어 명령 |
| /metasejong2025/object_info | metasejong_msgs/ObjectInfo | 발행 | 물체 정보 |
| /metasejong2025/task_status | metasejong_msgs/TaskStatus | 발행 | 작업 상태 정보 |

### 센서 데이터 토픽

#### 카메라 데이터

카메라 토픽은 다음과 같은 데이터 구조를 가집니다:

```json
{
  "header": {
    "stamp": {
      "sec": 1234567890,
      "nanosec": 123456789
    },
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

#### 깊이 카메라 데이터

깊이 카메라 토픽은 다음과 같은 데이터 구조를 가집니다:

```json
{
  "header": {
    "stamp": {
      "sec": 1234567890,
      "nanosec": 123456789
    },
    "frame_id": "depth_camera_link"
  },
  "height": 480,
  "width": 640,
  "encoding": "32FC1",
  "is_bigendian": 0,
  "step": 2560,
  "data": [/* 깊이 데이터 */]
}
```

### 로봇 상태 토픽

#### 오도메트리 데이터

오도메트리 토픽은 다음과 같은 데이터 구조를 가집니다:

```json
{
  "header": {
    "stamp": {
      "sec": 1234567890,
      "nanosec": 123456789
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
    "covariance": [/* 공분산 행렬 */]
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
    "covariance": [/* 공분산 행렬 */]
  }
}
```

### 제어 명령 토픽

#### 로봇 이동 명령

로봇 이동 명령 토픽은 다음과 같은 데이터 구조를 가집니다:

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

#### 로봇 팔 제어 명령

로봇 팔 제어 명령 토픽은 다음과 같은 데이터 구조를 가집니다:

```json
{
  "command": "grasp",
  "target": {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0
  },
  "parameters": {
    "force": 10.0,
    "speed": 0.5
  }
}
```

## 경진대회 프로토콜

### 참가자 요청 메시지

참가자 요청 메시지는 다음과 같은 기본 구조를 가집니다:

```json
{
  "msg": "COMPETITOR_APP_STARTED",
  "session": "unique_session_id",
  "payload": {
    "team_name": "팀 이름",
    "version": "1.0.0"
  }
}
```

### 플랫폼 응답 메시지

플랫폼 응답 메시지는 다음과 같은 기본 구조를 가집니다:

```json
{
  "msg": "COMPETITOR_APP_STARTED_RESPONSE",
  "status": "success",
  "status_message": "성공적으로 처리되었습니다.",
  "result": {
    "session": "unique_session_id",
    "stage": 1,
    "time_limit": 300
  }
}
```

### 플랫폼 통지 메시지

플랫폼 통지 메시지는 다음과 같은 기본 구조를 가집니다:

```json
{
  "msg": "TIME_CONSTRAINT_EXPIRED",
  "session": "unique_session_id",
  "payload": {
    "current_stage": 1,
    "elapsed_time": 300,
    "remaining_tasks": 2
  }
}
```

## 사용 예시

다음은 기본적인 ROS2 노드 구현 예시입니다:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class CompetitorNode(Node):
    def __init__(self):
        super().__init__('competitor_node')
        
        # 구독자 생성
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
            
        # 발행자 생성
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/metasejong2025/cmd_vel',
            10)
            
    def image_callback(self, msg):
        # 이미지 처리 로직
        pass
        
    def depth_callback(self, msg):
        # 깊이 데이터 처리 로직
        pass
        
    def move_robot(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

def main():
    rclpy.init()
    node = CompetitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 주의사항

1. **메시지 주기**
   - 센서 데이터는 30Hz로 발행됩니다.
   - 제어 명령은 최대 10Hz로 발행할 수 있습니다.

2. **오류 처리**
   - 모든 메시지에는 타임스탬프가 포함되어 있습니다.
   - 메시지 손실이나 지연이 발생할 수 있으므로 적절한 오류 처리가 필요합니다.

3. **성능 고려사항**
   - 이미지 처리 시 메모리 사용량을 고려해야 합니다.
   - 제어 명령은 적절한 주기로 발행해야 합니다.

4. **디버깅**
   - ROS2 명령어를 사용하여 토픽 모니터링이 가능합니다.
   - rqt 도구를 사용하여 시각적 디버깅이 가능합니다.
