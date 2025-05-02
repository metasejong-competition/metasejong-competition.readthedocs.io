# 기술 가이드

## 개요

본 가이드는 MARC (Meta-Sejong AI Robotics Challenge) 2025의 개발 환경 구축과 데모 애플리케이션에 대한 상세한 기술 정보를 제공합니다.

## 개발 환경 구축

### 1. 시스템 요구사항

#### 1.1 하드웨어 요구사항
- **CPU**: Intel Core i5 또는 AMD Ryzen 5 이상
- **RAM**: 8GB 이상
- **GPU**: NVIDIA GPU (선택 사항, AI 알고리즘 구현 시 권장)
- **저장공간**: 20GB 이상의 여유 공간
- **네트워크**: 100Mbps 이상의 네트워크 연결

#### 1.2 소프트웨어 요구사항
- **운영체제**: Ubuntu 22.04 LTS
- **Python**: 3.10 이상
- **ROS2**: Humble Hawksbill
- **Docker**: 20.10 이상
- **Docker Compose**: 2.0 이상

### 2. 개발 도구 설치

#### 2.1 필수 개발 도구
- **Git**: 버전 관리 및 협업
  ```bash
  sudo apt update
  sudo apt install git
  ```
- **Docker**: 컨테이너 기반 개발 환경 관리
  ```bash
  # Docker 설치
  sudo apt install docker.io
  sudo systemctl enable docker
  sudo systemctl start docker
  
  # Docker Compose 설치
  sudo apt install docker-compose
  ```
- **ROS2**: 로봇 제어 시스템 개발 

[선택적] 제공된 Dockerfile과 docker-compose.yml 파일을 이용하여 ROS2 설치 없이 docker 환경으로 개발 가능
  
  ```bash
  # ROS2 Humble 설치
  sudo apt install software-properties-common
  sudo add-apt-repository universe
  sudo apt update && sudo apt install curl
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  sudo apt update
  sudo apt install ros-humble-desktop
  ```

#### 2.2 개발 환경 설정
- **Python 가상환경 설정**
  ```bash
  # Python 가상환경 생성
  python3 -m venv venv
  source venv/bin/activate
  
  # 필수 패키지 설치
  pip install -r requirements.txt
  ```
- **ROS2 워크스페이스 설정**
  ```bash
  # ROS2 워크스페이스 생성
  mkdir -p ~/metasejong_ws/src
  cd ~/metasejong_ws
  colcon build
  ```

## 데모 애플리케이션

### 1. 애플리케이션 구조

```
.
|   # 프로젝트 가이드 문서
├── README.md       
|   # Docker 관련 파일
├── Dockerfile      
├── Dockerfile.dev
├── docker-compose.yaml
├── entrypoint.sh
|   # 개발, 점수 계산 등을 위한 명령어
├── Makefile
|   # 참가자 애플리케이션 작업 공간 (ROS2 Workspace)
└── metasejong_competitor_ws
    └── src
        |   
        └── airobotics_app
            |   # ROS2 패키지 정의 및 모듈 의존성
            ├── package.xml
            ├── requirements.txt
            |   # 참가자 ROS Node 구현
            ├── airobotics_node
            │   ├── __init__.py
            |   |   # ROS Node 실행 진입점
            │   ├── airobotics_node.py
            |   |   # 기본 대회 참가자 애플리케이션 템플릿 예제 구현 (추상 구현)
            │   ├── competition_task_base.py
            |   |   # 기본 대회 참가자 애플리케이션 템플릿 예제 구현
            │   ├── competition_task_implementation.py
            │   ├── competitor_request_message.py
            |   |   # 유틸리티 함수 예제 구현
            │   ├── robot_node.py
            │   ├── robot_util.py
            │   └── world_coordinates_util.py
            ├── resource
            │   ├── airobotics_app
            │   └── metasejong
            ├── setup.cfg
            └── setup.py
```

### 2. 주요 컴포넌트 설명

#### 2.1 ROS2 노드
- **airobotics_node.py**: 메인 실행 파일
  - 대회 애플리케이션의 진입점
  - ROS2 노드 초기화 및 실행
  - 메시지 발행/구독 설정

- **competition_task_base.py**: 추상 기본 클래스
  - 대회 과제 구현을 위한 기본 인터페이스 정의
  - 필수 메서드 선언
  - 공통 유틸리티 함수 제공

- **competition_task_implementation.py**: 실제 구현 클래스
  - 대회 과제의 구체적인 구현
  - 객체 감지 및 포즈 추정 로직
  - 로봇 제어 로직

#### 2.2 유틸리티 모듈
- **robot_node.py**: 로봇 제어 관련 기능
  - 로봇 이동 제어
  - 로봇팔 제어
  - 센서 데이터 처리

- **robot_util.py**: 로봇 관련 유틸리티 함수
  - 좌표 변환
  - 경로 계획
  - 충돌 회피

- **world_coordinates_util.py**: 월드 좌표계 관련 유틸리티
  - 좌표계 변환
  - 위치 추정
  - 방향 계산

### 3. 개발 가이드

#### 3.1 개발 환경 설정
1. **저장소 복제**
   ```bash
   git clone https://github.com/<your_team_account>/metasejong-airobotics
   cd metasejong-airobotics
   ```

2. **환경 변수 설정**
   ```bash
   export ENV_METASEJONG_TEAM_NAME="your_team_name"
   export ENV_METASEJONG_TEAM_TOKEN="your_team_token"
   export ENV_METASEJONG_TEAM_TARGET_STAGE="your_target_stage"
   ```

3. **Docker 이미지 빌드**
   ```bash
   make build-dev
   ```

#### 3.2 개발 및 테스트

1. **데모 애플리케이션 동작 흐름**

![데모 애플리케이션 동작 흐름](./_static/images/task_and_evaluation_protocol.jpg)

데모 애플리케이션의 동작 흐름은 다음과 같은 단계로 구성됩니다:

##### 1.1 참가자 애플리케이션 시작 요청
- **요청 메시지**: COMPETITOR_APP_STARTED
- **필수 정보**:
  - team ID: 참가 팀 식별자
  - authentication token: 인증 토큰
  - target stage: 지원하는 stage 번호
- **Stage별 목표**:
  - Stage 1: 객체 감지 및 포즈 추정
  - Stage 2: 객체 수집 및 분류

##### 1.2 참가자 작업 준비 단계
- **인증 검증**:
  - team ID와 인증 토큰 유효성 검사
  - 참가 자격 확인
- **환경 구성**:
  - 가상 환경 초기화
  - 로봇 및 센서 설정
- **데이터 스트리밍**:
  - 가상 환경 데이터 전송 시작
  - ROS2 토픽 설정 및 활성화

##### 1.3 시작 응답 수신
- **응답 메시지**: COMPETITOR_APP_STARTED_RESPONSE
- **응답 내용**:
  - 세션 ID
  - 준비 상태
  - 오류 메시지 (있는 경우)
- **다음 단계**:
  - 응답 확인 후 본격적인 작업 시작
  - 오류 발생 시 재시도 또는 문제 해결

##### 1.4 Stage 1 작업
- **이미지 분석**:
  - 고정 카메라 이미지 수신
  - 객체 감지 알고리즘 적용
  - 포즈 추정 수행
- **데이터 처리**:
  - 객체 위치 계산
  - 방향 정보 추출
  - 신뢰도 점수 계산

##### 1.5 Stage 1 결과 보고 및 평가
- **결과 전송**:
  - REPORT_STAGE1_RESULT 요청
  - JSON 형식의 결과 데이터
  ```json
  {
      "msg": 102,
      "session": <session id>,
      "payload": {
          "object_detections": [
              {
                  "class_name": "master_shelf_can",
                  "position": [x, y, z],
                  "orientation": [qx, qy, qz, qw],
                  "confidence": 0.95
              }
          ]
      }
  }
  ```
- **평가 기준**:
  - 객체 감지 정확도
  - 위치 추정 정밀도
  - 방향 추정 정확도

##### 1.6 Stage 2 작업
- **로봇 제어**:
  - 경로 계획 및 이동
  - 로봇팔 제어
  - 그리퍼 조작
- **객체 처리**:
  - 객체 수집
  - 분류 작업
  - 적절한 위치에 배치

##### 1.7 로봇 제어 인터페이스
- **ROS2 토픽**:
  - `/metasejong2025/odom`: 로봇 위치 정보
    - x, y, z 좌표
    - 방향 정보 (쿼터니언)
  - `/metasejong2025/tf`: 좌표 변환 정보
    - 프레임 간 변환 행렬
  - `/metasejong2025/cmd_vel`: 로봇 이동 명령
    - 선속도, 각속도
  - `/metasejong2025/ppcmd`: 로봇팔 제어 명령
    - 관절 각도
    - 그리퍼 상태

##### 1.8 Stage 2 작업 평가
- **평가 항목**:
  - 수집 성공률
  - 분류 정확도
  - 작업 완료 시간
- **점수 계산**:
  ![Stage 2 점수 계산 수식](./_static/images/math_formation_stage2_score.jpg)
  - 기본 점수
  - 보너스 점수
  - 시간 감점

2. **개발 환경 실행**
   ```bash
   make up-dev
   ```

3. **코드 수정 및 테스트**
   - `airobotics_node` 디렉토리 내 파일 수정
   - ROS2 노드 재시작으로 변경사항 적용
   - 로그를 통한 디버깅

### 4. 문제 해결

#### 4.1 일반적인 문제
1. **ROS2 노드 실행 실패**
   - ROS2 환경 설정 확인
   - 필수 패키지 설치 확인
   - 로그 파일 확인

2. **Docker 컨테이너 실행 실패**
   - Docker 서비스 상태 확인
   - 이미지 빌드 오류 확인
   - 컨테이너 로그 확인

3. **성능 문제**
   - 시스템 리소스 사용량 확인
   - GPU 가속 설정 확인
   - 네트워크 대역폭 확인

#### 4.2 디버깅 가이드
1. **로그 확인**
   ```bash
   # ROS2 로그 확인
   ros2 topic echo /rosout
   
   # Docker 로그 확인
   docker logs <container_id>
   ```

2. **성능 모니터링**
   ```bash
   # 시스템 리소스 모니터링
   htop
   
   # GPU 사용량 확인
   nvidia-smi
   ```

3. **네트워크 진단**
   ```bash
   # 네트워크 연결 확인
   ping <target_ip>
   
   # 포트 확인
   netstat -tulpn
   ``` 