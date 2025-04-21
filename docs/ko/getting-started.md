# 시작하기 가이드

## 소개

MetaSejong AI Robotics Challenge 2025에 오신 것을 환영합니다! 본 대회는 IEEE MetaCom 2025의 부대행사이며, MetaCom 2025 Student Challenge Programs의 일환으로 진행됩니다. 메타버스와 AI 로보틱스 기술을 융합한 혁신적인 도전의 장을 제공합니다.

MetaSejong AI Robotics Challenge는 세종대학교를 모델로 한 메타버스 공간에서 진행됩니다. 참가 팀들은 이 가상 환경에서 Embodied AI 기술을 활용하여 주어진 과제를 해결하게 됩니다. 참가자들은 가상의 세종대학교 캠퍼스에서 로봇을 제어하고, 다양한 AI 기술을 적용하여 미션을 수행하게 됩니다.

본 가이드는 개발 환경 구성부터 미션 수행까지의 전체 과정을 안내합니다. ROS2 표준 인터페이스를 통해 가상 환경과 상호작용하는 방법을 단계별로 상세히 설명하여, 참가자들이 원활하게 대회를 준비할 수 있도록 도와드립니다.

## 시스템 아키텍처

![대회 개발 환경](./_static/images/interface_architecture.jpg)

대회 시스템은 다음과 같은 주요 구성 요소로 이루어져 있습니다:

1. **MetaSejong 플랫폼**
MetaSejong 플랫폼은 NVIDIA IsaacSim을 기반으로 구축된 세종대학교의 메타버스 가상 환경입니다. 다음과 같은 주요 기능을 제공합니다:

    - 실시간 3D 가상 환경 시뮬레이션
    - 물리 엔진 기반의 정밀한 로봇 동작 시뮬레이션
    - 다양한 센서 데이터 스트리밍
    - 실시간 로봇 제어 인터페이스

2. **참가자 개발 환경**
    - ROS2 기반의 애플리케이션 개발 환경
    - AI 알고리즘 구현 및 테스트 환경
    - Docker 컨테이너 기반의 독립적인 개발 공간

3. **통신 인터페이스**
대회 참가자들이 미션을 수행하는 데 필요한 API는 ROS2 인터페이스를 통해 제공됩니다. API에 대한 자세한 설명은 별도의 문서에서 확인할 수 있으며, 다음과 같은 주요 기능을 포함합니다:

    - 로봇 제어 인터페이스
    - 센서 데이터 수집
    - 환경 정보 접근
    - 미션 진행 상태 모니터링

MetaSejong 플랫폼은 참가자들이 개발하는 AI Robotics 애플리케이션의 학습 환경으로 활용됩니다. 이를 통해 실제 로봇을 사용하지 않고도 다양한 시나리오를 테스트하고 최적화할 수 있습니다.

Docker를 사용하여 IsaacSim을 실행하고 메타버스로 구현된 세종대학교를 탐험해보세요. 이 프로젝트는 Docker Compose를 통해 실행할 수 있는 환경을 제공합니다. 다음 단계를 따라 설치를 진행하세요:

## 필수 요구사항

MetaSejong AI Robotics Challenge 2025에 참가하기 위해서는 다음과 같은 요구사항을 충족해야 합니다. 각 요구사항은 대회 참가와 개발에 필수적인 요소들입니다:

### 1. 하드웨어 요구사항

#### MetaSejong 플랫폼 실행 환경
- **CPU**: Intel Core i7-8700K 또는 AMD Ryzen 7 3700X 이상
- **RAM**: 32GB 이상
- **GPU**: NVIDIA RTX 3080 또는 NVIDIA RTX A5000 이상
  - CUDA 11.7 이상 지원
  - 최소 8GB VRAM
- **저장공간**: SSD 100GB 이상의 여유 공간
- **네트워크**: 1Gbps 이상의 네트워크 연결

#### 참가자 개발 환경
- **CPU**: Intel Core i5 또는 AMD Ryzen 5 이상
- **RAM**: 8GB 이상
- **GPU**: NVIDIA GPU (선택 사항, AI 알고리즘 구현 시 권장)
- **저장공간**: 20GB 이상의 여유 공간
- **네트워크**: 100Mbps 이상의 네트워크 연결

### 2. 소프트웨어 요구사항

#### MetaSejong 플랫폼
- **운영체제**: Ubuntu 22.04 LTS
- **Python**: 3.10 이상
- **Docker**: 20.10 이상
- **Docker Compose**: 2.0 이상
- **NVIDIA 드라이버**: 525.60.13 이상
- **CUDA**: 11.7 이상

#### 참가자 개발 환경
- **운영체제**: Ubuntu 22.04 LTS
- **Python**: 3.10 이상
- **ROS2**: Humble Hawksbill
- **Docker**: 20.10 이상
- **Docker Compose**: 2.0 이상

### 3. 개발 도구 및 지식 요구사항

#### 필수 개발 도구
- **Git**: 버전 관리 및 협업
- **Docker**: 컨테이너 기반 개발 환경 관리
- **ROS2**: 로봇 제어 시스템 개발
- **Python IDE**: 코드 작성 및 디버깅

#### 필수 지식
- **Python 프로그래밍**: 알고리즘 구현 및 디버깅
- **ROS2 기본 개념**: 토픽, 서비스, 액션 등
- **Docker 기본 사용법**: 이미지 빌드 및 컨테이너 실행
- **기본적인 AI/ML 지식**: 객체 감지, 포즈 추정 등

### 4. 대회 참가 요구사항

#### 필수 정보
- **팀 정보**: 팀명, 팀원 정보
- **참가 신청서**: 대회 주최측에서 제공하는 양식 작성
- **팀 ID**: 참가 신청 승인 후 발급
- **인증 토큰**: 참가 신청 승인 후 발급

#### 제출 요구사항
- **GitHub 저장소**: 비공개 저장소 권장
- **코드 제출**: main/master 브랜치에 최종 코드
- **문서화**: HOWTORUN.md 파일 포함
- **실행 방법**: make up-prod 명령으로 실행 가능

## 설치 방법

### 1. MetaSejong 플랫폼 설치

#### 1.1 시스템 사전 요구사항 설정

MetaSejong 플랫폼은 Docker로 배포되는 IsaacSim 시뮬레이션 응용프로그램으로, GUI를 포함하고 있습니다. GUI 애플리케이션을 Docker에서 실행하기 위해서는 X11 설정이 필요합니다.

1. **X11 설정 확인**
   ```bash
   # X11 설정 확인
   echo $DISPLAY
   ```
   - 출력이 `:1` 또는 `localhost:1` 형식이어야 합니다.
   - X11 설정이 되어있지 않은 경우, [Ubuntu X Architecture](https://wiki.ubuntu.com/X/Architecture)를 참고하여 설정하세요.

#### 1.2 대회 저장소 복제

1. **저장소 복제**
   ```bash
   # MetaSejong 플랫폼 저장소 복제
   git clone https://github.com/metasejong-competition/metacom2025-metasejong
   cd metacom2025-metasejong
   ```

2. **저장소 구성**
   - `docker-compose.yml`: 환경 구성 설정 파일
   - `Makefile`: 빌드 및 실행 명령어
   - 예제 코드 및 문서

#### 1.3 Docker 이미지 다운로드

1. **이미지 다운로드**
   ```bash
   # Makefile을 통한 다운로드
   make download
   ```
   - 또는 직접 다운로드: [metasejong-metacom2025-r02.tar](https://drive.google.com/file/d/10r-tzDj0qS6OKWEle0gl4GnRtEitVKD5/view?usp=sharing)
   - 다운로드 받은 파일명은 변경하지 마세요.

2. **Docker 이미지 로드**
   ```bash
   # 다운로드 받은 이미지 로드
   make load
   ```

#### 1.4 플랫폼 실행

1. **X11 활성화**
   ```bash
   # X11 활성화 (한 번만 실행)
   make setup
   ```

2. **플랫폼 실행**
   ```bash
   # MetaSejong 플랫폼 실행
   make run
   ```
   - 실행 시 성능 제한이 있을 수 있으니 참고하세요.

### 2. 참가자 개발 환경 설정

#### 2.1 사전 준비

1. **참가 신청**
   - [신청서 제출](https://metasejong-competition.github.io/)에서 참가 신청
   - 신청서 검토 후 팀 ID와 인증 토큰 발급
   - 이메일로 확인 메시지와 함께 팀 정보 수신

#### 2.2 개발 환경 저장소 설정

1. **저장소 Fork**
   - [개발환경 리포지토리](https://github.com/metasejong-competition/metasejong-airobotics)를 Fork
   - 비공개 저장소로 설정 권장

2. **저장소 복제**
   ```bash
   # 개발 환경 저장소 복제
   git clone https://github.com/<your_team_account>/metasejong-airobotics
   cd metasejong-airobotics
   ```

#### 2.3 환경 변수 설정

1. **필수 환경 변수 설정**
   ```bash
   # 환경 변수 설정
   export ENV_METASEJONG_TEAM_NAME="your_team_name"
   export ENV_METASEJONG_TEAM_TOKEN="your_team_token"
   export ENV_METASEJONG_TEAM_TARGET_STAGE="your_target_stage"

   # 영구적인 환경 변수 설정 (선택)
   echo "export ENV_METASEJONG_TEAM_NAME=\"your_team_name\"" >> ~/.bashrc
   ```

2. **데모용 환경 변수 값**
   |환경변수|값|
   |---|---|
   |ENV_METASEJONG_TEAM_NAME|team_passion_for_challenges|
   |ENV_METASEJONG_TEAM_TOKEN|87cef2059293b764451516c5e632e8b5|
   |ENV_METASEJONG_TEAM_TARGET_STAGE|2|

#### 2.4 빌드 및 실행

1. **빌드 명령어**
   ```bash
   # 개발용 이미지 빌드
   make build-dev
   
   # 배포용 이미지 빌드
   make build-prod
   ```

2. **실행 명령어**
   ```bash
   # 개발 환경 실행
   make up-dev
   
   # 배포 환경 실행
   make up-prod
   
   # 컨테이너 중지
   make down
   
   # Docker 리소스 정리
   make clean
   ```

### 3. 결과물 제출

#### 3.1 제출 준비

1. **코드 정리**
   - main/master 브랜치에 최종 코드 push
   - 실행 방법 문서화 (HOWTORUN.md)

2. **저장소 공유**
   - 주최측 GitHub 계정을 collaborator로 등록
   - 비공개 저장소 설정 확인

#### 3.2 제출 요구사항

1. **필수 파일**
   - 수정된 Makefile
   - 수정된 Dockerfile
   - 수정된 docker-compose.yml
   - HOWTORUN.md (필요한 경우)

2. **실행 방법**
   - `make up-prod` 명령으로 실행 가능해야 함
   - 특수한 설정이 필요한 경우 상세히 문서화

## 데모 애플리케이션

### 1. 데모 애플리케이션 구조

데모 애플리케이션은 다음과 같은 구조로 제공됩니다:

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

### 2. 데모 애플리케이션 동작 흐름

![workflow](./_static/images/task_and_evaluation_protocol.jpg)

데모 애플리케이션의 동작 흐름은 다음과 같습니다:

1. **참가자 애플리케이션 시작 요청**
   - COMPETITOR_APP_STARTED 요청 전송
   - team ID, authentication token, 지원 stage 포함
   - stage 1: 객체 감지 및 포즈 추정
   - stage 2: 객체 수집 및 분류

2. **참가자 작업 준비 단계**
   - 플랫폼에서 team ID와 인증 토큰 검증
   - 대회 환경 구성
   - 가상 환경 데이터 스트리밍 시작
   - ROS Topic으로 데이터 제공 시작

3. **시작 응답 수신**
   - COMPETITOR_APP_STARTED_RESPONSE 응답 수신
   - 모든 준비 완료 확인
   - 본격적인 대회 작업 시작

4. **Stage 1 작업**
   - 고정 카메라 이미지 분석
   - 객체 감지 및 포즈 추정
   - 쓰레기 물체 위치 및 방향 파악

5. **Stage 1 결과 보고 및 평가**
   - REPORT_STAGE1_RESULT 요청 전송
   - 플랫폼에서 평가 결과 회신
   - 분석 데이터 포맷:
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

6. **Stage 2 작업**
   - 로봇을 이용한 실제 작업 수행
   - 로봇 이동 및 로봇팔 제어
   - 쓰레기 수집 및 분류

7. **로봇 제어 인터페이스**
   - `/metasejong2025/odom`: 로봇 위치 정보
   - `/metasejong2025/tf`: 좌표 변환 정보
   - `/metasejong2025/cmd_vel`: 로봇 이동 명령
   - `/metasejong2025/ppcmd`: 로봇팔 제어 명령

8. **Stage 2 작업 평가**
   - 실시간 작업 수행 평가
   - 수집 성공 여부 및 분류 성공 여부 평가
   - 최종 점수 계산:
   ![Stage 2 점수 계산 수식](./_static/images/math_formation_stage2_score.jpg)

## 다음 단계

- [기술 가이드](technical-guide.md)를 읽어보세요: 상세한 기술 정보를 확인할 수 있습니다.
- [대회 규칙](rules.md)을 검토하세요: 대회 참가를 위한 필수 규칙을 확인할 수 있습니다.
- [제출 가이드](submit-guide.md)를 확인하세요: 결과물 제출 방법에 대한 안내를 확인할 수 있습니다. 