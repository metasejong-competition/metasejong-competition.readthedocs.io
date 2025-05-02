# 시작하기 가이드

## 소개

MARC (Meta-Sejong AI Robotics Challenge) 2025에 오신 것을 환영합니다! 본 대회는 IEEE MetaCom 2025의 부대행사이며, MetaCom 2025 Student Challenge Programs의 일환으로 진행됩니다. 메타버스와 AI 로보틱스 기술을 융합한 혁신적인 도전의 장을 제공합니다.

MARC (Meta-Sejong AI Robotics Challenge) 2025는 세종대학교를 모델로 한 메타버스 공간에서 진행됩니다. 참가 팀들은 이 가상 환경에서 Embodied AI 기술을 활용하여 주어진 과제를 해결하게 됩니다. 참가자들은 가상의 세종대학교 캠퍼스에서 로봇을 제어하고, 다양한 AI 기술을 적용하여 미션을 수행하게 됩니다.

본 가이드는 개발 환경 구성부터 미션 수행까지의 전체 과정을 안내합니다. ROS2 표준 인터페이스를 통해 가상 환경과 상호작용하는 방법을 단계별로 상세히 설명하여, 참가자들이 원활하게 대회를 준비할 수 있도록 도와드립니다.

## 시스템 아키텍처

![대회 개발 환경](./_static/images/interface_architecture.jpg)

대회 시스템은 다음과 같은 주요 구성 요소로 이루어져 있습니다:

1. **Meta-Sejong 플랫폼**
Meta-Sejong 플랫폼은 NVIDIA IsaacSim을 기반으로 구축된 세종대학교의 메타버스 가상 환경입니다. 다음과 같은 주요 기능을 제공합니다:

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

Meta-Sejong 플랫폼은 참가자들이 개발하는 AI Robotics 애플리케이션의 학습 환경으로 활용됩니다. 이를 통해 실제 로봇을 사용하지 않고도 다양한 시나리오를 테스트하고 최적화할 수 있습니다.

Docker를 사용하여 IsaacSim을 실행하고 메타버스로 구현된 세종대학교를 탐험해보세요. 이 프로젝트는 Docker Compose를 통해 실행할 수 있는 환경을 제공합니다. 다음 단계를 따라 설치를 진행하세요:

## 설치 방법

### 1. Meta-Sejong 플랫폼 설치

#### 1.1 시스템 사전 요구사항 설정

Meta-Sejong 플랫폼은 Docker로 배포되는 IsaacSim 시뮬레이션 응용프로그램으로, GUI를 포함하고 있습니다. GUI 애플리케이션을 Docker에서 실행하기 위해서는 X11 설정이 필요합니다.

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
   # Meta-Sejong 플랫폼 저장소 복제
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
   - 또는 직접 다운로드: [metasejong-metacom2025-with-playground-r05.tar](https://drive.google.com/file/d/1DJ9TKhRjXyTLyUMTSaaSEs4mg_UPDcd4/view?usp=drive_link)
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
   # Meta-Sejong 플랫폼 실행
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
   export ENV_METASEJONG_TEAM_TARGET_LEVEL="your_target_level"

   # 영구적인 환경 변수 설정 (선택)
   echo "export ENV_METASEJONG_TEAM_NAME=\"your_team_name\"" >> ~/.bashrc
   ```

2. **데모용 환경 변수 값**
   |환경변수|값|
   |---|---|
   |ENV_METASEJONG_TEAM_NAME|team_passion_for_challenges|
   |ENV_METASEJONG_TEAM_TOKEN|87cef2059293b764451516c5e632e8b5|
   |ENV_METASEJONG_TEAM_TARGET_LEVEL|2|

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

## 다음 단계

- [기술 가이드](technical-guide.md)를 읽어보세요: 상세한 기술 정보를 확인할 수 있습니다.
- [대회 규칙](rules.md)을 검토하세요: 대회 참가를 위한 필수 규칙을 확인할 수 있습니다.
- [제출 가이드](submit-guide.md)를 확인하세요: 결과물 제출 방법에 대한 안내를 확인할 수 있습니다. 