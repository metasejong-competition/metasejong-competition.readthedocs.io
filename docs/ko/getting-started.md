# 시작하기 가이드

## 소개

META-SEJONG AI Robotics Challenge 2025에 오신 것을 환영합니다! 이 행사는 IEEE MetaCom 2025의 부대행사이며, MetaCom 2025 Student Challenge Programs의 일환입니다. 본 대회는 메타버스와 AI Robotics 기술을 융합한 혁신적인 도전의 장을 제공합니다.

META-Sejong AI Robotics Challenge는 세종대학교를 모델로 한 메타버스 공간을 제공하여, 도전적인 학생들이 팀을 이루어 Embodied AI 기술을 마음껏 활용할 수 있도록 합니다. 참가자들은 가상의 세종대학교 캠퍼스에서 로봇을 제어하고, 다양한 AI 기술을 적용하여 주어진 미션을 해결하게 됩니다.

이 가이드를 따라 개발 환경을 구성하고, ROS2 표준 인터페이스를 통해 가상환경과 소통하면서 주어진 임무를 완수해보세요. 각 단계별로 상세한 설명과 예제를 제공하여, 참가자들이 쉽게 시작할 수 있도록 도와드립니다.

## 시스템 아키텍처

![대회 개발 환경](./_static/images/interface_architecture.jpg)

대회 시스템은 다음과 같은 주요 컴포넌트로 구성되어 있습니다:

1. **MetaSejong 플랫폼**
Meta-Sejong 플랫폼은 NVidia IsaacSim을 기반으로 구축된 세종대학교의 메타버스 가상환경입니다. 이 플랫폼은 다음과 같은 특징을 가지고 있습니다:

    - 실시간 3D 가상 환경 시뮬레이션
    - 물리 엔진 기반의 정확한 로봇 동작 시뮬레이션
    - 다양한 센서 데이터 스트리밍
    - 실시간 로봇 제어 인터페이스

2. **참가자 개발 환경**
    - ROS2 기반의 애플리케이션 개발 환경
    - AI 알고리즘 구현 및 테스트 환경
    - Docker 컨테이너 기반의 격리된 개발 공간

3. **통신 인터페이스**

대회 참가자들이 임무를 수행하는 데 필요한 API는 ROS2 인터페이스를 통해 제공합니다. 제공되는 API에 대한 자세한 정보는 별도의 페이지에서 설명됩니다. API는 다음과 같은 기능을 포함합니다:

    - 로봇 제어 인터페이스
    - 센서 데이터 수집
    - 환경 정보 접근
    - 미션 진행 상태 모니터링



Meta-Sejong 플랫폼은 참가자들이 개발하는 AI Robotics 애플리케이션의 학습 환경으로 활용됩니다. 이를 통해 실제 로봇을 사용하지 않고도 다양한 시나리오를 테스트하고 최적화할 수 있습니다.

Docker를 사용하여 IsaacSim을 실행하고 메타버스로 구현된 세종대학교를 탐험해보세요. 이 프로젝트는 Docker Compose를 통해 실행할 수 있는 환경을 제공합니다. 다음 단계를 따라 설치를 진행하세요:




## 필수 요구사항

시작하기 전에 다음 항목들이 준비되어 있는지 확인하세요. 각 요구사항은 대회 참가와 개발에 필수적인 요소들입니다:

### MetaSejong Platform 시스템 요구사항

MetaSejong Platform은 NVIDIA IsaacSim을 기반으로 구축되었으므로, IsaacSim의 시스템 요구사항과 동일합니다:

#### 하드웨어 요구사항
- **CPU**: Intel Core i7-8700K 또는 AMD Ryzen 7 3700X 이상
- **RAM**: 32GB 이상
- **GPU**: NVIDIA RTX 3080 또는 NVIDIA RTX A5000 이상
  - CUDA 11.7 이상 지원
  - 최소 8GB VRAM
- **저장공간**: SSD 100GB 이상의 여유 공간
- **네트워크**: 1Gbps 이상의 네트워크 연결

#### 소프트웨어 요구사항
- **운영체제**: Ubuntu 22.04 LTS
- **Python**: 3.10 이상
- **Docker**: 20.10 이상
- **Docker Compose**: 2.0 이상
- **NVIDIA 드라이버**: 525.60.13 이상
- **CUDA**: 11.7 이상

### 대회 참가자 개발 환경 요구사항

대회 참가자 개발 환경은 ROS2 Humble을 기반으로 구축되었으므로, ROS2 Humble의 시스템 요구사항과 동일합니다:

#### 하드웨어 요구사항
- **CPU**: Intel Core i5 또는 AMD Ryzen 5 이상
- **RAM**: 8GB 이상
- **GPU**: NVIDIA GPU (선택 사항, AI 알고리즘 구현 시 권장)
- **저장공간**: 20GB 이상의 여유 공간
- **네트워크**: 100Mbps 이상의 네트워크 연결

#### 소프트웨어 요구사항
- **운영체제**: Ubuntu 22.04 LTS
- **Python**: 3.10 이상
- **ROS2**: Humble Hawksbill
- **Docker**: 20.10 이상
- **Docker Compose**: 2.0 이상

#### 개발 도구 요구사항
- **Git**: 버전 관리 및 협업
- **Python 프로그래밍 기초 지식**: 알고리즘 구현 및 디버깅
- **ROS2 사용 경험**: 로봇 제어 시스템 개발
- **Docker 사용 경험**: 컨테이너 기반 개발 환경 관리

## 설치 방법

### Meta-Sejong 플랫폼

Docker를 사용하여 IsaacSim을 실행하고 메타버스로 구현된 세종대학교를 탐험해보세요. 이 프로젝트는 Docker Compose를 통해 실행할 수 있는 환경을 제공합니다. 다음 단계를 따라 설치를 진행하세요:

1. 대회 저장소 복제:
```bash
git clone https://github.com/metasejong-competition/metacom2025-metasejong
cd metacom2025-metasejong
```

이 저장소는 대회 애플리케이션 개발을 위한 환경을 제공합니다. Fork는 필요하지 않습니다. 저장소에는 다음과 같은 주요 파일들이 포함되어 있습니다:

- docker-compose.yml 설정 파일
- Makefile (빌드 및 실행 명령어)
- 예제 코드 및 문서

환경 구성을 위한 필수 요소이지만, 저장소에 포함되지 않고 별도로 다운로드 받아야 하는 파일들은 다음과 같습니다. 파일들을 다운로드 받는 방법에 대해서는 다음 단계 절차로 설명합니다. 

- MetaSejong Platform docker image 파일
- 가상공간 리소스 (IsaacSim USD 파일)

2. Docker 이미지 파일 다운로드

MetaSejong Platform docker image 파일은 NVIDIA IsaacSim으로 구현된 MetaSejong Platform을 docker로 배포하기 위한 이미지파일입니다. 참가자는 이 이미지를 다운로드 하여 시뮬레이션 어

[metasejong-metacom2025-r02.tar](https://drive.google.com/file/d/10r-tzDj0qS6OKWEle0gl4GnRtEitVKD5/view?usp=sharing)

2. Docker 이미지 로드
다운로드된 파일에는 Docker 이미지 tar 파일, docker-compose.yml, Makefile이 포함되어 있습니다. 이미지 로드는 다음 두 가지 방법으로 수행할 수 있습니다:

Docker 명령어를 직접 사용하거나 Makefile에 제공된 명령어를 통해 수행할 수 있습니다.

2.1. Docker 명령어 사용:
```bash
cd <git clone folder>
docker load -i <image_name>.tar
```

2.2. Makefile 명령어 사용:
```bash
make load-image
```

3. 플랫폼 실행

플랫폼을 실행하는 방법은 두 가지가 있습니다:

3.1. Docker 명령어 사용:
```bash
docker compose up
```

3.2. Makefile 사용:
```bash
make run
```

실행이 완료되면, 다음 URL에서 플랫폼에 접속할 수 있습니다:
- 웹 인터페이스: http://localhost:8080
- ROS2 인터페이스: localhost:11311

### 대회 참가자를 위한 데모 애플리케이션

데모 애플리케이션은 메타세종 플랫폼에서 제공하는 데이터를 활용하고 플랫폼에서 동작하는 로봇을 제어하는 예제 코드를 제공합니다. 이 애플리케이션은 다음과 같은 목적으로 제공됩니다:

- 플랫폼 API 사용 방법 시연
- 기본적인 로봇 제어 예제 제공
- 대회 미션 해결을 위한 참고 자료
- 개발 환경 설정 가이드

이 애플리케이션은 ROS2 애플리케이션 패키지 형태로 제공되며, 대회 절차에 따라 참가자의 애플리케이션과 메타세종 플랫폼 간의 메시지 교환을 포함합니다. 참가자들은 이를 주의 깊게 분석하고 필요한 절차를 따라야 합니다.

대회 참가자는 다음 절차를 따라야 합니다:

1. 참가 신청 절차

|참가자|   |주최측|
|---|---|---|
|참가 신청서 제출| --> | |
| | <-- | 팀 ID와 인증 토큰 발급 (이메일)|

2. 데모 애플리케이션 검토

주최측은 대회 참가자를 위한 데모 애플리케이션 GitHub 저장소를 제공합니다. 이 저장소는 다음과 같은 내용을 포함합니다:

- 기본적인 ROS2 노드 구현 예제
- 플랫폼 API 사용 예제
- 데이터 처리 및 분석 예제
- 로봇 제어 예제

이 저장소를 통해 참가자들은 메타세종 플랫폼이 제공하는 API와 데이터를 효과적으로 활용하는 방법을 배울 수 있습니다.

참가자들은 최종 제출물을 팀의 비공개 GitHub 저장소에서 관리하고, 주최측 계정을 readonly 협력자로 등록해야 합니다. 개발 및 제출을 위해 데모 애플리케이션 저장소를 Fork하는 것을 권장합니다.

데모 애플리케이션의 작업 흐름은 다음과 같습니다:

|참가자|   |MetaSejong 플랫폼|
|---|---|---|
|COMPETITOR_APP_STARTED (팀 ID, 인증 토큰)| > | 토큰 검증, 세션 키 발급 |
| | | MetaSejong 가상 공간에 대회 시나리오 구성 |
|  |  | 시간 제한 타이머 시작, 점수 계산 시작 |
| | < | ROS Topics를 통해 가상 공간 데이터 스트리밍 (/metasejong2025/xxx 형식)|
| | < | COMPETITOR_APP_STARTED_RESPONSE(세션 키) |
|ROS 데이터 분석을 통한 LEVEL 1 (객체 감지 및 포즈 추정)| | |
|REPORT_LEVEL1_RESULT(세션 키, 분석 결과)| > | 분석 결과 점수 계산 |
| | < | REPORT_LEVEL1_RESULT_RESPONSE |
|LEVEL 2 시작 (최적 경로 계산) | | |
|최적 경로 이동 및 pick&place 명령 전송 | > | |
|프로그램 종료 | | |

이 작업 흐름을 구현한 데모 애플리케이션의 가이드는 다음과 같습니다:

2.1 저장소 Fork 및 복제:
```bash
git clone https://github.com/metasejong-competition/metasejong-airobotics
cd metasejong-airobotics
```

2.2 프로젝트 구조

데모 애플리케이션은 다음과 같은 구조로 제공됩니다.
데모 애플리케이션은 참가자가 수행해야 하는 절차를 데모 형태로 제공하므로, 동작 구조를 명확히 이해해야 합니다. 그러나 이 코드를 사용할 필요는 없습니다.

데모 애플리케이션은 대회 참가자에게 요구되는 AI Robotics 기술들을 competition_task_implementation.py에 mockup 예제로 구현합니다. 참가자들은 mockup으로 구현된 입출력 데이터 형식을 고려하여 최적의 알고리즘을 구현해야 합니다.

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

2.3 환경 설정
데모 애플리케이션에는 다음 세 가지 환경 변수가 필요합니다:

이 세 가지 환경 변수는 참가 신청서를 제출한 후 이메일로 통보받는 팀 ID와 인증 토큰 정보입니다.
그리고 참가 팀이 지원하는 최종 단계에 대한 정보입니다.

- 단계 1: 고정 카메라 이미지에서 객체 감지 및 포즈 추정
- 단계 2: 감지된 객체의 효율적인 수집과 로봇 팔 제어를 통한 재활용 폐기물 수집을 위한 최적 경로 분석

```bash
# 환경 변수 설정 예시
export ENV_METASEJONG_TEAM_NAME="your_team_name"
export ENV_METASEJONG_TEAM_TOKEN="your_team_token"
export ENV_METASEJONG_TEAM_TARGET_LEVEL="your_target_level"

# 영구적인 환경 변수 설정 (선택 사항)
echo "export ENV_METASEJONG_TEAM_NAME=\"your_team_name\"" >> ~/.bashrc
```

demo용 어플리케이션 실행에 필요한 값은 다음과 같다.

|환경변수 | 값 |
|---|---|
|ENV_METASEJONG_TEAM_NAME|team_passion_for_challenges|
|ENV_METASEJONG_TEAM_TOKEN|87cef2059293b764451516c5e632e8b5|
|ENV_METASEJONG_TEAM_TARGET_LEVEL|2|

2.3 빌드 및 실행

프로젝트의 build와 실행은 make 명령을 통해 실행할 수 있다. 각 명령어는 다음과 같은 목적으로 사용됩니다:

|명령 | 설명 |
|---|---|
|make build-dev    | 개발용 Docker 이미지 빌드|
|make build-prod   | 배포용 Docker 이미지 빌드|
|make up-dev       | 개발 환경 실행|
|make up-prod      | 배포 환경 실행|
|make down         | 모든 컨테이너 중지|
|make clean        | 사용하지 않는 Docker 리소스 정리|
|make help         | 도움말 표시|

2.4 제출

앞서 언급된것과 같이 개발된 결과물은 github repository로 제출되어야 한다. 제출 과정은 다음과 같습니다:

1. 주최측에서는 참가신청서가 접수되면 이메일로 참가팀에 할당할 팀 ID와 인증토큰을 이메일로 회신한다. 
2. 이 때, 제출에 대한 안내와 함께 주최측의 github 계정을 제시하게 된다. 
3. 참가팀은 제출 기한이 만료되기 전에 개발한 결과물을 github repository에 push해야 한다.
4. 이메일에 포함된 주최측 github 계정을 repository의 collaborator로 등록해야 한다.

주최측에서는 공유된 repository의 main (또는 master) 브렌치의 코드를 이용하여 제출된 결과물을 평가하게 된다.

주최측에서는 github repository를 clone한 후 make up-prod 명령만을 수행하여 결과물을 평가한다.

참가자는 Makefile, Dockerfile, docker-compose.yml 파일을 팀이 개발한 프로젝트에 맞게 수정하여 주최측의 환경에서 실행될 수 있도록 준비해야 한다.

특수한 설정이 필요하다면, 실행 방법을 HOWTORUN.md 파일을 만들어 최대한 상세히 설명해야 한다. 이 파일에는 다음 내용이 포함되어야 합니다:

- 특수한 환경 설정 방법
- 추가로 필요한 패키지나 라이브러리
- 실행 시 주의사항
- 문제 해결 방법

## 다음 단계

- [기술 가이드](technical-guide.md)를 읽어보세요: 상세한 기술 정보를 확인할 수 있습니다.
- [대회 규칙](rules.md)을 검토하세요: 대회 참가를 위한 필수 규칙을 확인할 수 있습니다.
- [제출 가이드](submit-guide.md)를 확인하세요: 결과물 제출 방법에 대한 안내를 확인할 수 있습니다. 