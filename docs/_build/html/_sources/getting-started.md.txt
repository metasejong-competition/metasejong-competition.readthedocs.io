# Getting Started Guide

## Introduction

Welcome to the META-SEJONG AI Robotics Challenge 2025! 이 행사는 IEEE MetaCom 2025 행사의 부대행사로서 MetaCom 2025 Student challenge programs에 해당합니다. 

META-Sejong AI Robotics Challenge는 도전적인 학생들이 팀을 이루어 Embodied AI 기술들을 마음껏 활용할 수 있도록 제공되는 세종대학교를 모델로한 메타버스 공간을 제공합니다. 

다음 가이드에 따라 개발환경을 구성하고, ROS2 표준 인터페이스를 통해 가상환경과 소통하면서 주어진 임무를 완수해보세요.

## Prerequisites

Before you begin, make sure you have:

### MetaSejong (메타버스 서비스 플랫폼)
- Python 3.10 or higher installed
- Git installed
- Basic understanding of Python programming
- Familiarity with ROS2
- Familiarity with NVidia IsaacSim 4.0.2
- Docker and Docker composer 

### 경진대회 참가자 개발환경
- Python 3.10 or higher installed
- Git installed
- Basic understanding of Python programming
- Familiarity with ROS2
- Docker and Docker composer 


## Installation

### Meta-Sejong platform

Meta-Sejong platform은 NVidia IsaacSim을 이용하여 구축된 세종대학교 메타버스 가상환경입니다.  더불어 경진대회에서 참가가자 임무를 수행하는데 필요한 API를 제공합니다. 
제공되는 API에 대한 상세 내용은 별도 페이지에서 설명합니다.

Meta-Sejong platform은 참가자가 개발하는 AI Robotics application을 훈련하는데 있어 카운터파트 역할을 수행합니다. 

다음 절차에 따라 docker 기반으로 isaacsim을 구동하고, 메타버스로 구현된 세종대학교를 확인하세요.
이 프로젝트는 docker-compose를 통해 구동할 수 있는 환경을 제공합니다. 

1. Clone the competition repository:
```bash
git clone https://github.com/metasejong-competition/metacom2025-metasejong
cd metacom2025-metasejong
```

이 리포지토리는 경진대회용 어플리케이션을 개발하기 위한 일종의 환경을 제공합니다. fork할 필요는 없습니다. 

2. Docker image load
git으로 다운로드 받은 파일들은 docker image tar파일, docker-compose.yml, Makefile로 구성되어 있습니다. 

docker 명령을 직접 입력하거나 Makefile을 통해 제공되는 명령을 통해 수행할 수 있습니다. 

2.1. docker 명령을 사용하는 경우 

```
cd <git clone folder>
docker load -i <image_name>.tar
```

2.2. Makefile 명령을 사용하는 경우 
```
make load-image
```

3. 실행 

3.1 docker 명령을 사용하는 경우 

Using docker compose 
```bash
docker compose up
```

3.2. Makefile을 사용하는 경우 
```bash
make run
```


### 경진대회 참가자를 위한 Demo application

경진대회 참가자를 위한 demo application은 metasejong 플랫폼에서 제공하는 데이터의 활용과, 플랫폼 상에서 운용되는 로봇 제어를 위한 예제 코드를 제공한다. 

어플리케이션은 ROS2 어플리케이션 패키지 형태를 하고 있으며, 경연 진행 절차에 대한 참가자 응용프로그램과 metasejong 플랫폼 간의 메시지 교환을 포함하고 있기 때문에 참가자는 주의깊게 분석해보고 요구되는 절차를 따라야 한다.

경진대회 참가자는 다음과 같은 절차를 따라야 한다.


1. 참가 신청 

|참가자|   |주최측|
|---|---|---|
|참가신청서 제출| --> | |
| | <-- | 참가팀 ID와 인증 토큰 발급(email)|

2. 참가자용 데모 응용프로그램 확인  

주최측에서는 경진대회 참가자를 위한 demo application github repository를 제공합니다. 

참가자는 이 repository를 통해 제공되는 demo application을 통해 metasejong 플랫폼에서 제공하는 API와 데이터에 대한 명확한 활용 방법을 학습할 수 있을것입니다. 

참고로, 참가자들이 개발한 최종 결과물은 팀의 private github repository로 관리하면서 주최측 계정을 readonly collaborator로 등록할것을 요청받게 됩니다.  약한 권고사항으로 demo application repository를 fork하여 개발에 활용 후 제출하기를 바랍니다. 

demo application의 동작 흐름은 다음과 같습니다. 


|참가자|   |MetaSejong 플랫폼|
|---|---|---|
|COMPETITOR_APP_STARTED (팀 ID, 인증토큰)| > | 토큰 검증, 세션키 발급 |
| | | MetaSejong 가상공간에 경연 시나리오 구성 |
|  |  | 시간제한 타이머 시작, 채점 시작 |
| | < | /metasejong2025/xxx 형식의 ROS Topic으로 가상공간 데이터 스트리밍|
| | < | COMPETITOR_APP_STARTED_RESPONSE(세션키) |
|ROS 데이터 분석을 통한 LEVEL 1(object detection & pose estimation)| | |
|REPORT_LEVEL1_RESULT(세션키, 분석결과)| > | 분설결과 채점 |
| | < | REPORT_LEVEL1_RESULT_RESPONSE |
|LEVEL 2 시작(최적경로 계산) | | |
|전체 최적경로 이동 명령 & pick&place 명령 | > | |
|프로그램 종료 | | |


이러한 동작이 구현된 demo application에 대한 가이드는 다음과 같다.

2.1 Fork & clone repository:
```bash
git clone https://github.com/metasejong-competition/metasejong-airobotics
cd metasejong-airobotics
```

2.2 Project structure

demo application은 아래와 같은 구조로 제공된다. 
demo application은 참가자가 반드시 수행해야 하는 절차를 demo 형태고 제공하기 때문에 동작 구조를 명확하게 파악해야 한다. 하지만, 반드시 이 코드를 사용할 필요는 없다.

demo application에는 경진대회 참가자에게 요구하는 AI Robotics 기술들의 구현을 mockup 형태로 competition_task_implementation.py에 예시로서 구현하고 있다. 참가자들은 mockup으로 구현된 입출력 데이터 형태를 고려하여 최적의 알고리즘을 사용하거나 고안하여 구현해야 한다. 

```
.
|   # 프로젝트 가이드 문서 
├── README.md       
|   # Dockerizing 관련 파일 
├── Dockerfile      
├── Dockerfile.dev
├── docker-compose.yaml
├── entrypoint.sh
|   # 개발, 채정 등에 활용되는 명령 정의
├── Makefile
|   # 참가자 응용프로그램용 워크스페이스(ROS2 Workspace)
└── metasejong_competitor_ws
    └── src
        |   
        └── airobotics_app
            |   #   ROS2 Package 정의와 module dependancy
            ├── package.xml
            ├── requirements.txt
            |   #   참가자용 ROS Node 구현 
            ├── airobotics_node
            │   ├── __init__.py
            |   |   #   ROS Node 실행 entry point 
            │   ├── airobotics_node.py
            |   |   #   경진대회 참가자 Application의 기본 동작 템플릿 구현 예시(Abstract implementation)
            │   ├── competition_task_base.py
            |   |   #   경진대회 참가자 Application의 기본 동작 템플릿 구현 예시
            │   ├── competition_task_implementation.py
            │   ├── competitor_request_message.py
            |   |   #   Utility 기능 구현 예시
            │   ├── robot_node.py
            │   ├── robot_util.py
            │   └── world_coordinates_util.py
            ├── resource
            │   ├── airobotics_app
            │   └── metasejong
            ├── setup.cfg
            └── setup.py
```


2.3 환경 구성
데모용 어플리케이션에서는 다음과 같은 3개의 환경변수를 필수로 요구하고 있다.

3개의 환경변수들은 참가신청서 제출 후 이메일을 통해 고지되는 팀 ID와 인증토큰 정보이다. 
그리고, 참가팀이 지원하느 최종 단계에 대한 정보이다

- 1단계: Fixed camera 영상으로부터 object detection과 pose estimation
- 2단계: detect된 object의 효율적인 수거를 위한 최적경로 분석 및 로봇팔 제어를 통한 재활용 쓰레기 수거


```bash
# 환경변수 설정 예시
export ENV_METASEJONG_TEAM_NAME="your_team_name"
export ENV_METASEJONG_TEAM_TOKEN="your_team_token"
export ENV_METASEJONG_TEAM_TARGET_LEVEL="your_target_level"

# 환경변수 영구 설정 (선택사항)
echo "export ENV_METASEJONG_TEAM_NAME=\"your_team_name\"" >> ~/.bashrc
echo "export ENV_METASEJONG_TEAM_TOKEN=\"your_team_token\"" >> ~/.bashrc
echo "export ENV_METASEJONG_TEAM_TARGET_LEVEL=\"your_target_level\"" >> ~/.bashrc
source ~/.bashrc
```

demo용 어플리케이션 실행에 필요한 값은 다음과 같다.

|환경변수 | 값 |
|---|---|
|ENV_METASEJONG_TEAM_NAME|team_passion_for_challenges|
|ENV_METASEJONG_TEAM_TOKEN|87cef2059293b764451516c5e632e8b5|
|ENV_METASEJONG_TEAM_TARGET_LEVEL|2|


2.3 빌드 및 실행

프로젝트의 build와 실행은 make 명령을 통해 실행할 수 있다.

사용할 수 있는 make 명령 목록은 다음과 같으며, 프로젝트의 README.md 파일에 보다 상세한 내용이 기술되어 있다. 

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

앞서 언급된것과 같이 개발된 결과물은 github repository로 제출되어야 한다. 주최측에서는 참가신청서가 접수되면 이메일로 참가팀에 할당할 팀 ID와 인증토큰을 이메일로 회신한다. 

이 때, 제출에 대한 안내와 함께 주최측의 github 계정을 제시하게 된다. 

참가팀은 제출 기안이 만료되기 전에 개발한 결과물을 github repository에 push하고, 이메일에 포함된 주최측 github 계정을 reposotiry의 collaborator로 등록할것이 요구된다.

조최측에서는 공유된 respository의 main (또는 master) 브렌치의 코드를 이용하여  제출된 결과물을 평가하게 된다.

주최측에서는 github repository를 clone한 후 make run 명령만을 수행하여 결과물을 평가한다.

참가자는 Makefile, Dockerfile, docker-compose.yml 파일을 팀이 개발한 프로젝트에 맞게 수정하여 주최측의 환경에서 실행될 수 있도록 준비해야 한다.

특수한 설정이 필요하다면, 실행 방법을 HOWTORUN.md 파일을 만들어 최대한 상세히 설명해야 한다.





## Next Steps

- Read the [Technical Guide](technical-guide.md) for detailed technical information
- Review the [Competition Rules](rules.md)
- Check the [Submission Guide](submit-guide.md) for submission instructions 