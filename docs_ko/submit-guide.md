# 제출 가이드

## 제출 과정

1. **제출물 준비**
   - 개발 환경 설정 및 코드 작성
   - 필수 파일 준비 (Makefile, Dockerfile 등)
   - 문서화 작업 완료

2. **GitHub을 통한 제출**
   - 제출 마감시각 전에 최종 코드를 GitHub 저장소에 push
   - 주최측 GitHub 계정을 collaborator로 등록

## 제출 요구사항

### 필수 파일

- Makefile: 빌드 및 실행 명령어 정의
- Dockerfile: 컨테이너 이미지 설정
- docker-compose.yml: 서비스 구성 및 의존성 관리
- HOWTORUN.md: 팀 정보와 실행 방법, 설정 가이드

### 문서화 요구사항

HOWTORUN.md에는 다음 내용이 포함되어야 합니다:
- **팀 이름**
- 설치 방법
- 사용 예시
- 의존성 목록
- 환경 설정 방법
- 문제 해결 가이드

## 제출 마감일

- 정규 제출: June 16, 2025 (Submission deadline at 24:00 UTC)

## 주의사항

1. **저장소 관리**
   - main/master 브랜치에 최종 코드 push
   - 비공개 저장소 설정 확인
   - 주최측 GitHub 계정을 collaborator로 등록
   - **주의** 마감 시각 이후 update된 코드는 평가에 반영되지 않을 수 있습니다.

2. **실행 방법**
   - `make run` 명령으로 실행 가능해야 함
   - 특수한 설정이 필요한 경우 상세히 문서화
   - 모든 의존성은 requirements.txt에 명시

