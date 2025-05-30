.PHONY: clean build serve

# 기본 설정
PYTHON := python3
SPHINX := $(PYTHON) -m sphinx
SERVER_PORT := 8000

# 빌드 디렉토리
BUILD_DIR := _build
HTML_DIR := $(BUILD_DIR)/html

# 기본 타겟
all: build

# 문서 빌드
build:
	@echo "Building documentation..."
	@cd docs && $(SPHINX) -b html . $(BUILD_DIR)/html

# 로컬 서버 실행
serve:
	@echo "Starting local server at http://localhost:$(SERVER_PORT)"
	@cd $(HTML_DIR) && $(PYTHON) -m http.server $(SERVER_PORT)

# 빌드 후 서버 실행
dev: build serve

# 빌드 파일 정리
clean:
	@echo "Cleaning build files..."
	@rm -rf $(BUILD_DIR)

# 도움말
help:
	@echo "사용 가능한 명령어:"
	@echo "  make build  - 문서 빌드"
	@echo "  make serve  - 로컬 서버 실행"
	@echo "  make dev    - 문서 빌드 후 서버 실행"
	@echo "  make clean  - 빌드 파일 정리"
	@echo "  make help   - 도움말 표시" 