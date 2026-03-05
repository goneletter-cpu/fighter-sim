#!/bin/bash
set -e  # 任何命令失败就停下来

echo "=== Fighter Sim Build ==="

# ── 1. 检查 Homebrew ──────────────────────────────────────────────────────────
if ! command -v brew &>/dev/null; then
    echo "[错误] 没有找到 Homebrew。请先安装："
    echo "  /bin/bash -c \"\$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)\""
    exit 1
fi

# ── 2. 安装依赖（已安装则跳过）────────────────────────────────────────────────
echo "[1/3] 检查依赖..."
brew list glfw &>/dev/null || brew install glfw
brew list glm  &>/dev/null || brew install glm

# ── 3. 找头文件和库路径 ────────────────────────────────────────────────────────
GLFW_PREFIX=$(brew --prefix glfw)
GLM_PREFIX=$(brew --prefix glm)

INCLUDES="-I include -I${GLFW_PREFIX}/include -I${GLM_PREFIX}/include"
LIBS="-L${GLFW_PREFIX}/lib -lglfw -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo"

# ── 4. 编译 ───────────────────────────────────────────────────────────────────
echo "[2/3] 编译..."
mkdir -p build

clang++ -std=c++17 -O2 -Wall \
    $INCLUDES \
    src/pid.cpp \
    src/physics.cpp \
    src/flight_controller.cpp \
    src/renderer.cpp \
    src/main.cpp \
    $LIBS \
    -o build/fighter_sim

echo "[3/3] 完成！"
echo ""
echo "运行方式："
echo "  ./build/fighter_sim"
echo ""
echo "操控："
echo "  W/S    = 俯仰"
echo "  A/D    = 滚转"
echo "  Q/E    = 偏航"
echo "  Shift/Ctrl = 油门"
echo "  ESC    = 退出"
