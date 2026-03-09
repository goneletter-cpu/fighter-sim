#!/bin/bash
set -e  # stop on first command failure

echo "=== Fighter Sim Build ==="

# translated comment
if ! command -v brew &>/dev/null; then
    echo "[Error] Homebrew not found. Please install it first:"
    echo "  /bin/bash -c \"\$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)\""
    exit 1
fi

# translated comment
echo "[1/3] Checking dependencies..."
brew list glfw &>/dev/null || brew install glfw
brew list glm  &>/dev/null || brew install glm

# translated comment
GLFW_PREFIX=$(brew --prefix glfw)
GLM_PREFIX=$(brew --prefix glm)

INCLUDES="-I include -I${GLFW_PREFIX}/include -I${GLM_PREFIX}/include"
LIBS="-L${GLFW_PREFIX}/lib -lglfw -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo"

# translated comment
echo "[2/3] Building..."
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

echo "[3/3] Done!"
echo ""
echo "Run:"
echo "  ./build/fighter_sim"
echo ""
echo "Controls:"
echo "  W/S    = climb / dive"
echo "  A/D    = bank / diagonal assist"
echo "  Q/E    = strafe left / right"
echo "  Shift/Ctrl = game speed +/-"
echo "  ESC    = quit"
