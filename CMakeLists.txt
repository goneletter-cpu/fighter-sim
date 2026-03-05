cmake_minimum_required(VERSION 3.16)
project(FighterSim VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 找依赖
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)

# 源文件
file(GLOB_RECURSE SOURCES "src/*.cpp")

add_executable(fighter_sim ${SOURCES})

target_include_directories(fighter_sim PRIVATE include)

target_link_libraries(fighter_sim
    OpenGL::GL
    glfw
    glm::glm
)

# 编译警告
target_compile_options(fighter_sim PRIVATE
    -Wall -Wextra -Wpedantic
)
