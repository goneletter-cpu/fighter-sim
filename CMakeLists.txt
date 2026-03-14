cmake_minimum_required(VERSION 3.16)
project(FighterSim VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# translated comment
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)

# JSBSim options (avoid extra components)
set(BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_MODULE OFF CACHE BOOL "" FORCE)
set(INSTALL_JSBSIM_PYTHON_MODULE OFF CACHE BOOL "" FORCE)
set(BUILD_JULIA_PACKAGE OFF CACHE BOOL "" FORCE)
set(BUILD_MATLAB_SFUNCTION OFF CACHE BOOL "" FORCE)

# JSBSim library
add_subdirectory(jsbsim EXCLUDE_FROM_ALL)

# translated comment
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "src/*.cpp")

add_executable(fighter_sim ${SOURCES})

target_include_directories(fighter_sim PRIVATE include)

target_link_libraries(fighter_sim
    OpenGL::GL
    glfw
    glm::glm
    libJSBSim
)

target_compile_definitions(fighter_sim PRIVATE
    FIGHTER_SIM_ROOT="${CMAKE_SOURCE_DIR}"
)

# translated comment
target_compile_options(fighter_sim PRIVATE
    -Wall -Wextra -Wpedantic
)
