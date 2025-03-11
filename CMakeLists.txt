cmake_minimum_required(VERSION 3.1.0)

set(CMAKE_CXX_COMPILER /usr/local/gcc-11.4/bin/g++)
project(unitree_actuator_sdk)

# 使用 C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# 包含头文件路径
include_directories(include)
file(GLOB_RECURSE CONTORL_SRCS "contorl/src/*.cpp")
link_directories(lib)

# 查找 yaml-cpp 库
find_package(yaml-cpp REQUIRED)

# 根据系统架构选择库
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(EXTRA_LIBS libUnitreeMotorSDK_Arm64.so)
else()
    set(EXTRA_LIBS libUnitreeMotorSDK_Linux64.so)
endif()

# 示例程序 example_a1_motor
add_executable(example_a1_motor example/example_a1_motor.cpp include/contorl/src/leg_control.cpp)
# 链接 Unitree SDK 和 yaml-cpp
target_link_libraries(example_a1_motor ${EXTRA_LIBS} yaml-cpp)


# Python绑定部分
set(LIBRARY_OUTPUT_PATH "../lib")
add_subdirectory(thirdparty/pybind11)
pybind11_add_module(unitree_actuator_sdk thirdparty/python_wrapper/wrapper.cpp)
target_link_libraries(unitree_actuator_sdk PRIVATE ${EXTRA_LIBS})
set_target_properties(unitree_actuator_sdk PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${LIBRARY_OUTPUT_PATH}")
