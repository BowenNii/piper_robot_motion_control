cmake_minimum_required(VERSION 3.16)
project(piper_robot_motion_control LANGUAGES CXX)

# 所有可执行文件输出到 build 根目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

# ====================== 核心库 piper_control ======================
add_library(piper_control
    core/src/lie_group/so3.cpp
    core/src/lie_group/se3.cpp
    core/src/lie_group/screw.cpp
    core/src/kinematics/forward_kinematics.cpp
    core/src/kinematics/jacobian.cpp
    core/src/kinematics/singularity.cpp
    core/src/kinematics/ik_solver.cpp
)

target_include_directories(piper_control
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/core/include
)

target_link_libraries(piper_control
    PUBLIC
        Eigen3::Eigen
)

target_compile_features(piper_control PUBLIC cxx_std_17)

target_compile_options(piper_control
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

# ====================== Project 1 ======================
add_executable(project1
    apps/project1/main.cpp
)

target_link_libraries(project1
    PRIVATE
        piper_control
)

# ====================== Project 2 ======================
add_executable(project2
    apps/project2/main.cpp
)

target_link_libraries(project2
    PRIVATE
        piper_control
)

# ====================== Project 3 ======================
add_executable(project3
    apps/project3/main.cpp
)

target_link_libraries(project3
    PRIVATE
        piper_control
)

# ====================== Project 4 ======================
add_executable(project4
    apps/project4/main.cpp
)

target_link_libraries(project4
    PRIVATE
        piper_control
)

# ====================== 单元测试 + CTest ======================
enable_testing()

# test_so3
add_executable(test_so3
    tests/unit/test_so3.cpp
)

target_link_libraries(test_so3
    PRIVATE
        piper_control
)

add_test(NAME SO3Test COMMAND test_so3)

# test_se3
add_executable(test_se3
    tests/unit/test_se3.cpp
)

target_link_libraries(test_se3
    PRIVATE
        piper_control
)

add_test(NAME SE3Test COMMAND test_se3)

# test_poe_fk
# add_executable(test_poe_fk
#     tests/unit/test_poe_fk.cpp
# )
# target_link_libraries(test_poe_fk PRIVATE piper_control)
# add_test(NAME PoeFKTest COMMAND test_poe_fk)

# test_jacobian
# add_executable(test_jacobian
#     tests/unit/test_jacobian.cpp
# )
# target_link_libraries(test_jacobian PRIVATE piper_control)
# add_test(NAME JacobianTest COMMAND test_jacobian)
