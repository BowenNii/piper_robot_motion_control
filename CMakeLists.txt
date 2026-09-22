cmake_minimum_required(VERSION 3.16)

project(piper_robot_motion_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(Eigen3 REQUIRED)

# ============================================================
# Core library: piper_control
# ============================================================

add_library(piper_control

    # -------------------- Lie Group --------------------
    core/src/lie_group/so3.cpp
    core/src/lie_group/se3.cpp
    core/src/lie_group/screw.cpp
    core/src/lie_group/wrench.cpp

    # -------------------- Kinematics -------------------
    core/src/kinematics/forward_kinematics.cpp
    core/src/kinematics/jacobian.cpp
    core/src/kinematics/singularity.cpp
    core/src/kinematics/ik_solver.cpp

    # -------------------- Robot Model ------------------
    core/src/robot_model/piper_model.cpp

    # -------------------- Dynamics ---------------------
    core/src/dynamics/spatial.cpp
    core/src/dynamics/spatial_inertia.cpp
    core/src/dynamics/rnea.cpp
    core/src/dynamics/crba.cpp
    core/src/dynamics/gravity.cpp
    core/src/dynamics/friction.cpp
    core/src/dynamics/regressor.cpp

    # -------------------- Control ----------------------
    core/src/control/pd_controller.cpp
    core/src/control/gravity_compensation.cpp
    core/src/control/cartesian_velocity_controller.cpp  
    core/src/control/computed_torque.cpp
    core/src/control/joint_impedance.cpp
    core/src/control/cartesian_impedance.cpp

    # -------------------- Trajectory -------------------
    core/src/trajectory/se3_trajectory.cpp

    # -------------------- Optimization -----------------
    core/src/optimization/qp_problem.cpp
    core/src/optimization/trajectory_optimizer.cpp
    core/src/optimization/sequential_convexification.cpp

    # -------------------- Estimation -------------------
    core/src/estimation/low_pass_filter.cpp
    core/src/estimation/kalman_filter.cpp
    core/src/estimation/joint_state_estimator.cpp

    # -------------------- Safety -----------------------
    core/src/safety/joint_limits.cpp
    core/src/safety/command_limiter.cpp
    core/src/safety/watchdog.cpp
)

target_include_directories(
    piper_control
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/core/include
)

target_link_libraries(
    piper_control
    PUBLIC
        Eigen3::Eigen
)

target_compile_features(
    piper_control
    PUBLIC
        cxx_std_17
)

target_compile_options(
    piper_control
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

# ============================================================
# Projects
# ============================================================

add_executable(project1
    apps/project1/main.cpp
)

target_link_libraries(
    project1
    PRIVATE
        piper_control
)

add_executable(project2
    apps/project2/main.cpp
)

target_link_libraries(
    project2
    PRIVATE
        piper_control
)

add_executable(project3
    apps/project3/main.cpp
)

target_link_libraries(
    project3
    PRIVATE
        piper_control
)

add_executable(project4
    apps/project4/main.cpp
)

target_link_libraries(
    project4
    PRIVATE
        piper_control
)

# ============================================================
# Tests
# ============================================================

include(CTest)

if(BUILD_TESTING)

    # -------------------- SO(3) ------------------------
    add_executable(test_so3
        tests/unit/test_so3.cpp
    )

    target_link_libraries(
        test_so3
        PRIVATE
            piper_control
    )

    add_test(
        NAME SO3Test
        COMMAND test_so3
    )

    # -------------------- SE(3) ------------------------
    add_executable(test_se3
        tests/unit/test_se3.cpp
    )

    target_link_libraries(
        test_se3
        PRIVATE
            piper_control
    )

    add_test(
        NAME SE3Test
        COMMAND test_se3
    )

    # -------------------- Screw -------------------------
    add_executable(test_screw
        tests/unit/test_screw.cpp
    )

    target_link_libraries(
        test_screw
        PRIVATE
            piper_control
    )

    add_test(
        NAME ScrewTest
        COMMAND test_screw
    )

    # -------------------- Forward Kinematics ------------
    add_executable(test_forward_kinematics
        tests/unit/test_forward_kinematics.cpp
    )

    target_link_libraries(
        test_forward_kinematics
        PRIVATE
            piper_control
    )

    add_test(
        NAME ForwardKinematicsTest
        COMMAND test_forward_kinematics
    )

    # -------------------- Jacobian ---------------------
    add_executable(test_jacobian
        tests/unit/test_jacobian.cpp
    )

    target_link_libraries(
        test_jacobian
        PRIVATE
            piper_control
    )

    add_test(
        NAME JacobianTest
        COMMAND test_jacobian
    )

    # -------------------- Singularity ------------------
    add_executable(test_singularity
        tests/unit/test_singularity.cpp
    )

    target_link_libraries(
        test_singularity
        PRIVATE
            piper_control
    )

    add_test(
        NAME SingularityTest
        COMMAND test_singularity
    )

    # -------------------- SE(3) Trajectory --------------
    add_executable(test_se3_trajectory
        tests/unit/test_se3_trajectory.cpp
    )

    target_link_libraries(
        test_se3_trajectory
        PRIVATE
            piper_control
    )

    add_test(
        NAME SE3TrajectoryTest
        COMMAND test_se3_trajectory
    )

    add_executable(test_cartesian_velocity_controller
    tests/unit/test_cartesian_velocity_controller.cpp
)

    target_link_libraries(
        test_cartesian_velocity_controller
        PRIVATE
            piper_control
    )

    add_test(
        NAME CartesianVelocityControllerTest
        COMMAND test_cartesian_velocity_controller
    )
    
    # -------------------- Command Limiter --------------
    add_executable(test_command_limiter
        tests/unit/test_command_limiter.cpp
    )

    target_link_libraries(
        test_command_limiter
        PRIVATE
            piper_control
    )

    add_test(
        NAME CommandLimiterTest
        COMMAND test_command_limiter
    )

    # -------------------- PiPER Model -------------------
    add_executable(test_piper_model
        tests/unit/test_piper_model.cpp
    )

    target_link_libraries(
        test_piper_model
        PRIVATE
            piper_control
    )

    add_test(
        NAME PiperModelTest
        COMMAND test_piper_model
    )

endif()