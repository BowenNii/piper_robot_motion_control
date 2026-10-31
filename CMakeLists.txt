cmake_minimum_required(VERSION 3.21)

project(piper_robot_motion_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(Eigen3 REQUIRED)

# 仅控制构建/离线验证范围，不是机械臂运行参数。
option(PIPER_BUILD_PLACEHOLDER_APPS "Build unfinished Project2/3/4 examples" OFF)
option(PIPER_ENABLE_OFFLINE_PYTHON_TESTS "Register pure Python tests (no ROS/CAN)" ON)
option(PIPER_ENABLE_EXPERIMENT_DATA_TESTS "Verify retained Project1 dataset" OFF)
option(PIPER_ENABLE_ANALYSIS_TESTS "Test NumPy/SciPy offline analysis and dynamics" OFF)

# ============================================================
# Core library: piper_control
# ============================================================

add_library(piper_control
    # -------------------- Common / Validation ----------
    core/src/common/error_statistics.cpp
    core/src/validation/pose_error.cpp
    core/src/validation/kinematics_validation.cpp

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
    core/src/trajectory/time_parameterization.cpp

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
# HAL library: piper_hal
# ============================================================

# CAN 协议解码独立于 Eigen、ROS 和 SocketCAN，方便离线测试。
add_library(piper_hal
    hal/src/piper_can_decoder.cpp
    hal/src/can_log_parser.cpp
)
target_include_directories(piper_hal PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/hal/include)
target_compile_features(piper_hal PUBLIC cxx_std_17)
target_compile_options(piper_hal PRIVATE -Wall -Wextra -Wpedantic)

# ============================================================
# Experiment I/O library: piper_experiment_io
# ============================================================

# 文件读写不属于数学 core，也不属于 CAN HAL。
add_library(piper_experiment_io
    tools/experiment_io/src/fk_csv_writer.cpp
    tools/experiment_io/src/ik_trajectory_csv.cpp
)
target_include_directories(piper_experiment_io PUBLIC tools/experiment_io/include)
target_link_libraries(piper_experiment_io PUBLIC piper_control)
target_compile_options(piper_experiment_io PRIVATE -Wall -Wextra -Wpedantic)

# ============================================================
# Project 1：运动学与运动控制（每个main对应一个独立程序）
# ============================================================

# 实验1～3：CAN日志解码与FK一致性验证。
add_executable(fk_validation
    apps/project1/fk_validation.cpp
)
target_link_libraries(fk_validation PRIVATE
    piper_control
    piper_hal
    piper_experiment_io
)
target_compile_options(fk_validation PRIVATE -Wall -Wextra -Wpedantic)

# 连续SE(3)位姿路径的离线IK验证，不控制真机。
add_executable(ik_trajectory_validation
    apps/project1/ik_trajectory_validation.cpp
)
target_link_libraries(ik_trajectory_validation PRIVATE piper_control piper_experiment_io)
target_compile_options(ik_trajectory_validation PRIVATE -Wall -Wextra -Wpedantic)

# 五组关节时间规律离线对照；不改变原SE(3) IK实验入口。
add_executable(joint_profile_comparison apps/project1/joint_profile_comparison.cpp)
target_link_libraries(joint_profile_comparison PRIVATE piper_control)
target_compile_options(joint_profile_comparison PRIVATE -Wall -Wextra -Wpedantic)
add_executable(project1_feedback_export apps/project1/project1_feedback_export.cpp)
target_link_libraries(project1_feedback_export PRIVATE piper_control piper_hal piper_experiment_io)
target_compile_options(project1_feedback_export PRIVATE -Wall -Wextra -Wpedantic)

# ============================================================
# Project 2：动力学与模型控制
# ============================================================

if(PIPER_BUILD_PLACEHOLDER_APPS)
add_executable(project2
    apps/project2/main.cpp
)

target_link_libraries(
    project2
    PRIVATE
        piper_control
)

# ============================================================
# Project 3：约束轨迹优化
# ============================================================

add_executable(project3
    apps/project3/main.cpp
)

target_link_libraries(
    project3
    PRIVATE
        piper_control
)

# ============================================================
# Project 4：柔顺控制
# ============================================================

add_executable(project4
    apps/project4/main.cpp
)

target_link_libraries(
    project4
    PRIVATE
        piper_control
)
endif()

# ============================================================
# Tests
# ============================================================

include(CTest)

if(BUILD_TESTING)
    add_executable(test_comparison_profile tests/unit/test_comparison_profile.cpp)
    target_link_libraries(test_comparison_profile PRIVATE piper_control)
    add_test(NAME ComparisonProfileTest COMMAND test_comparison_profile)
    add_executable(test_time_parameterization tests/unit/test_time_parameterization.cpp)
    target_link_libraries(test_time_parameterization PRIVATE piper_control)
    add_test(NAME TimeParameterizationTest COMMAND test_time_parameterization)
    # -------------------- Application integration ------
    # 程序只有全部采样点通过时才返回0，不将测试绑死在101点。
    add_test(NAME IkTrajectoryValidationTest COMMAND ik_trajectory_validation)
    set_tests_properties(IkTrajectoryValidationTest PROPERTIES TIMEOUT 60)
    add_test(NAME IkTrajectoryCsvTest COMMAND ${CMAKE_COMMAND}
        -DIK_TRAJECTORY=$<TARGET_FILE:ik_trajectory_validation>
        -DTEST_ROOT=${CMAKE_CURRENT_BINARY_DIR}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/integration/test_ik_trajectory_csv.cmake)
    add_test(NAME FkCliIntegrationTest COMMAND ${CMAKE_COMMAND}
        -DFK_VALIDATION=$<TARGET_FILE:fk_validation>
        -DTEST_ROOT=${CMAKE_CURRENT_BINARY_DIR}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/integration/test_fk_cli.cmake)

    # -------------------- Validation / I/O / HAL --------
    add_executable(test_validation_modules tests/unit/test_validation_modules.cpp)
    target_link_libraries(test_validation_modules PRIVATE piper_control)
    add_test(NAME ValidationModulesTest COMMAND test_validation_modules)
    add_executable(test_fk_csv_writer tests/unit/test_fk_csv_writer.cpp)
    target_link_libraries(test_fk_csv_writer PRIVATE piper_experiment_io piper_hal)
    add_test(NAME FkCsvWriterTest COMMAND test_fk_csv_writer)
    add_executable(test_assembler_diagnostics tests/unit/test_assembler_diagnostics.cpp)
    target_link_libraries(test_assembler_diagnostics PRIVATE piper_hal)
    add_test(NAME AssemblerDiagnosticsTest COMMAND test_assembler_diagnostics)
    add_executable(test_can_log_parser tests/unit/test_can_log_parser.cpp)
    target_link_libraries(test_can_log_parser PRIVATE piper_hal)
    add_test(NAME CanLogParserTest COMMAND test_can_log_parser)

    add_executable(test_piper_can_decoder tests/unit/test_piper_can_decoder.cpp)
    target_link_libraries(test_piper_can_decoder PRIVATE piper_hal)
    add_test(NAME PiperCanDecoderTest COMMAND test_piper_can_decoder)

    # -------------------- Inverse Kinematics ------------
    add_executable(test_ik_solver tests/unit/test_ik_solver.cpp)
    target_link_libraries(test_ik_solver PRIVATE piper_control)
    add_test(NAME IKSolverTest COMMAND test_ik_solver)

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

    # -------------------- Cartesian velocity control ----
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

    # 所有自动注册项均为离线测试；不注册tests/hardware或ROS真机操作。
    get_property(piper_cpp_tests DIRECTORY PROPERTY TESTS)
    set_tests_properties(${piper_cpp_tests} PROPERTIES LABELS "cpp;offline" TIMEOUT 120)
    if(PIPER_ENABLE_OFFLINE_PYTHON_TESTS OR PIPER_ENABLE_EXPERIMENT_DATA_TESTS OR PIPER_ENABLE_ANALYSIS_TESTS)
        find_package(Python3 3.10 REQUIRED COMPONENTS Interpreter)
    endif()
    if(PIPER_ENABLE_OFFLINE_PYTHON_TESTS)
        foreach(piper_test IN ITEMS
                tests/integration/test_movej_single_joint_probe.py
                tests/integration/test_robot_graph_discovery.py
                tests/unit/test_real_ik_trajectory.py
                tests/unit/test_repository_configuration.py)
            get_filename_component(piper_test_name ${piper_test} NAME_WE)
            add_test(NAME ${piper_test_name} COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/${piper_test} -q)
            set_tests_properties(${piper_test_name} PROPERTIES
                LABELS "python;offline" TIMEOUT 60
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
        endforeach()
    endif()
    if(PIPER_ENABLE_EXPERIMENT_DATA_TESTS)
        if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/experiments/project1/2026-10-27_movej_repeat/project1_closeout/inventory.json")
            message(FATAL_ERROR "Project1 dataset is required by PIPER_ENABLE_EXPERIMENT_DATA_TESTS")
        endif()
        add_test(NAME Project1CloseoutDataTest COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/integration/test_project1_closeout_data.py -q)
        set_tests_properties(Project1CloseoutDataTest PROPERTIES
            LABELS "python;offline;data" TIMEOUT 60
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    endif()
    if(PIPER_ENABLE_ANALYSIS_TESTS)
        execute_process(COMMAND ${Python3_EXECUTABLE} -c "import numpy, scipy, matplotlib"
            RESULT_VARIABLE piper_analysis_dependencies)
        if(NOT piper_analysis_dependencies EQUAL 0)
            message(FATAL_ERROR "Analysis needs NumPy/SciPy/Matplotlib. See analysis/README.md; select the analysis Python interpreter explicitly.")
        endif()
        add_test(NAME PythonAnalysisTest COMMAND ${Python3_EXECUTABLE} -m unittest discover
            -s analysis/tests -v)
        set_tests_properties(PythonAnalysisTest PROPERTIES LABELS "python;offline;analysis"
            TIMEOUT 120 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
        if(PIPER_ENABLE_EXPERIMENT_DATA_TESTS)
            add_test(NAME PythonRefactorRegressionTest COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/tests/integration/test_python_refactor.py -q)
            set_tests_properties(PythonRefactorRegressionTest PROPERTIES
                LABELS "python;offline;analysis;data;regression"
                TIMEOUT 180 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
            add_test(NAME PythonMigrationDataTest COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/tests/integration/test_python_migration.py -q)
            set_tests_properties(PythonMigrationDataTest PROPERTIES LABELS "python;offline;analysis;data"
                TIMEOUT 120 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
        endif()
    endif()
endif()
