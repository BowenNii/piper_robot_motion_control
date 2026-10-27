string(RANDOM LENGTH 12 ALPHABET 0123456789abcdef suffix)
set(root "${TEST_ROOT}/ik-csv-test-${suffix}")
file(MAKE_DIRECTORY "${root}")
set(path "${root}/trajectory.csv")
execute_process(COMMAND "${IK_TRAJECTORY}" "${path}"
    RESULT_VARIABLE code OUTPUT_VARIABLE output ERROR_VARIABLE error)
if(NOT code EQUAL 0 OR NOT EXISTS "${path}")
    message(FATAL_ERROR "CSV export failed: ${output} ${error}")
endif()
file(STRINGS "${path}" lines)
list(LENGTH lines length)
if(NOT length EQUAL 502)
    message(FATAL_ERROR "Expected header plus 501 data rows")
endif()
list(GET lines 0 header)
if(NOT header MATCHES "position_error_m,rotation_error_rad" OR
   NOT header MATCHES "delta_q6_rad" OR NOT header MATCHES "target_r33" OR
   NOT header MATCHES "q_goal6_rad" OR NOT header MATCHES "t_s,ds_dt,d2s_dt2" OR
   NOT header MATCHES "ddq6_rad_s2" OR NOT header MATCHES "offline_velocity_limit_rad_s")
    message(FATAL_ERROR "CSV schema missing required fields")
endif()
foreach(duration IN ITEMS 0 -1 nan inf 10junk 0.1)
    set(rejected_path "${root}/rejected-${duration}.csv")
    execute_process(COMMAND "${IK_TRAJECTORY}" "${rejected_path}" "${duration}"
        RESULT_VARIABLE code OUTPUT_QUIET ERROR_QUIET)
    if(code EQUAL 0 OR EXISTS "${rejected_path}")
        message(FATAL_ERROR "Invalid/too fast duration accepted: ${duration}")
    endif()
endforeach()

# 终点IK再关节五次轨迹，固定60秒20ms；原模式列名和点数不变。
execute_process(COMMAND "${IK_TRAJECTORY}" "${root}/endpoint.csv" 60
    --endpoint-joint-target 0 0 0 0 0 0 .3 .3 -.3 .3 .3 .5
    RESULT_VARIABLE code OUTPUT_QUIET ERROR_VARIABLE error)
if(NOT code EQUAL 0)
    message(FATAL_ERROR "Endpoint IK joint trajectory failed: ${error}")
endif()
file(STRINGS "${root}/endpoint.csv" endpoint_lines)
list(LENGTH endpoint_lines endpoint_length)
list(GET endpoint_lines 0 endpoint_header)
if(NOT endpoint_length EQUAL 3002 OR NOT endpoint_header MATCHES ",path_mode$")
    message(FATAL_ERROR "Endpoint CSV must have 3001 samples and explicit path mode")
endif()
execute_process(COMMAND "${IK_TRAJECTORY}" "${root}/bad-endpoint.csv" 10
    --endpoint-joint-target 0 0 0 0 0 0 .3 .3 -.3 .3 .3 .5
    RESULT_VARIABLE code OUTPUT_QUIET ERROR_QUIET)
if(code EQUAL 0 OR EXISTS "${root}/bad-endpoint.csv")
    message(FATAL_ERROR "Wrong endpoint experiment duration accepted")
endif()
file(SHA256 "${path}" before)
execute_process(COMMAND "${IK_TRAJECTORY}" "${path}"
    RESULT_VARIABLE code OUTPUT_QUIET ERROR_QUIET)
file(SHA256 "${path}" after)
if(code EQUAL 0 OR NOT before STREQUAL after)
    message(FATAL_ERROR "CSV overwrite was not rejected")
endif()
execute_process(COMMAND "${IK_TRAJECTORY}" "${root}/missing/output.csv"
    RESULT_VARIABLE code OUTPUT_QUIET ERROR_QUIET)
if(code EQUAL 0)
    message(FATAL_ERROR "Missing output directory was not rejected")
endif()
execute_process(COMMAND "${IK_TRAJECTORY}" extra extra
    RESULT_VARIABLE code OUTPUT_QUIET ERROR_QUIET)
if(code EQUAL 0)
    message(FATAL_ERROR "Extra CLI arguments were not rejected")
endif()

# 相对笛卡尔目标：真实反馈形状的非零起点，保持姿态、基座Z平移2mm。
set(q_start 0.0999724595542352 0.09972811345895599 -0.1015432558810301
    0.08391543043588737 0.01153662635568252 -0.08513716091228339)
execute_process(COMMAND "${IK_TRAJECTORY}" "${root}/relative.csv" 10
    --relative-target ${q_start} 0 0 0.002
    RESULT_VARIABLE code OUTPUT_VARIABLE output ERROR_VARIABLE error)
if(NOT code EQUAL 0 OR NOT EXISTS "${root}/relative.csv")
    message(FATAL_ERROR "Relative trajectory failed: ${output} ${error}")
endif()
foreach(offset IN ITEMS 0 0.006 nan)
    execute_process(COMMAND "${IK_TRAJECTORY}" "${root}/bad-relative-${offset}.csv" 10
        --relative-target ${q_start} 0 0 "${offset}"
        RESULT_VARIABLE code OUTPUT_QUIET ERROR_QUIET)
    if(code EQUAL 0 OR EXISTS "${root}/bad-relative-${offset}.csv")
        message(FATAL_ERROR "Invalid relative offset accepted: ${offset}")
    endif()
endforeach()
