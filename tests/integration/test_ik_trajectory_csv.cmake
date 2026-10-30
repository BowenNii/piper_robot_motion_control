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

# 旧60秒终点IK/2mm真机接口测试随历史实验归档，不宣称这些模式受当前CLI支持。
# 当前保留生成器只接受输出路径和时长，旧模式必须明确拒绝且不产生文件。
foreach(legacy_mode IN ITEMS --endpoint-joint-target --relative-target)
    set(legacy_path "${root}/unsupported${legacy_mode}.csv")
    execute_process(COMMAND "${IK_TRAJECTORY}" "${legacy_path}" 10 ${legacy_mode}
        RESULT_VARIABLE code OUTPUT_QUIET ERROR_QUIET)
    if(code EQUAL 0 OR EXISTS "${legacy_path}")
        message(FATAL_ERROR "Unsupported legacy mode accepted: ${legacy_mode}")
    endif()
endforeach()
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
