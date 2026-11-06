# 离线命令行回归，只在构建目录中新建临时目录，不碰实验归档。
if(NOT DEFINED FK_VALIDATION OR NOT EXISTS "${FK_VALIDATION}")
    message(FATAL_ERROR "FK_VALIDATION must point to the built fk_validation executable")
endif()
string(RANDOM LENGTH 12 ALPHABET 0123456789abcdef suffix)
set(root "${TEST_ROOT}/fk-cli-test-${suffix}")
file(MAKE_DIRECTORY "${root}/logs" "${root}/csv")
set(log "${root}/input.log")
file(WRITE "${log}"
    "(10.0000) can0 2A2#0000000000000000\n"
    "(10.0001) can0 2A3#0000000000000000\n"
    "(10.0002) can0 2A4#0000000000000000\n"
    "(10.0003) can0 2A5#0000000000000000\n"
    "(10.0004) can0 2A6#0000000000000000\n"
    "(10.0005) can0 2A7#0000000000000000\n")
execute_process(COMMAND "${FK_VALIDATION}" "${log}" "${root}/single.csv"
    RESULT_VARIABLE code OUTPUT_VARIABLE output ERROR_VARIABLE error)
if(NOT code EQUAL 0 OR NOT output MATCHES "Valid feedback samples: 1")
    message(FATAL_ERROR "single mode failed: ${output} ${error}")
endif()
file(STRINGS "${root}/single.csv" rows)
list(LENGTH rows count)
if(NOT count EQUAL 2)
    message(FATAL_ERROR "expected header plus one row")
endif()
file(SHA256 "${root}/single.csv" before)
execute_process(COMMAND "${FK_VALIDATION}" "${log}" "${root}/single.csv"
    RESULT_VARIABLE code OUTPUT_QUIET ERROR_QUIET)
file(SHA256 "${root}/single.csv" after)
if(code EQUAL 0 OR NOT before STREQUAL after)
    message(FATAL_ERROR "same path must be rejected without overwrite")
endif()
foreach(i RANGE 1 10)
    if(i LESS 10)
        set(name "pose0${i}")
    else()
        set(name "pose${i}")
    endif()
    configure_file("${log}" "${root}/logs/${name}.log" COPYONLY)
endforeach()
execute_process(COMMAND "${FK_VALIDATION}" --multi "${root}/logs" "${root}/csv"
    RESULT_VARIABLE code OUTPUT_VARIABLE output ERROR_VARIABLE error)
if(NOT code EQUAL 0 OR NOT output MATCHES "Validated all 10 static poses")
    message(FATAL_ERROR "batch failed: ${output} ${error}")
endif()
foreach(i RANGE 1 9)
    if(NOT EXISTS "${root}/csv/pose0${i}.csv")
        message(FATAL_ERROR "missing batch file ${i}")
    endif()
endforeach()
if(NOT EXISTS "${root}/csv/pose10.csv")
    message(FATAL_ERROR "missing pose10")
endif()
execute_process(COMMAND "${FK_VALIDATION}" --multi "${root}/logs"
    RESULT_VARIABLE code OUTPUT_QUIET ERROR_VARIABLE error)
if(code EQUAL 0 OR NOT error MATCHES "Usage:")
    message(FATAL_ERROR "missing CLI argument was not rejected")
endif()
execute_process(COMMAND "${FK_VALIDATION}" "${root}/absent.log" "${root}/absent.csv"
    RESULT_VARIABLE code OUTPUT_QUIET ERROR_QUIET)
if(code EQUAL 0 OR EXISTS "${root}/absent.csv")
    message(FATAL_ERROR "missing log should fail before creating output")
endif()
execute_process(COMMAND "${FK_VALIDATION}" "${log}" "${root}/absent/output.csv"
    RESULT_VARIABLE code OUTPUT_QUIET ERROR_QUIET)
if(code EQUAL 0)
    message(FATAL_ERROR "missing parent directory should fail")
endif()
file(WRITE "${root}/empty.log" "invalid text\n")
execute_process(COMMAND "${FK_VALIDATION}" "${root}/empty.log" "${root}/empty.csv"
    RESULT_VARIABLE code OUTPUT_QUIET ERROR_VARIABLE error)
if(code EQUAL 0 OR NOT error MATCHES "No complete feedback")
    message(FATAL_ERROR "zero-sample log should fail")
endif()
# 半轮旧关节帧 + 半轮新位姿帧不能拼成完整样本，即使跨度小于 2 ms。
file(WRITE "${root}/cross-cycle.log"
    "(10.0045) can0 2A5#000003E8000003E8\n"
    "(10.0046) can0 2A6#000003E8000003E8\n"
    "(10.0047) can0 2A7#000003E8000003E8\n"
    "(10.0050) can0 2A2#0000000000000000\n"
    "(10.0051) can0 2A3#0000000000000000\n"
    "(10.0052) can0 2A4#0000000000000000\n")
execute_process(COMMAND "${FK_VALIDATION}" "${root}/cross-cycle.log" "${root}/cross-cycle.csv"
    RESULT_VARIABLE code OUTPUT_QUIET ERROR_VARIABLE error)
if(code EQUAL 0 OR NOT error MATCHES "No complete feedback")
    message(FATAL_ERROR "cross-cycle mixed sample must be rejected")
endif()
file(STRINGS "${root}/cross-cycle.csv" rows)
list(LENGTH rows count)
if(NOT count EQUAL 1)
    message(FATAL_ERROR "mixed sample must not be written")
endif()

file(READ "${log}" group10)
string(REPLACE "(10." "(11." group11 "${group10}")
string(REPLACE "(10." "(12." group12 "${group10}")
# 不完整轮之后，遇到新的 2A2 应恢复，并且只写入新完整轮。
file(READ "${root}/cross-cycle.log" incomplete)
file(WRITE "${root}/recovered.log" "${incomplete}${group11}")
execute_process(COMMAND "${FK_VALIDATION}" "${root}/recovered.log" "${root}/recovered.csv"
    RESULT_VARIABLE code OUTPUT_VARIABLE output ERROR_VARIABLE error)
if(NOT code EQUAL 0 OR NOT output MATCHES "Valid feedback samples: 1")
    message(FATAL_ERROR "ordered cycle recovery failed: ${output} ${error}")
endif()
file(READ "${root}/single.csv" expected)
file(READ "${root}/recovered.csv" recovered)
# 两个样本仅绝对时间不同；其余列（包括关节角）应完全一致。
string(REPLACE "11.0005" "10.0005" recovered "${recovered}")
if(NOT recovered STREQUAL expected)
    message(FATAL_ERROR "recovery must use only the new complete cycle")
endif()

file(WRITE "${root}/increasing.log" "${group10}${group11}")
execute_process(COMMAND "${FK_VALIDATION}" "${root}/increasing.log" "${root}/increasing.csv"
    RESULT_VARIABLE code OUTPUT_VARIABLE output ERROR_VARIABLE error)
if(NOT code EQUAL 0 OR NOT output MATCHES "Valid feedback samples: 2")
    message(FATAL_ERROR "increasing groups should succeed: ${output} ${error}")
endif()
foreach(kind IN ITEMS reversed duplicate)
    if(kind STREQUAL "reversed")
        set(groups "${group10}${group12}${group11}")
        set(expected_rows 3)
    else()
        set(groups "${group10}${group10}")
        set(expected_rows 2)
    endif()
    file(WRITE "${root}/${kind}.log" "${groups}")
    execute_process(COMMAND "${FK_VALIDATION}" "${root}/${kind}.log" "${root}/${kind}.csv"
        RESULT_VARIABLE code OUTPUT_VARIABLE output ERROR_VARIABLE error)
    if(code EQUAL 0 OR NOT error MATCHES "timestamps must be strictly increasing"
            OR output MATCHES "CSV saved:")
        message(FATAL_ERROR "${kind} group timestamps must fail: ${output} ${error}")
    endif()
    file(STRINGS "${root}/${kind}.csv" rows)
    list(LENGTH rows count)
    if(NOT count EQUAL expected_rows)
        message(FATAL_ERROR "${kind} sample must be rejected before writing")
    endif()
endforeach()
message(STATUS "FK CLI integration PASS")
