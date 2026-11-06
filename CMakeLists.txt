cmake_minimum_required(VERSION 3.16)
project(kpi4 LANGUAGES CXX)

add_subdirectory(core)
add_subdirectory(protocol)

add_library(kpi4_host_input app/byte_source.cpp)
target_compile_features(kpi4_host_input PUBLIC cxx_std_17)
target_include_directories(kpi4_host_input PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/app)

add_library(kpi4_host_logging app/frame_log.cpp)
target_compile_features(kpi4_host_logging PUBLIC cxx_std_17)
target_include_directories(kpi4_host_logging PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/app)
target_link_libraries(kpi4_host_logging PUBLIC kpi4_protocol)

add_executable(kpi4_cli app/main.cpp)
target_compile_features(kpi4_cli PRIVATE cxx_std_17)
target_link_libraries(kpi4_cli PRIVATE kpi4_host_input kpi4_host_logging)

include(CTest)
if(BUILD_TESTING)
    add_executable(kpi4_byte_source_test tests/byte_source_test.cpp)
    target_link_libraries(kpi4_byte_source_test PRIVATE kpi4_host_input)
    add_test(NAME byte_source COMMAND kpi4_byte_source_test)

    add_test(NAME cli_file_input
        COMMAND kpi4_cli --input ${CMAKE_CURRENT_SOURCE_DIR}/fixtures/heartbeat_v2.bin --chunk-size 5)
    set_tests_properties(cli_file_input PROPERTIES PASS_REGULAR_EXPRESSION "bytes=21 chunks=")

    add_executable(kpi4_frame_reader_test tests/frame_reader_test.cpp)
    target_link_libraries(kpi4_frame_reader_test PRIVATE kpi4_protocol)
    target_compile_definitions(kpi4_frame_reader_test PRIVATE
        KPI4_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
    add_test(NAME frame_reader COMMAND kpi4_frame_reader_test)

    add_executable(kpi4_frame_reader_chunks_test tests/frame_reader_chunks_test.cpp)
    target_link_libraries(kpi4_frame_reader_chunks_test PRIVATE
        kpi4_host_input kpi4_protocol)
    target_compile_definitions(kpi4_frame_reader_chunks_test PRIVATE
        KPI4_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
    add_test(NAME frame_reader_chunks COMMAND kpi4_frame_reader_chunks_test)

    add_executable(kpi4_frame_reader_recovery_test tests/frame_reader_recovery_test.cpp)
    target_link_libraries(kpi4_frame_reader_recovery_test PRIVATE kpi4_protocol)
    target_compile_definitions(kpi4_frame_reader_recovery_test PRIVATE
        KPI4_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
    add_test(NAME frame_reader_recovery COMMAND kpi4_frame_reader_recovery_test)

    add_executable(kpi4_frame_reader_channels_test tests/frame_reader_channels_test.cpp)
    target_link_libraries(kpi4_frame_reader_channels_test PRIVATE kpi4_protocol)
    target_compile_definitions(kpi4_frame_reader_channels_test PRIVATE
        KPI4_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
    add_test(NAME frame_reader_channels COMMAND kpi4_frame_reader_channels_test)

    add_executable(kpi4_heartbeat_test tests/heartbeat_test.cpp)
    target_link_libraries(kpi4_heartbeat_test PRIVATE kpi4_protocol)
    target_compile_definitions(kpi4_heartbeat_test PRIVATE
        KPI4_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
    add_test(NAME heartbeat COMMAND kpi4_heartbeat_test)

    add_executable(kpi4_mission_transfer_test tests/mission_transfer_test.cpp)
    target_link_libraries(kpi4_mission_transfer_test PRIVATE kpi4_protocol)
    add_test(NAME mission_transfer COMMAND kpi4_mission_transfer_test)

    add_executable(kpi4_mission_normalizer_test tests/mission_normalizer_test.cpp)
    target_link_libraries(kpi4_mission_normalizer_test PRIVATE kpi4_protocol)
    add_test(NAME mission_normalizer COMMAND kpi4_mission_normalizer_test)

    add_executable(kpi4_mission_command_validator_test
        tests/mission_command_validator_test.cpp)
    target_link_libraries(kpi4_mission_command_validator_test PRIVATE kpi4_protocol)
    add_test(NAME mission_command_validator
        COMMAND kpi4_mission_command_validator_test)

    add_executable(kpi4_estimated_position_test tests/estimated_position_test.cpp)
    target_link_libraries(kpi4_estimated_position_test PRIVATE kpi4_protocol)
    add_test(NAME estimated_position COMMAND kpi4_estimated_position_test)

    add_executable(kpi4_raw_gps_test tests/raw_gps_test.cpp)
    target_link_libraries(kpi4_raw_gps_test PRIVATE kpi4_protocol)
    add_test(NAME raw_gps COMMAND kpi4_raw_gps_test)

    add_test(NAME cli_heartbeat_gcs
        COMMAND kpi4_cli --input ${CMAKE_CURRENT_SOURCE_DIR}/fixtures/heartbeat_gcs_v2.bin)
    set_tests_properties(cli_heartbeat_gcs PROPERTIES
        PASS_REGULAR_EXPRESSION "heartbeat channel=unspecified sysid=42 compid=190 type=GCS\\(6\\) autopilot=INVALID\\(8\\) status=STANDBY\\(3\\)")

    add_test(NAME cli_heartbeat_unknown
        COMMAND kpi4_cli --input ${CMAKE_CURRENT_SOURCE_DIR}/fixtures/heartbeat_unknown_v2.bin)
    set_tests_properties(cli_heartbeat_unknown PROPERTIES
        PASS_REGULAR_EXPRESSION "heartbeat channel=unspecified sysid=99 compid=2 type=UNKNOWN\\(255\\) autopilot=UNKNOWN\\(255\\) status=UNKNOWN\\(255\\)")

    add_test(NAME cli_bad_crc
        COMMAND kpi4_cli --input ${CMAKE_CURRENT_SOURCE_DIR}/fixtures/heartbeat_v2_bad_crc.bin)
    set_tests_properties(cli_bad_crc PROPERTIES
        PASS_REGULAR_EXPRESSION "accepted=0 rejected=1 incomplete=0")

    add_test(NAME cli_truncated
        COMMAND kpi4_cli --input ${CMAKE_CURRENT_SOURCE_DIR}/fixtures/heartbeat_v2_truncated.bin)
    set_tests_properties(cli_truncated PROPERTIES
        PASS_REGULAR_EXPRESSION "accepted=0 rejected=0 incomplete=1")

    add_test(NAME cli_recovery
        COMMAND kpi4_cli --input ${CMAKE_CURRENT_SOURCE_DIR}/fixtures/recovery_stream.bin)
    set_tests_properties(cli_recovery PROPERTIES
        PASS_REGULAR_EXPRESSION "accepted=2 rejected=2 incomplete=0 bad_crc=2 bad_signature=0 invalid_header=0")

    add_executable(kpi4_frame_log_test tests/frame_log_test.cpp)
    target_link_libraries(kpi4_frame_log_test PRIVATE kpi4_host_logging)
    add_test(NAME frame_log COMMAND kpi4_frame_log_test)

    add_test(NAME cli_metadata_v1
        COMMAND kpi4_cli --input ${CMAKE_CURRENT_SOURCE_DIR}/fixtures/heartbeat_v1.bin)
    set_tests_properties(cli_metadata_v1 PROPERTIES
        PASS_REGULAR_EXPRESSION "frame time=.*Z mavlink=1 sysid=1 compid=1 msgid=0 seq=0 len=9 signed=no")

    add_test(NAME cli_metadata_v2
        COMMAND kpi4_cli --input ${CMAKE_CURRENT_SOURCE_DIR}/fixtures/heartbeat_v2.bin)
    set_tests_properties(cli_metadata_v2 PROPERTIES
        PASS_REGULAR_EXPRESSION "frame time=.*Z mavlink=2 sysid=1 compid=1 msgid=0 seq=0 len=9 signed=no")
endif()
