cmake_minimum_required(VERSION 3.16)
project(kpi4 LANGUAGES CXX)

add_subdirectory(core)
add_subdirectory(protocol)

add_library(kpi4_host_input app/byte_source.cpp)
target_compile_features(kpi4_host_input PUBLIC cxx_std_17)
target_include_directories(kpi4_host_input PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/app)

add_executable(kpi4_cli app/main.cpp)
target_compile_features(kpi4_cli PRIVATE cxx_std_17)
target_link_libraries(kpi4_cli PRIVATE kpi4_host_input kpi4_protocol)

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
endif()
