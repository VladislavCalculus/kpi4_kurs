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
endif()
