cmake_minimum_required(VERSION 3.16)
project(kpi4 LANGUAGES CXX)

add_subdirectory(core)
add_subdirectory(protocol)
add_subdirectory(state)

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
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_executable(kpi4_test_bridge tools/test_bridge.cpp)
    target_include_directories(kpi4_test_bridge PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/app)
    target_link_libraries(kpi4_test_bridge PRIVATE kpi4_host_logging)
    add_test(NAME python_reader COMMAND ${CMAKE_COMMAND} -E env
        KPI4_BRIDGE=$<TARGET_FILE:kpi4_test_bridge>
        KPI4_CLI=$<TARGET_FILE:kpi4_cli>
        ${Python3_EXECUTABLE} -m unittest discover
        -s ${CMAKE_CURRENT_SOURCE_DIR}/tests/python -p "test_reader.py" -v)

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

    add_executable(kpi4_mission_upload_test tests/mission_upload_test.cpp)
    target_link_libraries(kpi4_mission_upload_test PRIVATE kpi4_protocol)
    add_test(NAME mission_upload COMMAND kpi4_mission_upload_test)

    add_executable(kpi4_estimated_position_test tests/estimated_position_test.cpp)
    target_link_libraries(kpi4_estimated_position_test PRIVATE kpi4_protocol)
    add_test(NAME estimated_position COMMAND kpi4_estimated_position_test)

    add_executable(kpi4_raw_gps_test tests/raw_gps_test.cpp)
    target_link_libraries(kpi4_raw_gps_test PRIVATE kpi4_protocol)
    add_test(NAME raw_gps COMMAND kpi4_raw_gps_test)

    add_executable(kpi4_state_provider_test tests/state_provider_test.cpp)
    target_link_libraries(kpi4_state_provider_test PRIVATE kpi4_state)
    add_test(NAME state_provider COMMAND kpi4_state_provider_test)

    add_executable(kpi4_state_validator_test tests/state_validator_test.cpp)
    target_link_libraries(kpi4_state_validator_test PRIVATE kpi4_state)
    add_test(NAME state_validator COMMAND kpi4_state_validator_test)

endif()
