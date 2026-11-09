cmake_minimum_required(VERSION 3.16)
project(kpi4 LANGUAGES CXX)

add_subdirectory(core)
add_subdirectory(protocol)
add_subdirectory(mission)
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
    target_link_libraries(kpi4_test_bridge PRIVATE kpi4_host_logging kpi4_state kpi4_mission)
    add_test(NAME python_reader COMMAND ${CMAKE_COMMAND} -E env
        KPI4_BRIDGE=$<TARGET_FILE:kpi4_test_bridge>
        KPI4_CLI=$<TARGET_FILE:kpi4_cli>
        ${Python3_EXECUTABLE} -m unittest discover
        -s ${CMAKE_CURRENT_SOURCE_DIR}/tests/python -p "test_reader.py" -v)

    add_test(NAME python_mission COMMAND ${CMAKE_COMMAND} -E env
        KPI4_BRIDGE=$<TARGET_FILE:kpi4_test_bridge>
        KPI4_CLI=$<TARGET_FILE:kpi4_cli>
        ${Python3_EXECUTABLE} -m unittest discover
        -s ${CMAKE_CURRENT_SOURCE_DIR}/tests/python -p "test_mission.py" -v)

    add_test(NAME python_state COMMAND ${CMAKE_COMMAND} -E env
        KPI4_BRIDGE=$<TARGET_FILE:kpi4_test_bridge>
        KPI4_CLI=$<TARGET_FILE:kpi4_cli>
        ${Python3_EXECUTABLE} -m unittest discover
        -s ${CMAKE_CURRENT_SOURCE_DIR}/tests/python -p "test_state.py" -v)
endif()
