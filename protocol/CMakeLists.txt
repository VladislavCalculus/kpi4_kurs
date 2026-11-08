add_library(kpi4_protocol STATIC frame_reader.cpp heartbeat.cpp mission_transfer.cpp
    mission_normalizer.cpp mission_command_validator.cpp estimated_position.cpp
    raw_gps.cpp mission_upload.cpp)
target_compile_features(kpi4_protocol PUBLIC cxx_std_17)
target_include_directories(kpi4_protocol PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/third_party/mavlink)
target_link_libraries(kpi4_protocol PUBLIC kpi4_core)
