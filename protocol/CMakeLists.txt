add_library(kpi4_protocol STATIC frame_reader.cpp)
target_compile_features(kpi4_protocol PUBLIC cxx_std_17)
target_include_directories(kpi4_protocol PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/third_party/mavlink)
target_link_libraries(kpi4_protocol PUBLIC kpi4_core)
