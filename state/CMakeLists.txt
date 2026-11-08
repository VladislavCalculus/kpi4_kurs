add_library(kpi4_state STATIC mavlink_provider.cpp simulated_provider.cpp
    validator.cpp)
target_compile_features(kpi4_state PUBLIC cxx_std_17)
target_include_directories(kpi4_state PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(kpi4_state PUBLIC kpi4_protocol)
