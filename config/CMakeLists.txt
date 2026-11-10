add_library(kpi4_config STATIC host_config.cpp)
target_compile_features(kpi4_config PUBLIC cxx_std_17)
target_include_directories(kpi4_config PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(kpi4_config PUBLIC kpi4_state)
