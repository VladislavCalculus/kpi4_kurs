add_library(kpi4_core INTERFACE)
target_compile_features(kpi4_core INTERFACE cxx_std_17)
target_include_directories(kpi4_core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
