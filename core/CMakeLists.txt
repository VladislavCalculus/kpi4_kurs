add_library(kpi4_core guidance.cpp)
target_compile_features(kpi4_core PUBLIC cxx_std_17)
target_include_directories(kpi4_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
