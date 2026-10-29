cmake_minimum_required(VERSION 3.16)
project(kpi4 LANGUAGES CXX)

add_subdirectory(core)
add_subdirectory(protocol)

add_executable(kpi4_cli app/main.cpp)
target_compile_features(kpi4_cli PRIVATE cxx_std_17)
target_link_libraries(kpi4_cli PRIVATE kpi4_protocol)
