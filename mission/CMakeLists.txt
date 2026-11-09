add_library(kpi4_mission STATIC plan_store.cpp)
target_compile_features(kpi4_mission PUBLIC cxx_std_17)
target_include_directories(kpi4_mission PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(kpi4_mission PUBLIC kpi4_protocol)
