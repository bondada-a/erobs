cmake_minimum_required(VERSION 3.8)
project(mtc_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ur_msgs REQUIRED)

# === Create a library for shared logic ===
add_library(mtc_pipeline_core
  src/pick_place_stages.cpp
  src/tool_exchange_stages.cpp
  src/moveto_stages.cpp
  src/end_effector_stages.cpp
)
ament_target_dependencies(mtc_pipeline_core
  moveit_task_constructor_core
  rclcpp
  ur_msgs
  nlohmann_json
)
target_include_directories(mtc_pipeline_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(mtc_pipeline_core PUBLIC c_std_99 cxx_std_17)

# === Executables ===
add_executable(mtc_orchestrator  src/mtc_orchestrator.cpp)

# === Link executables to dependencies ===
ament_target_dependencies(mtc_orchestrator moveit_task_constructor_core rclcpp ur_msgs)

target_link_libraries(mtc_orchestrator nlohmann_json::nlohmann_json)

# === Link mtc_pipeline_core library to relevant executables ===
target_link_libraries(mtc_orchestrator mtc_pipeline_core)

# === Include directories ===
target_include_directories(mtc_orchestrator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# === C++ Standards ===
target_compile_features(mtc_orchestrator PUBLIC c_std_99 cxx_std_17)

# === Install targets ===
install(TARGETS
  mtc_orchestrator
  mtc_pipeline_core
  DESTINATION lib/${PROJECT_NAME}
)

# === Install launch files ===
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# === Testing (optional) ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
