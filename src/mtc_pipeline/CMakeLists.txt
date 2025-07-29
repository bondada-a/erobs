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
  src/tool_exchange_stages.cpp    # <-- Added this line
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
add_executable(mtc_tutorial src/mtc_tutorial.cpp)
add_executable(mtc_simple  src/mtc_simple.cpp)
add_executable(mtc_complex  src/mtc_complex.cpp)
add_executable(mtc_pickplace  src/mtc_pickplace.cpp)
add_executable(mtc_toolexchange  src/mtc_toolexchange.cpp)
add_executable(mtc_orchestrator  src/mtc_orchestrator.cpp)
# (remove add_executable(pick_place_stages ...) since it's now a library)

# === Link executables to dependencies ===
ament_target_dependencies(mtc_tutorial moveit_task_constructor_core rclcpp)
ament_target_dependencies(mtc_simple moveit_task_constructor_core rclcpp ur_msgs)
ament_target_dependencies(mtc_complex moveit_task_constructor_core rclcpp ur_msgs)
ament_target_dependencies(mtc_pickplace moveit_task_constructor_core rclcpp ur_msgs)
ament_target_dependencies(mtc_toolexchange moveit_task_constructor_core rclcpp ur_msgs)
ament_target_dependencies(mtc_orchestrator moveit_task_constructor_core rclcpp ur_msgs)

target_link_libraries(mtc_pickplace nlohmann_json::nlohmann_json)
target_link_libraries(mtc_toolexchange nlohmann_json::nlohmann_json)
target_link_libraries(mtc_orchestrator nlohmann_json::nlohmann_json)

# === Link mtc_pipeline_core library to relevant executables ===
target_link_libraries(mtc_orchestrator mtc_pipeline_core)
target_link_libraries(mtc_toolexchange mtc_pipeline_core)   # <-- Add this if mtc_toolexchange uses tool exchange logic
# target_link_libraries(mtc_pickplace mtc_pipeline_core)    # Uncomment if needed

target_include_directories(mtc_tutorial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(mtc_orchestrator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(mtc_pickplace PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(mtc_tutorial PUBLIC c_std_99 cxx_std_17)
target_compile_features(mtc_simple PUBLIC c_std_99 cxx_std_17)
target_compile_features(mtc_complex PUBLIC c_std_99 cxx_std_17)
target_compile_features(mtc_pickplace PUBLIC c_std_99 cxx_std_17)
target_compile_features(mtc_toolexchange PUBLIC c_std_99 cxx_std_17)
target_compile_features(mtc_orchestrator PUBLIC c_std_99 cxx_std_17)

install(TARGETS
  mtc_tutorial
  mtc_simple
  mtc_complex
  mtc_pickplace
  mtc_toolexchange
  mtc_orchestrator
  mtc_pipeline_core
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
