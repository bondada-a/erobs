cmake_minimum_required(VERSION 3.8)
project(ur5e_with_gripper_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY config rviz srdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

#if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
 # install(
 #   DIRECTORY launch
 #   DESTINATION share/${PROJECT_NAME}
 #   PATTERN "setup_assistant.launch" EXCLUDE)
#endif()

#install(DIRECTORY config srdf
#  DESTINATION share/${PROJECT_NAME}
#)


#install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
#install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
