cmake_minimum_required(VERSION 3.8)
project(robot_data_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

file(GLOB INTERFACE_SOURCES_ABS "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg" "${CMAKE_CURRENT_SOURCE_DIR}/srv/*.srv")
set(INTERFACE_SOURCES "")
foreach(f ${INTERFACE_SOURCES_ABS})
    string(REPLACE "${CMAKE_CURRENT_SOURCE_DIR}/" "" f_rel ${f})
    list(APPEND INTERFACE_SOURCES ${f_rel})
endforeach()
# 此时 INTERFACE_SOURCES 包含 "msg/xxx.msg" 和 "srv/xxx.srv"
rosidl_generate_interfaces(${PROJECT_NAME} 
	${INTERFACE_SOURCES}
	DEPENDENCIES std_msgs
	)

ament_package()
