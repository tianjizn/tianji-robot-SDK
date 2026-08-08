cmake_minimum_required(VERSION 3.8)
project(test_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

if(WIN32)
    add_compile_definitions(CMPL_WIN)
else()
    add_compile_definitions(CMPL_LIN)   # 注意是 LIN，不是 LINUX
endif()

add_compile_definitions(L1_SDK_EXPORTS)   # 如果 SDK 需要

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_data_interfaces REQUIRED)


file(GLOB_RECURSE SUB_DIRS LIST_DIRECTORIES true "include/*")
foreach(dir ${SUB_DIRS})
    if(IS_DIRECTORY ${dir})
        message(STATUS "Adding include dir: ${dir}")
        include_directories(${dir})
    endif()
endforeach()

# 库路径
set(LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/")
link_directories(${LIB_DIR})

# 查找 GentoSDK（Linux 下名称不变）
find_library(GENTO_LIB NAMES GentoSDK PATHS ${LIB_DIR} NO_DEFAULT_PATH)
if(NOT GENTO_LIB)
    message(FATAL_ERROR "找不到 GentoSDK 库")
endif()
message(STATUS "找到 GentoSDK: ${GENTO_LIB}")

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

ament_package()

#add_executable(${EXAMPLE_NAME} ${EXAMPLE_SOURCE})
#target_link_libraries(${EXAMPLE_NAME} ${GENTO_LIB} ws2_32 winmm)

add_executable(example_basic_pd_user src/example_basic_pd_user.cpp src/basic_pd_user.cpp)
#target_link_libraries(example_basic_pd_user ${GENTO_LIB})
target_include_directories(example_basic_pd_user PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(example_basic_pd_user PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(example_basic_pd_user rclcpp robot_data_interfaces)

add_executable(example_basic_pd_orin src/example_basic_pd_orin.cpp src/basic_orin.cpp)
target_link_libraries(example_basic_pd_orin ${GENTO_LIB})
target_include_directories(example_basic_pd_orin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(example_basic_pd_orin PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(example_basic_pd_orin rclcpp robot_data_interfaces)


install(TARGETS
  example_basic_pd_user
  example_basic_pd_orin
  DESTINATION lib/${PROJECT_NAME}
)