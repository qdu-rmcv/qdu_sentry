cmake_minimum_required(VERSION 3.10)
project(armor_detector)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim::ArmorDetectorNode
  EXECUTABLE armor_detector_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)
  ament_add_gtest(test_node_startup test/test_node_startup.cpp)
  target_link_libraries(test_node_startup ${PROJECT_NAME})

  ament_add_gtest(test_number_cls test/test_number_cls.cpp)
  target_link_libraries(test_number_cls ${PROJECT_NAME})

endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  model
)


# # 链接 OpenCV 和 ROS 2 的依赖
# ament_target_dependencies(armor_detector_node
#   rclcpp
#   auto_aim_interfaces
#   OpenCV
# )

# # 安装
# install(TARGETS
#   armor_detector_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# # 生成消息
# ament_package()

# cmake_minimum_required(VERSION 3.10)
# project(armor_detector)

# ## Use C++14
# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
# ## enforcing cleaner code.
# add_definitions(-Wall -Werror)

# ## Export compile commands for clangd
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# #######################
# ## Find dependencies ##
# #######################

# # Find ament_cmake_auto and OpenCV
# find_package(ament_cmake_auto REQUIRED)
# find_package(OpenCV REQUIRED)
# find_package(auto_aim_interfaces REQUIRED)

# # Automatically find build dependencies
# ament_auto_find_build_dependencies()

# ###########
# ## Build ##
# ###########

# # Define the library for the node
# ament_auto_add_library(${PROJECT_NAME} SHARED
#   DIRECTORY src
# )

# # Include OpenCV headers
# target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})

# # Link OpenCV libraries
# target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

# # Register the node for components
# rclcpp_components_register_node(${PROJECT_NAME}
#   PLUGIN rm_auto_aim::ArmorDetectorNode
#   EXECUTABLE armor_detector_node
# )

# #############
# ## Testing ##
# #############

# if(BUILD_TESTING)
#   # Find lint dependencies and exclude some checks
#   find_package(ament_lint_auto REQUIRED)
#   list(APPEND AMENT_LINT_AUTO_EXCLUDE
#     ament_cmake_copyright
#     ament_cmake_uncrustify
#     ament_cmake_cpplint
#   )
#   ament_lint_auto_find_test_dependencies()

#   # Find gtest and add unit tests
#   find_package(ament_cmake_gtest)
  
#   # Example test for node startup
#   ament_add_gtest(test_node_startup test/test_node_startup.cpp)
#   target_link_libraries(test_node_startup ${PROJECT_NAME})

#   # Example test for number class
#   ament_add_gtest(test_number_cls test/test_number_cls.cpp)
#   target_link_libraries(test_number_cls ${PROJECT_NAME})

# endif()

# #############
# ## Install ##
# #############

# # Define install targets and installation directories
# ament_auto_package(
#   INSTALL_TO_SHARE
#   model
# )

# # Optionally, install any other resources or models if needed
# install(DIRECTORY models/
#   DESTINATION share/${PROJECT_NAME}/models
# )

# # Install executables
# install(TARGETS
#   armor_detector_node
#   DESTINATION lib/${PROJECT_NAME}
# )
