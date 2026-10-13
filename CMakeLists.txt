cmake_minimum_required(VERSION 3.8)
project(socnav_gazebo_reproducer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Configure Qt5 for the GUI Plugin
set(CMAKE_AUTOMOC ON)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(include)
include_directories(SYSTEM
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# --------------------------------------------------------
# 2. WorldPlugin SocNavPlugin
# --------------------------------------------------------
add_library(SocNavPlugin SHARED src/SocNavPlugin.cpp)
target_include_directories(SocNavPlugin PUBLIC include)
ament_target_dependencies(SocNavPlugin 
  rclcpp 
  gazebo 
  gazebo_ros 
)
target_link_libraries(SocNavPlugin 
  nlohmann_json::nlohmann_json 
  ${GAZEBO_LIBRARIES}
)

# --------------------------------------------------------
# 3. GUI Plugin AutoRecordGuiPlugin
# --------------------------------------------------------
add_library(AutoRecordGuiPlugin SHARED src/AutoRecordGuiPlugin.cpp)
target_include_directories(AutoRecordGuiPlugin PUBLIC 
  include 
  ${Qt5Widgets_INCLUDE_DIRS}
)
ament_target_dependencies(AutoRecordGuiPlugin 
  gazebo 
)
target_link_libraries(AutoRecordGuiPlugin 
  ${GAZEBO_LIBRARIES}
  Qt5::Widgets
)

# --------------------------------------------------------
# 4. Export and Install both plugins
# --------------------------------------------------------
ament_export_libraries(
  SocNavPlugin
  AutoRecordGuiPlugin
)

install(TARGETS 
  SocNavPlugin 
  AutoRecordGuiPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
        DESTINATION include
)

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY worlds/
        DESTINATION share/${PROJECT_NAME}/worlds
)

install(DIRECTORY meshes/
        DESTINATION share/${PROJECT_NAME}/meshes
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_camera_framing test/test_camera_framing.py)
endif()

# install(DIRECTORY ./
#         DESTINATION share/${PROJECT_NAME}
#         FILES_MATCHING PATTERN "*.world"
# )
ament_export_dependencies(rclcpp gazebo_ros gazebo)
ament_package()
