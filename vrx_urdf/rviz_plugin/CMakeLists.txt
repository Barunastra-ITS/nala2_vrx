cmake_minimum_required(VERSION 3.8)
project(rviz_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif() 

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)
set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_compile_definitions(QT_NO_KEYWORDS)

set(CMAKE_AUTOUIC_SEARCH_PATHS ui)
qt5_wrap_ui(UIC_FILES
  ui/WaypointNavigation.ui
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Models
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models)

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/rviz_plugin/waypoint_nav_tool.hpp
  include/rviz_plugin/waypoint_nav_frame.hpp
  include/rviz_plugin/json.hpp
  src/waypoint_nav_frame.cpp
  src/waypoint_nav_tool.cpp
  ${UIC_FILES}
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
register_rviz_ogre_media_exports(DIRECTORIES "media")

ament_auto_package(
  INSTALL_TO_SHARE
  icons
  media
  launch
  rviz
)
