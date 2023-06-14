cmake_minimum_required(VERSION 3.5)
project(racecarx-sim)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-std=c++20)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(gz-cmake3 REQUIRED)

find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

find_package(gz-common5 REQUIRED)
gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)

set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

# Add sources for each plugin to be registered.
add_library(RoadGen SHARED plugins/road_gen/RoadGen.cc)
set_property(TARGET RoadGen PROPERTY CXX_STANDARD 17)
target_link_libraries(RoadGen
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS RoadGen EXPORT RoadGen
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
