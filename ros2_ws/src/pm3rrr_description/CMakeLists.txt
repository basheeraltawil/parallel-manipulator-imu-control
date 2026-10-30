cmake_minimum_required(VERSION 3.8)
project(pm3rrr_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME})

# Lets Gazebo Classic resolve package://pm3rrr_description/... mesh URIs
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf test/test_urdf.py
    ENV PM3RRR_DESCRIPTION_SHARE=${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
