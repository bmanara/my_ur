cmake_minimum_required(VERSION 3.8)
project(my_ur)

find_package(ament_cmake REQUIRED)

option(
  UR_SIM_INTEGRATION_TESTS
  "Run my_ur integration tests"
  OFF
)

# install(
#   PROGRAMS
#     scripts/mimic_joint_state_publisher.py
#     DESTINATION lib/${PROJECT_NAME}
# )

install(
  DIRECTORY config launch meshes rviz urdf test
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(ur_urdf_xacro test/test_ur_urdf_xacro.py)
  ament_add_pytest_test(view_ur_launch test/test_view_ur_launch.py)

  if (${UR_SIM_INTEGRATION_TESTS})
    add_launch_test(test/test_gz.py
      TIMEOUT
        180
    )
  endif()
endif()

ament_package()
