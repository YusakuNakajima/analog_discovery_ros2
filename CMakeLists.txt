cmake_minimum_required(VERSION 3.8)
project(analog_discovery_ros2)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ScopeCapture.msg"
  DEPENDENCIES std_msgs
)

# For testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_scope
    test/test_scope.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_mock_scope
    test/test_mock_scope.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_node
    test/test_node.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_selftest
    test/test_selftest.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

#-- for Python
# The driver library is its own module (analog_discovery) because rosidl already
# installs a Python package named ${PROJECT_NAME} for the messages.
ament_python_install_package(analog_discovery)

install(PROGRAMS
  scripts/analog_discovery_node.py
  scripts/ad_probe.py
  scripts/ad_selftest.py
  DESTINATION lib/${PROJECT_NAME}
)
#--- end Python

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
