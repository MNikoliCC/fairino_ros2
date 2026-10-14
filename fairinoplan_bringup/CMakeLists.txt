cmake_minimum_required(VERSION 3.8)
project(fairinoplan_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config meshes
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/fairino_plan_execute.py
    scripts/fairino_viser_ui.py
    scripts/fairino_servo_benchmark.py
    scripts/fairino_fault_injection_test.py
    scripts/fairino_servo_state_machine_test.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
