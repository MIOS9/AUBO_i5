cmake_minimum_required(VERSION 2.8.3)
project(aubo_i5_moveit_config)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS
  scripts/moveit_attached_object_demo.py
  scripts/moveit_cartesian_demo.py
  scripts/probot_demo.py
  scripts/moveit_circle_demo.py
  scripts/moveit_fk_demo.py
  scripts/shagou9000.py
  scripts/moveit_ik_demo.py                 
  scripts/test_move.py
  scripts/moveit_revise_trejectory_demo.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
