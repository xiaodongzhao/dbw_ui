cmake_minimum_required(VERSION 2.8.3)
project(dbw_ui)


find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
)

catkin_python_setup()

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
catkin_install_python(PROGRAMS scripts/dbw_ui
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY resource/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resource)
install(FILES plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})