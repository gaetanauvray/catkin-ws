include_directories(
  include/
)

# MFBot Hardware Interface
add_library(mfbot_hw_interface
  src/mfbot_hw_interface.cpp
)
target_link_libraries(mfbot_hw_interface
  generic_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(mfbot_hw_main src/mfbot_hw_main.cpp)
target_link_libraries(mfbot_hw_main
  mfbot_hw_interface
  generic_hw_control_loop
  ${catkin_LIBRARIES}
)

## Install ------------------------------------------------------------

# Install libraries
install(TARGETS
    mfbot_hw_interface
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS
    mfbot_hw_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/mfbot_control/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
