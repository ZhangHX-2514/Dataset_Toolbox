execute_process(COMMAND "/home/dai/workspace/Dataset_Toolbox/build/dvs_calibration_gui/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/dai/workspace/Dataset_Toolbox/build/dvs_calibration_gui/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
