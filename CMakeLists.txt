cmake_minimum_required(VERSION 2.8)

include(cmake/base.cmake)

set(PROJECT_NAME mc_hmc2test_controller)
set(PROJECT_DESCRIPTION "HMC2 test controller")
set(PROJECT_URL "")

setup_project()

#Enable C++11
if(NOT Win32)
  add_definitions(-std=c++0x)
endif()

set(PKG_CONFIG_ADDITIONAL_VARIABLES "${PKG_CONFIG_ADDITIONAL_VARIABLES} libdir")
add_required_dependency(mc_rtc)

add_subdirectory(src)
setup_project_finalize()
