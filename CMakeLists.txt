###########################################################################
# This file is part of LImA, a Library for Image Acquisition
#
#  Copyright (C) : 2009-2017
#  European Synchrotron Radiation Facility
#  CS40220 38043 Grenoble Cedex 9 
#  FRANCE
# 
#  Contact: lima@esrf.fr
# 
#  This is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 3 of the License, or
#  (at your option) any later version.
# 
#  This software is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# 
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, see <http://www.gnu.org/licenses/>.
############################################################################
cmake_minimum_required(VERSION 3.0)
#--------------------------------------------------------------------------------
# cmake/ directory contains tools 
#--------------------------------------------------------------------------------
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/../../cmake ${CMAKE_MODULE_PATH})
message("${CMAKE_MODULE_PATH}")
include(LimaTools)

find_package(Threads REQUIRED)

set(NAME "pilatus")

set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--add-needed")

set(${NAME}_srcs src/PilatusCamera.cpp  src/PilatusInterface.cpp
	 src/PilatusSaving.cpp)
	 
add_library(lima${NAME} SHARED ${${NAME}_srcs})

#INCLUDES
target_include_directories(lima${NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_include_directories(lima${NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../../common/include")
target_include_directories(lima${NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../../hardware/include")


#LIBRARIES
find_library(PILATUS_CBF_LIB cbf)
message("${PILATUS_CBF_LIB}")
if("${PILATUS_CBF_LIB}" STREQUAL "PILATUS_CBF_LIB-NOTFOUND")
  message(FATAL_ERROR "${NAME} : libcbf not found, you can search it in cmake-gui or try to install it if it isn't here.")
endif()

find_library(LIB_LIMACORE limacore  "${CMAKE_CURRENT_SOURCE_DIR}/../../install/lib")
target_link_libraries(lima${NAME} ${LIB_LIMACORE})
target_link_libraries(lima${NAME} ${PILATUS_CBF_LIB})

limatools_set_library_soversion(lima${NAME} "VERSION")

# Compile python wrapping code generated using SIP
IF(DEFINED ENV{LIMA_ENABLE_PYTHON})
    set(LIMA_ENABLE_PYTHON "$ENV{LIMA_ENABLE_PYTHON}" CACHE BOOL "compile python modules?" FORCE)
else()
    option(LIMA_ENABLE_PYTHON "compile python modules?" OFF)
endif()

#--------------------------------------------------------------------------------
# Enable python binding code compilation using sip generator
#--------------------------------------------------------------------------------
if(LIMA_ENABLE_PYTHON)
    find_package(PythonInterp 2.7)
    find_package(PythonLibs 2.7)
    if(${PYTHONINTERP_FOUND})
      # python site-packages folder
      execute_process(
	      COMMAND ${PYTHON_EXECUTABLE} -c "from distutils.sysconfig import get_python_lib; print (get_python_lib())"
	OUTPUT_VARIABLE _PYTHON_SITE_PACKAGES_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
	else()
		message(FATAL_ERROR "No python found, please install or disable LIMA_ENABLE_PYTHON")
    endif()
    find_package(SIP)
	if(${SIP_FOUND})
		include(SIPMacros)
	else()
		message(FATAL_ERROR "python SIP not installed, please install or disable LIMA_ENABLE_PYTHON")
	endif()

    set(PYTHON_SITE_PACKAGES_DIR ${_PYTHON_SITE_PACKAGES_DIR} CACHE PATH "where should python modules be installed?")

    if(WIN32)
        set(NUMPY_INCLUDE_DIR "${_PYTHON_SITE_PACKAGES_DIR}/numpy/core/include" CACHE PATH "Path to NumPy include folder")
        set(NUMPY_LIB_DIR "${_PYTHON_SITE_PACKAGES_DIR}/numpy/core/lib" CACHE PATH "Path to NumPy lib folder")
        include_directories(${NUMPY_INCLUDE_DIR})
        link_directories(${NUMPY_LIB_DIR})
    endif()
    if(!${SIP_FOUND})
        message(FATAL_ERROR "sip executable not found, cannot build python extensions")
    endif()
    if(WIN32)
        set(SIP_TAGS WIN32_PLATFORM)
    elseif(UNIX)
        set(SIP_TAGS POSIX_PLATFORM)
    endif(WIN32)
    set(SIP_EXTRA_OPTIONS -e -g)

    
  
    limatools_run_sip_for_camera(${NAME})
    install(FILES python/__init__.py DESTINATION "${PYTHON_SITE_PACKAGES_DIR}/Lima/Pilatus")
endif()


install(TARGETS lima${NAME} LIBRARY DESTINATION lib)

