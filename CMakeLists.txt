############################################################################
# This file is part of LImA, a Library for Image Acquisition
#
# Copyright (C) : 2009-2017
# European Synchrotron Radiation Facility
# BP 220, Grenoble 38043
# FRANCE
#
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, see <http://www.gnu.org/licenses/>.
############################################################################
# CMakeLists.txt for Pilatus camera

SET(CMAKE_SHARED_LINKER_FLAGS "-Wl,--add-needed")

SET(pilatus_srcs src/PilatusCamera.cpp  src/PilatusInterface.cpp
	 src/PilatusSaving.cpp)
	 
FILE(STRINGS "VERSION" pilatus_vers)
ADD_LIBRARY(limapilatus SHARED ${pilatus_srcs})

#INCLUDES
TARGET_INCLUDE_DIRECTORIES(limapilatus PUBLIC "${CMAKE_SOURCE_DIR}/third-party/CBFLib/include")
TARGET_INCLUDE_DIRECTORIES(limapilatus PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

#LIBRARIES
FIND_LIBRARY(PILATUS_LIB cbf)
IF("${PILATUS_LIB}" STREQUAL "PILATUS_LIB-NOTFOUND")
  MESSAGE(FATAL_ERROR "pilatus : libcbf not found, you can search it in cmake-gui or try to install it if it isn't here.")
ENDIF()


TARGET_LINK_LIBRARIES(limapilatus limacore)
TARGET_LINK_LIBRARIES(limapilatus ${PILATUS_LIB})

SET_TARGET_PROPERTIES(limapilatus PROPERTIES VERSION ${pilatus_vers} SOVERSION "1")

INSTALL(TARGETS limapilatus LIBRARY DESTINATION lib)

IF(COMPILE_SIP)
    SET(NAME "pilatus")
    include(CameraRunSIP)
    INSTALL(FILES python/__init__.py DESTINATION "${PYTHON_SITE_PACKAGES_DIR}/Lima/Pilatus")
ENDIF()
