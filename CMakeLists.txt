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


TARGET_LINK_LIBRARIES(limapilatus limacore)
TARGET_LINK_LIBRARIES(limapilatus ${PILATUS_LIB})

SET_TARGET_PROPERTIES(limapilatus PROPERTIES VERSION ${pilatus_vers} SOVERSION "1")

INSTALL(TARGETS limapilatus LIBRARY DESTINATION lib)

IF(COMPILE_SIP)
    SET(NAME "pilatus")
    SET(INCLUDES)
    FILE(GLOB sipfiles RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/sip" "${CMAKE_CURRENT_SOURCE_DIR}/sip/*.sip")
    FOREACH(sipfile ${sipfiles})
        SET(INCLUDES 
        "${INCLUDES} 
        %Include ${sipfile}")
    ENDFOREACH()

    CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/sip/limamodules.sip.in sip/limapilatus.sip)
    SET(SIP_CONCAT_PARTS 1)
    SET(SIP_INCLUDES ${SIP_INCLUDES}
        "${CMAKE_SOURCE_DIR}/third-party/Processlib/sip"
        "${CMAKE_BINARY_DIR}/sip/core"
        "${CMAKE_SOURCE_DIR}/third-party/Processlib/tasks/sip"
        "${CMAKE_SOURCE_DIR}/common/sip"
        "${CMAKE_SOURCE_DIR}/hardware/sip"
        "${CMAKE_SOURCE_DIR}/control/sip"
        "${CMAKE_SOURCE_DIR}/control/software_operation/sip"
        "${CMAKE_CURRENT_SOURCE_DIR}/sip")
    set(SIP_DISABLE_FEATURES WITH_CONFIG)
    set(SIP_CHECK_EXC ON)
    ADD_SIP_PYTHON_MODULE(limapilatus ${CMAKE_CURRENT_BINARY_DIR}/sip/limapilatus.sip)
    TARGET_INCLUDE_DIRECTORIES(python_module_limapilatus PRIVATE
                               ${PYTHON_INCLUDE_DIRS}
                               "${CMAKE_SOURCE_DIR}/sip"
                               "${CMAKE_SOURCE_DIR}/sip/core")
    TARGET_LINK_LIBRARIES(python_module_limapilatus limapilatus)

    INSTALL(FILES python/__init__.py DESTINATION "${PYTHON_SITE_PACKAGES_DIR}/Lima/Pilatus")
ENDIF()
