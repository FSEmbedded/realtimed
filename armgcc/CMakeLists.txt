# CROSS COMPILER SETTING
SET(CMAKE_SYSTEM_NAME Generic)
CMAKE_MINIMUM_REQUIRED (VERSION 3.10.0)

# THE VERSION NUMBER
SET (MCUXPRESSO_CMAKE_FORMAT_MAJOR_VERSION 2)
SET (MCUXPRESSO_CMAKE_FORMAT_MINOR_VERSION 0)

#APP VERSION
SET(RDAEMON_VERSION "0.1")

include(ide_overrides.cmake OPTIONAL)

if(CMAKE_SCRIPT_MODE_FILE)
  message("${MCUXPRESSO_CMAKE_FORMAT_MAJOR_VERSION}")
  return()
endif()


# ENABLE ASM
ENABLE_LANGUAGE(ASM)

SET(CMAKE_STATIC_LIBRARY_PREFIX)
SET(CMAKE_STATIC_LIBRARY_SUFFIX)

SET(CMAKE_EXECUTABLE_LIBRARY_PREFIX)
SET(CMAKE_EXECUTABLE_LIBRARY_SUFFIX)

# CURRENT DIRECTORY
SET(ProjDirPath ${CMAKE_CURRENT_SOURCE_DIR})

SET(EXECUTABLE_OUTPUT_PATH ${ProjDirPath}/${CMAKE_BUILD_TYPE})
SET(LIBRARY_OUTPUT_PATH ${ProjDirPath}/${CMAKE_BUILD_TYPE})


project(realtimed)

set(MCUX_BUILD_TYPES debug release)

set(MCUX_SDK_PROJECT_NAME realtimed)

if (NOT DEFINED SdkRootDirPath)
    SET(SdkRootDirPath ${ProjDirPath}/../../__repo__)
endif()

include(${ProjDirPath}/flags.cmake)

include(${ProjDirPath}/config.cmake)

add_executable(${MCUX_SDK_PROJECT_NAME} 
"${ProjDirPath}/../FreeRTOSConfig.h"
"${ProjDirPath}/../rpmsg_config.h"
"${ProjDirPath}/../realtimed.c"
"${ProjDirPath}/../realtimed.h"
"${ProjDirPath}/../rsc_table.c"
"${ProjDirPath}/../rsc_table.h"
"${ProjDirPath}/../remoteproc.h"
"${ProjDirPath}/../srtm_config.h"
"${ProjDirPath}/../board/clock_config.c"
"${ProjDirPath}/../board/board.c"
"${ProjDirPath}/../board/board_uart.c"
"${ProjDirPath}/../board/board_i2c.c"
"${ProjDirPath}/../board/board_io.c"
"${ProjDirPath}/../board/board_descr.c"
"${ProjDirPath}/../board/pinmux.c"
"${ProjDirPath}/../lib/libfdt/fdt.c"
"${ProjDirPath}/../lib/libfdt/fdt_addresses.c"
"${ProjDirPath}/../lib/libfdt/fdt_check.c"
"${ProjDirPath}/../lib/libfdt/fdt_empty_tree.c"
"${ProjDirPath}/../lib/libfdt/fdt_overlay.c"
"${ProjDirPath}/../lib/libfdt/fdt_ro.c"
"${ProjDirPath}/../lib/libfdt/fdt_rw.c"
"${ProjDirPath}/../lib/libfdt/fdt_strerror.c"
"${ProjDirPath}/../lib/libfdt/fdt_sw.c"
"${ProjDirPath}/../lib/libfdt/fdt_wip.c"
"${ProjDirPath}/../srtm/app_srtm.c"
"${ProjDirPath}/../srtm/channels/srtm_rpmsg_endpoint.c"
"${ProjDirPath}/../srtm/services/srtm_lfcl_service.c"
"${ProjDirPath}/../srtm/services/srtm_i2c_service.c"
"${ProjDirPath}/../srtm/services/srtm_io_service.c"
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${ProjDirPath}/..
    ${ProjDirPath}/../include
)

set_source_files_properties("${ProjDirPath}/../FreeRTOSConfig.h" PROPERTIES COMPONENT_CONFIG_FILE "middleware_freertos-kernel_template")

include(${SdkRootDirPath}/core/devices/MIMX8UD7/all_lib_device.cmake)

IF(NOT DEFINED TARGET_LINK_SYSTEM_LIBRARIES)  
    SET(TARGET_LINK_SYSTEM_LIBRARIES "-lm -lc -lgcc -lnosys")  
ENDIF()  

TARGET_LINK_LIBRARIES(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--start-group)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE ${TARGET_LINK_SYSTEM_LIBRARIES})

TARGET_LINK_LIBRARIES(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--end-group)

ADD_CUSTOM_COMMAND(TARGET ${MCUX_SDK_PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_OBJCOPY}
-Obinary ${EXECUTABLE_OUTPUT_PATH}/${MCUX_SDK_PROJECT_NAME} ${EXECUTABLE_OUTPUT_PATH}/m33_image.bin)

add_compile_definitions(APP_VER="${RDAEMON_VERSION}")

execute_process(
    COMMAND git rev-parse --short HEAD
    WORKING_DIRECTORY "${ProjDirPath}/.."
    OUTPUT_VARIABLE GIT_HEAD
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

execute_process(
    COMMAND git diff --quiet
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    RESULT_VARIABLE GIT_IS_DIRTY
)

if(GIT_IS_DIRTY EQUAL 1)
    set(GIT_DIRTY_FLAG "-dirty")
else()
    set(GIT_DIRTY_FLAG "")
endif()

add_compile_definitions(GIT_VER="${GIT_HEAD}${GIT_DIRTY_FLAG}")

set_target_properties(${MCUX_SDK_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES "output.map;${EXECUTABLE_OUTPUT_PATH}/m33_image.bin")

