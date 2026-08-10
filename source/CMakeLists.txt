find_package(Corner REQUIRED)

add_library(DL_DEMO_Impl STATIC "")


if(WIN32)
        target_link_libraries(DL_DEMO_Impl
                PUBLIC
                $<BUILD_INTERFACE:Corner::corner>
                cklib
        )
else()
        target_link_libraries(DL_DEMO_Impl
                PUBLIC
                $<BUILD_INTERFACE:Corner::corner>
                cklib
#2026/05/07 del ch
 #               newlib
        )
endif()


target_sources(DL_DEMO_Impl
        PRIVATE
        DL_DEMO_Impl.cpp
)

target_include_directories(DL_DEMO_Impl
        PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/cklib/inc
)
if(WIN32)
        target_compile_options(DL_DEMO_Impl
                PRIVATE
                -fPIC
        )
else()
        target_compile_options(DL_DEMO_Impl
                PRIVATE
                -fPIC
#2026/05/07 del ch
#               -mcpu=cortex-r52
        )
endif()

# ------------------------------- install script -------------------------------
# 20250728 add ch
# install DL lib to bin
install(
        DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/../library/"
        DESTINATION "${CMAKE_CURRENT_LIST_DIR}/../bin/"
)

set(INSTALL_TARGET DL_DEMO_Impl)

target_include_directories(${INSTALL_TARGET}
        INTERFACE
        $<INSTALL_INTERFACE:${INSTALL_TARGET}/include>
)

# Configuration
# Used by cmake to find_package(xxx)
set(PROJECT_CONFIG "${CMAKE_CURRENT_BINARY_DIR}/${INSTALL_TARGET}-config.cmake")

# Library version control
set(VERSION_CONFIG "${CMAKE_CURRENT_BINARY_DIR}/${INSTALL_TARGET}-config-version.cmake")

# Include module with function 'write_basic_package_version_file'
# Configure 'xxxConfigVersion.cmake'
include(CMakePackageConfigHelpers)
write_basic_package_version_file(${VERSION_CONFIG}
        VERSION ${PACKAGE_VERSION}
        COMPATIBILITY SameMajorVersion)

# Configure 'xxxConfig.cmake'
configure_package_config_file(${INSTALL_TARGET}-config.cmake.in ${PROJECT_CONFIG}
        INSTALL_DESTINATION ${INSTALL_TARGET}/share/${INSTALL_TARGET})

# Targets:
# * <prefix>/lib/libxxx.so
# * header location after install: <prefix>/include/xxx.h
# * headers can be included by C++ code `#include <xxx.h>`
install(TARGETS ${INSTALL_TARGET}
        EXPORT ${INSTALL_TARGET}-targets
        LIBRARY DESTINATION ${INSTALL_TARGET}/lib
        ARCHIVE DESTINATION ${INSTALL_TARGET}/lib
        RUNTIME DESTINATION ${INSTALL_TARGET}/bin
)

# Headers:
# * include/xxx.h -> <prefix>/include/xxx.h
install(
        DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../include/"
        DESTINATION "${INSTALL_TARGET}/include"
        FILES_MATCHING PATTERN "*.h"
        PATTERN "*.a"
)

# Config
# * <prefix>/lib/cmake/xxx/xxxConfig.cmake
# * <prefix>/lib/cmake/xxx/xxxConfigVersion.cmake
install(FILES ${PROJECT_CONFIG} ${VERSION_CONFIG}
        DESTINATION ${INSTALL_TARGET}/share/${INSTALL_TARGET})

# Config
# * <prefix>/lib/cmake/xxx/xxxTargets.cmake
install(EXPORT ${INSTALL_TARGET}-targets
        DESTINATION ${INSTALL_TARGET}/share/${INSTALL_TARGET})

#2026/05/07 del ch
#install(FILES ${CMAKE_CURRENT_LIST_DIR}/../extern/newlib/libout.a DESTINATION ${INSTALL_TARGET}/lib)

# 20250728 add ch
install(
        DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/../bin/packages"
        DESTINATION "${CMAKE_CURRENT_LIST_DIR}/../bin/DL_SSM_CTRL_LIB/content/bin"
)
