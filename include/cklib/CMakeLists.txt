cmake_minimum_required(VERSION 3.10)

add_library(cklib STATIC IMPORTED GLOBAL)

#link .a & add dictionary ch@20250728
set_target_properties(cklib PROPERTIES
 #   IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/lib/libcklib.a"
    IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/lib/libcklib_20260731.a"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_LIST_DIR}/inc"
)
