########### AGGREGATED COMPONENTS AND DEPENDENCIES FOR THE MULTI CONFIG #####################
#############################################################################################

set(telemetry-recorder_COMPONENT_NAMES "")
list(APPEND telemetry-recorder_FIND_DEPENDENCY_NAMES libhal-icm libhal-neo libhal-mpl libhal-util libhal)
list(REMOVE_DUPLICATES telemetry-recorder_FIND_DEPENDENCY_NAMES)
set(libhal-icm_FIND_MODE "NO_MODULE")
set(libhal-neo_FIND_MODE "NO_MODULE")
set(libhal-mpl_FIND_MODE "NO_MODULE")
set(libhal-util_FIND_MODE "NO_MODULE")
set(libhal_FIND_MODE "NO_MODULE")

########### VARIABLES #######################################################################
#############################################################################################
set(telemetry-recorder_PACKAGE_FOLDER_DEBUG "/Users/adrien/.conan2/p/b/telemc81ea56982e3f/p")
set(telemetry-recorder_BUILD_MODULES_PATHS_DEBUG )


set(telemetry-recorder_INCLUDE_DIRS_DEBUG "${telemetry-recorder_PACKAGE_FOLDER_DEBUG}/include")
set(telemetry-recorder_RES_DIRS_DEBUG )
set(telemetry-recorder_DEFINITIONS_DEBUG )
set(telemetry-recorder_SHARED_LINK_FLAGS_DEBUG )
set(telemetry-recorder_EXE_LINK_FLAGS_DEBUG )
set(telemetry-recorder_OBJECTS_DEBUG )
set(telemetry-recorder_COMPILE_DEFINITIONS_DEBUG )
set(telemetry-recorder_COMPILE_OPTIONS_C_DEBUG )
set(telemetry-recorder_COMPILE_OPTIONS_CXX_DEBUG )
set(telemetry-recorder_LIB_DIRS_DEBUG "${telemetry-recorder_PACKAGE_FOLDER_DEBUG}/lib")
set(telemetry-recorder_BIN_DIRS_DEBUG )
set(telemetry-recorder_LIBRARY_TYPE_DEBUG UNKNOWN)
set(telemetry-recorder_IS_HOST_WINDOWS_DEBUG 0)
set(telemetry-recorder_LIBS_DEBUG telemetry-recorder)
set(telemetry-recorder_SYSTEM_LIBS_DEBUG )
set(telemetry-recorder_FRAMEWORK_DIRS_DEBUG )
set(telemetry-recorder_FRAMEWORKS_DEBUG )
set(telemetry-recorder_BUILD_DIRS_DEBUG )
set(telemetry-recorder_NO_SONAME_MODE_DEBUG FALSE)


# COMPOUND VARIABLES
set(telemetry-recorder_COMPILE_OPTIONS_DEBUG
    "$<$<COMPILE_LANGUAGE:CXX>:${telemetry-recorder_COMPILE_OPTIONS_CXX_DEBUG}>"
    "$<$<COMPILE_LANGUAGE:C>:${telemetry-recorder_COMPILE_OPTIONS_C_DEBUG}>")
set(telemetry-recorder_LINKER_FLAGS_DEBUG
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${telemetry-recorder_SHARED_LINK_FLAGS_DEBUG}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${telemetry-recorder_SHARED_LINK_FLAGS_DEBUG}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${telemetry-recorder_EXE_LINK_FLAGS_DEBUG}>")


set(telemetry-recorder_COMPONENTS_DEBUG )