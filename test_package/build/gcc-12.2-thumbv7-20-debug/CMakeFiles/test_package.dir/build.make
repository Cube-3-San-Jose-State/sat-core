# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.27

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /Users/adrien/.conan2/p/cmake253f38c8fbec3/p/CMake.app/Contents/bin/cmake

# The command to remove a file.
RM = /Users/adrien/.conan2/p/cmake253f38c8fbec3/p/CMake.app/Contents/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/adrien/Repos/hoplite-sat/test_package

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/adrien/Repos/hoplite-sat/test_package/build/gcc-12.2-thumbv7-20-debug

# Include any dependencies generated for this target.
include CMakeFiles/test_package.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/test_package.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/test_package.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_package.dir/flags.make

CMakeFiles/test_package.dir/main.cpp.obj: CMakeFiles/test_package.dir/flags.make
CMakeFiles/test_package.dir/main.cpp.obj: /Users/adrien/Repos/hoplite-sat/test_package/main.cpp
CMakeFiles/test_package.dir/main.cpp.obj: CMakeFiles/test_package.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/adrien/Repos/hoplite-sat/test_package/build/gcc-12.2-thumbv7-20-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test_package.dir/main.cpp.obj"
	/Users/adrien/.conan2/p/b/arm-ge47aa5425c57a/p/bin/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test_package.dir/main.cpp.obj -MF CMakeFiles/test_package.dir/main.cpp.obj.d -o CMakeFiles/test_package.dir/main.cpp.obj -c /Users/adrien/Repos/hoplite-sat/test_package/main.cpp

CMakeFiles/test_package.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/test_package.dir/main.cpp.i"
	/Users/adrien/.conan2/p/b/arm-ge47aa5425c57a/p/bin/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/adrien/Repos/hoplite-sat/test_package/main.cpp > CMakeFiles/test_package.dir/main.cpp.i

CMakeFiles/test_package.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/test_package.dir/main.cpp.s"
	/Users/adrien/.conan2/p/b/arm-ge47aa5425c57a/p/bin/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/adrien/Repos/hoplite-sat/test_package/main.cpp -o CMakeFiles/test_package.dir/main.cpp.s

# Object files for target test_package
test_package_OBJECTS = \
"CMakeFiles/test_package.dir/main.cpp.obj"

# External object files for target test_package
test_package_EXTERNAL_OBJECTS =

test_package: CMakeFiles/test_package.dir/main.cpp.obj
test_package: CMakeFiles/test_package.dir/build.make
test_package: /Users/adrien/.conan2/p/b/sat-c4116cefd39916/p/lib/libsat-core.a
test_package: /Users/adrien/.conan2/p/b/telemf3b345fd881f9/p/lib/libtelemetry-recorder.a
test_package: /Users/adrien/.conan2/p/b/libha948f473678d3c/p/lib/liblibhal-icm.a
test_package: /Users/adrien/.conan2/p/b/libha11063baf80f44/p/lib/liblibhal-neo.a
test_package: /Users/adrien/.conan2/p/b/libhabc868406ed398/p/lib/liblibhal-mpl.a
test_package: /Users/adrien/.conan2/p/b/libha6b62fedd043e1/p/lib/liblibhal-xbee.a
test_package: /Users/adrien/.conan2/p/libhac9a6203da3932/p/lib/liblibhal-util.a
test_package: CMakeFiles/test_package.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/Users/adrien/Repos/hoplite-sat/test_package/build/gcc-12.2-thumbv7-20-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable test_package"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_package.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_package.dir/build: test_package
.PHONY : CMakeFiles/test_package.dir/build

CMakeFiles/test_package.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_package.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_package.dir/clean

CMakeFiles/test_package.dir/depend:
	cd /Users/adrien/Repos/hoplite-sat/test_package/build/gcc-12.2-thumbv7-20-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/adrien/Repos/hoplite-sat/test_package /Users/adrien/Repos/hoplite-sat/test_package /Users/adrien/Repos/hoplite-sat/test_package/build/gcc-12.2-thumbv7-20-debug /Users/adrien/Repos/hoplite-sat/test_package/build/gcc-12.2-thumbv7-20-debug /Users/adrien/Repos/hoplite-sat/test_package/build/gcc-12.2-thumbv7-20-debug/CMakeFiles/test_package.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/test_package.dir/depend

