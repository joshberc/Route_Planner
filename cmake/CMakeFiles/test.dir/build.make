# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/workspace/CppND-Route-Planning-Project

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/workspace/CppND-Route-Planning-Project/cmake

# Include any dependencies generated for this target.
include CMakeFiles/test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test.dir/flags.make

CMakeFiles/test.dir/test/utest_rp_a_star_search.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/test/utest_rp_a_star_search.cpp.o: ../test/utest_rp_a_star_search.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/workspace/CppND-Route-Planning-Project/cmake/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test.dir/test/utest_rp_a_star_search.cpp.o"
	/usr/bin/g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test.dir/test/utest_rp_a_star_search.cpp.o -c /home/workspace/CppND-Route-Planning-Project/test/utest_rp_a_star_search.cpp

CMakeFiles/test.dir/test/utest_rp_a_star_search.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/test/utest_rp_a_star_search.cpp.i"
	/usr/bin/g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/workspace/CppND-Route-Planning-Project/test/utest_rp_a_star_search.cpp > CMakeFiles/test.dir/test/utest_rp_a_star_search.cpp.i

CMakeFiles/test.dir/test/utest_rp_a_star_search.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/test/utest_rp_a_star_search.cpp.s"
	/usr/bin/g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/workspace/CppND-Route-Planning-Project/test/utest_rp_a_star_search.cpp -o CMakeFiles/test.dir/test/utest_rp_a_star_search.cpp.s

# Object files for target test
test_OBJECTS = \
"CMakeFiles/test.dir/test/utest_rp_a_star_search.cpp.o"

# External object files for target test
test_EXTERNAL_OBJECTS = \
"/home/workspace/CppND-Route-Planning-Project/cmake/CMakeFiles/route_planner.dir/src/route_planner.cpp.o" \
"/home/workspace/CppND-Route-Planning-Project/cmake/CMakeFiles/route_planner.dir/src/model.cpp.o" \
"/home/workspace/CppND-Route-Planning-Project/cmake/CMakeFiles/route_planner.dir/src/route_model.cpp.o"

test: CMakeFiles/test.dir/test/utest_rp_a_star_search.cpp.o
test: CMakeFiles/route_planner.dir/src/route_planner.cpp.o
test: CMakeFiles/route_planner.dir/src/model.cpp.o
test: CMakeFiles/route_planner.dir/src/route_model.cpp.o
test: CMakeFiles/test.dir/build.make
test: lib/libgtest_main.a
test: ../lib/libpugixml.a
test: lib/libgtest.a
test: CMakeFiles/test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/workspace/CppND-Route-Planning-Project/cmake/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test.dir/build: test

.PHONY : CMakeFiles/test.dir/build

CMakeFiles/test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test.dir/clean

CMakeFiles/test.dir/depend:
	cd /home/workspace/CppND-Route-Planning-Project/cmake && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/workspace/CppND-Route-Planning-Project /home/workspace/CppND-Route-Planning-Project /home/workspace/CppND-Route-Planning-Project/cmake /home/workspace/CppND-Route-Planning-Project/cmake /home/workspace/CppND-Route-Planning-Project/cmake/CMakeFiles/test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test.dir/depend

