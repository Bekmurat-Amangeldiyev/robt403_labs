# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/almatikx/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/almatikx/catkin_ws/build

# Include any dependencies generated for this target.
include lab4/CMakeFiles/test_moveit.dir/depend.make

# Include the progress variables for this target.
include lab4/CMakeFiles/test_moveit.dir/progress.make

# Include the compile flags for this target's objects.
include lab4/CMakeFiles/test_moveit.dir/flags.make

lab4/CMakeFiles/test_moveit.dir/src/test.cpp.o: lab4/CMakeFiles/test_moveit.dir/flags.make
lab4/CMakeFiles/test_moveit.dir/src/test.cpp.o: /home/almatikx/catkin_ws/src/lab4/src/test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/almatikx/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object lab4/CMakeFiles/test_moveit.dir/src/test.cpp.o"
	cd /home/almatikx/catkin_ws/build/lab4 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_moveit.dir/src/test.cpp.o -c /home/almatikx/catkin_ws/src/lab4/src/test.cpp

lab4/CMakeFiles/test_moveit.dir/src/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_moveit.dir/src/test.cpp.i"
	cd /home/almatikx/catkin_ws/build/lab4 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/almatikx/catkin_ws/src/lab4/src/test.cpp > CMakeFiles/test_moveit.dir/src/test.cpp.i

lab4/CMakeFiles/test_moveit.dir/src/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_moveit.dir/src/test.cpp.s"
	cd /home/almatikx/catkin_ws/build/lab4 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/almatikx/catkin_ws/src/lab4/src/test.cpp -o CMakeFiles/test_moveit.dir/src/test.cpp.s

# Object files for target test_moveit
test_moveit_OBJECTS = \
"CMakeFiles/test_moveit.dir/src/test.cpp.o"

# External object files for target test_moveit
test_moveit_EXTERNAL_OBJECTS =

/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: lab4/CMakeFiles/test_moveit.dir/src/test.cpp.o
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: lab4/CMakeFiles/test_moveit.dir/build.make
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_common_planning_interface_objects.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_planning_scene_interface.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_move_group_interface.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_py_bindings_tools.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_warehouse.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libwarehouse_ros.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libtf.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_pick_place_planner.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_move_group_capabilities_base.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_rdf_loader.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_kinematics_plugin_loader.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_robot_model_loader.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_constraint_sampler_manager_loader.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_planning_pipeline.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_trajectory_execution_manager.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_plan_execution.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_planning_scene_monitor.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_collision_plugin_loader.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_cpp.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libdynamic_reconfigure_config_init_mutex.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_ros_occupancy_map_monitor.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_exceptions.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_background_processing.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_kinematics_base.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_robot_model.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_transforms.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_robot_state.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_robot_trajectory.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_planning_interface.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_collision_detection.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_collision_detection_fcl.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_collision_detection_bullet.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_kinematic_constraints.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_planning_scene.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_constraint_samplers.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_planning_request_adapter.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_profiler.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_python_tools.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_trajectory_processing.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_distance_field.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_collision_distance_field.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_kinematics_metrics.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_dynamics_solver.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_utils.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmoveit_test_utils.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/x86_64-linux-gnu/libfcl.so.0.6.1
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /usr/lib/x86_64-linux-gnu/libccd.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /usr/lib/x86_64-linux-gnu/libm.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/liboctomap.so.1.9.8
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/x86_64-linux-gnu/libruckig.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /usr/lib/x86_64-linux-gnu/libBulletSoftBody.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /usr/lib/x86_64-linux-gnu/libBulletDynamics.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /usr/lib/x86_64-linux-gnu/libBulletCollision.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /usr/lib/x86_64-linux-gnu/libLinearMath.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libkdl_parser.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/liburdf.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /usr/lib/x86_64-linux-gnu/liburdfdom_sensor.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /usr/lib/x86_64-linux-gnu/liburdfdom_model_state.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /usr/lib/x86_64-linux-gnu/liburdfdom_model.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /usr/lib/x86_64-linux-gnu/liburdfdom_world.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/librosconsole_bridge.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libsrdfdom.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libgeometric_shapes.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/liboctomap.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/liboctomath.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/librandom_numbers.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libclass_loader.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /usr/lib/x86_64-linux-gnu/libPocoFoundation.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /usr/lib/x86_64-linux-gnu/libdl.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libroslib.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/librospack.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /usr/lib/x86_64-linux-gnu/libpython3.8.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /usr/lib/liborocos-kdl.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /usr/lib/liborocos-kdl.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libtf2_ros.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libactionlib.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libmessage_filters.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libroscpp.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/librosconsole.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libtf2.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/librostime.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /opt/ros/noetic/lib/libcpp_common.so
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/almatikx/catkin_ws/devel/lib/lab4/test_moveit: lab4/CMakeFiles/test_moveit.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/almatikx/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/almatikx/catkin_ws/devel/lib/lab4/test_moveit"
	cd /home/almatikx/catkin_ws/build/lab4 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_moveit.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lab4/CMakeFiles/test_moveit.dir/build: /home/almatikx/catkin_ws/devel/lib/lab4/test_moveit

.PHONY : lab4/CMakeFiles/test_moveit.dir/build

lab4/CMakeFiles/test_moveit.dir/clean:
	cd /home/almatikx/catkin_ws/build/lab4 && $(CMAKE_COMMAND) -P CMakeFiles/test_moveit.dir/cmake_clean.cmake
.PHONY : lab4/CMakeFiles/test_moveit.dir/clean

lab4/CMakeFiles/test_moveit.dir/depend:
	cd /home/almatikx/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/almatikx/catkin_ws/src /home/almatikx/catkin_ws/src/lab4 /home/almatikx/catkin_ws/build /home/almatikx/catkin_ws/build/lab4 /home/almatikx/catkin_ws/build/lab4/CMakeFiles/test_moveit.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lab4/CMakeFiles/test_moveit.dir/depend

