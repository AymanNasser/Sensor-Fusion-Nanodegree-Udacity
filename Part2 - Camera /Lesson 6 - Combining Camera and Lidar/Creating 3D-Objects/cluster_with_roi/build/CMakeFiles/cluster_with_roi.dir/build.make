# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = "/home/an/FOEASU-Linux/Sensor Fusion Nanodegree/Part2 - Camera /Lesson 6 - Combining Camera and Lidar/Creating 3D-Objects/cluster_with_roi"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/an/FOEASU-Linux/Sensor Fusion Nanodegree/Part2 - Camera /Lesson 6 - Combining Camera and Lidar/Creating 3D-Objects/cluster_with_roi/build"

# Include any dependencies generated for this target.
include CMakeFiles/cluster_with_roi.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cluster_with_roi.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cluster_with_roi.dir/flags.make

CMakeFiles/cluster_with_roi.dir/src/cluster_with_roi.cpp.o: CMakeFiles/cluster_with_roi.dir/flags.make
CMakeFiles/cluster_with_roi.dir/src/cluster_with_roi.cpp.o: ../src/cluster_with_roi.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/an/FOEASU-Linux/Sensor Fusion Nanodegree/Part2 - Camera /Lesson 6 - Combining Camera and Lidar/Creating 3D-Objects/cluster_with_roi/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/cluster_with_roi.dir/src/cluster_with_roi.cpp.o"
	/usr/bin/g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cluster_with_roi.dir/src/cluster_with_roi.cpp.o -c "/home/an/FOEASU-Linux/Sensor Fusion Nanodegree/Part2 - Camera /Lesson 6 - Combining Camera and Lidar/Creating 3D-Objects/cluster_with_roi/src/cluster_with_roi.cpp"

CMakeFiles/cluster_with_roi.dir/src/cluster_with_roi.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cluster_with_roi.dir/src/cluster_with_roi.cpp.i"
	/usr/bin/g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/an/FOEASU-Linux/Sensor Fusion Nanodegree/Part2 - Camera /Lesson 6 - Combining Camera and Lidar/Creating 3D-Objects/cluster_with_roi/src/cluster_with_roi.cpp" > CMakeFiles/cluster_with_roi.dir/src/cluster_with_roi.cpp.i

CMakeFiles/cluster_with_roi.dir/src/cluster_with_roi.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cluster_with_roi.dir/src/cluster_with_roi.cpp.s"
	/usr/bin/g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/an/FOEASU-Linux/Sensor Fusion Nanodegree/Part2 - Camera /Lesson 6 - Combining Camera and Lidar/Creating 3D-Objects/cluster_with_roi/src/cluster_with_roi.cpp" -o CMakeFiles/cluster_with_roi.dir/src/cluster_with_roi.cpp.s

CMakeFiles/cluster_with_roi.dir/src/cluster_with_roi.cpp.o.requires:

.PHONY : CMakeFiles/cluster_with_roi.dir/src/cluster_with_roi.cpp.o.requires

CMakeFiles/cluster_with_roi.dir/src/cluster_with_roi.cpp.o.provides: CMakeFiles/cluster_with_roi.dir/src/cluster_with_roi.cpp.o.requires
	$(MAKE) -f CMakeFiles/cluster_with_roi.dir/build.make CMakeFiles/cluster_with_roi.dir/src/cluster_with_roi.cpp.o.provides.build
.PHONY : CMakeFiles/cluster_with_roi.dir/src/cluster_with_roi.cpp.o.provides

CMakeFiles/cluster_with_roi.dir/src/cluster_with_roi.cpp.o.provides.build: CMakeFiles/cluster_with_roi.dir/src/cluster_with_roi.cpp.o


CMakeFiles/cluster_with_roi.dir/src/structIO.cpp.o: CMakeFiles/cluster_with_roi.dir/flags.make
CMakeFiles/cluster_with_roi.dir/src/structIO.cpp.o: ../src/structIO.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/an/FOEASU-Linux/Sensor Fusion Nanodegree/Part2 - Camera /Lesson 6 - Combining Camera and Lidar/Creating 3D-Objects/cluster_with_roi/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/cluster_with_roi.dir/src/structIO.cpp.o"
	/usr/bin/g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cluster_with_roi.dir/src/structIO.cpp.o -c "/home/an/FOEASU-Linux/Sensor Fusion Nanodegree/Part2 - Camera /Lesson 6 - Combining Camera and Lidar/Creating 3D-Objects/cluster_with_roi/src/structIO.cpp"

CMakeFiles/cluster_with_roi.dir/src/structIO.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cluster_with_roi.dir/src/structIO.cpp.i"
	/usr/bin/g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/an/FOEASU-Linux/Sensor Fusion Nanodegree/Part2 - Camera /Lesson 6 - Combining Camera and Lidar/Creating 3D-Objects/cluster_with_roi/src/structIO.cpp" > CMakeFiles/cluster_with_roi.dir/src/structIO.cpp.i

CMakeFiles/cluster_with_roi.dir/src/structIO.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cluster_with_roi.dir/src/structIO.cpp.s"
	/usr/bin/g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/an/FOEASU-Linux/Sensor Fusion Nanodegree/Part2 - Camera /Lesson 6 - Combining Camera and Lidar/Creating 3D-Objects/cluster_with_roi/src/structIO.cpp" -o CMakeFiles/cluster_with_roi.dir/src/structIO.cpp.s

CMakeFiles/cluster_with_roi.dir/src/structIO.cpp.o.requires:

.PHONY : CMakeFiles/cluster_with_roi.dir/src/structIO.cpp.o.requires

CMakeFiles/cluster_with_roi.dir/src/structIO.cpp.o.provides: CMakeFiles/cluster_with_roi.dir/src/structIO.cpp.o.requires
	$(MAKE) -f CMakeFiles/cluster_with_roi.dir/build.make CMakeFiles/cluster_with_roi.dir/src/structIO.cpp.o.provides.build
.PHONY : CMakeFiles/cluster_with_roi.dir/src/structIO.cpp.o.provides

CMakeFiles/cluster_with_roi.dir/src/structIO.cpp.o.provides.build: CMakeFiles/cluster_with_roi.dir/src/structIO.cpp.o


# Object files for target cluster_with_roi
cluster_with_roi_OBJECTS = \
"CMakeFiles/cluster_with_roi.dir/src/cluster_with_roi.cpp.o" \
"CMakeFiles/cluster_with_roi.dir/src/structIO.cpp.o"

# External object files for target cluster_with_roi
cluster_with_roi_EXTERNAL_OBJECTS =

cluster_with_roi: CMakeFiles/cluster_with_roi.dir/src/cluster_with_roi.cpp.o
cluster_with_roi: CMakeFiles/cluster_with_roi.dir/src/structIO.cpp.o
cluster_with_roi: CMakeFiles/cluster_with_roi.dir/build.make
cluster_with_roi: /usr/local/lib/libopencv_gapi.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_stitching.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_alphamat.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_aruco.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_bgsegm.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_bioinspired.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_ccalib.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_dnn_objdetect.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_dnn_superres.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_dpm.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_face.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_freetype.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_fuzzy.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_hdf.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_hfs.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_img_hash.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_intensity_transform.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_line_descriptor.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_mcc.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_quality.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_rapid.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_reg.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_rgbd.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_saliency.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_stereo.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_structured_light.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_superres.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_surface_matching.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_tracking.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_videostab.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_viz.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_xfeatures2d.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_xobjdetect.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_xphoto.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_shape.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_highgui.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_datasets.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_plot.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_text.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_dnn.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_ml.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_phase_unwrapping.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_optflow.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_ximgproc.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_video.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_videoio.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_imgcodecs.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_objdetect.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_calib3d.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_features2d.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_flann.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_photo.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_imgproc.so.4.5.0
cluster_with_roi: /usr/local/lib/libopencv_core.so.4.5.0
cluster_with_roi: CMakeFiles/cluster_with_roi.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/an/FOEASU-Linux/Sensor Fusion Nanodegree/Part2 - Camera /Lesson 6 - Combining Camera and Lidar/Creating 3D-Objects/cluster_with_roi/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable cluster_with_roi"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cluster_with_roi.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cluster_with_roi.dir/build: cluster_with_roi

.PHONY : CMakeFiles/cluster_with_roi.dir/build

CMakeFiles/cluster_with_roi.dir/requires: CMakeFiles/cluster_with_roi.dir/src/cluster_with_roi.cpp.o.requires
CMakeFiles/cluster_with_roi.dir/requires: CMakeFiles/cluster_with_roi.dir/src/structIO.cpp.o.requires

.PHONY : CMakeFiles/cluster_with_roi.dir/requires

CMakeFiles/cluster_with_roi.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cluster_with_roi.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cluster_with_roi.dir/clean

CMakeFiles/cluster_with_roi.dir/depend:
	cd "/home/an/FOEASU-Linux/Sensor Fusion Nanodegree/Part2 - Camera /Lesson 6 - Combining Camera and Lidar/Creating 3D-Objects/cluster_with_roi/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/an/FOEASU-Linux/Sensor Fusion Nanodegree/Part2 - Camera /Lesson 6 - Combining Camera and Lidar/Creating 3D-Objects/cluster_with_roi" "/home/an/FOEASU-Linux/Sensor Fusion Nanodegree/Part2 - Camera /Lesson 6 - Combining Camera and Lidar/Creating 3D-Objects/cluster_with_roi" "/home/an/FOEASU-Linux/Sensor Fusion Nanodegree/Part2 - Camera /Lesson 6 - Combining Camera and Lidar/Creating 3D-Objects/cluster_with_roi/build" "/home/an/FOEASU-Linux/Sensor Fusion Nanodegree/Part2 - Camera /Lesson 6 - Combining Camera and Lidar/Creating 3D-Objects/cluster_with_roi/build" "/home/an/FOEASU-Linux/Sensor Fusion Nanodegree/Part2 - Camera /Lesson 6 - Combining Camera and Lidar/Creating 3D-Objects/cluster_with_roi/build/CMakeFiles/cluster_with_roi.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/cluster_with_roi.dir/depend

