# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.29

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
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.29.0/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.29.0/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/mac/computer_graphics/Assignment3/Code

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/mac/computer_graphics/Assignment3/Code

# Include any dependencies generated for this target.
include CMakeFiles/Rasterizer.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/Rasterizer.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/Rasterizer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Rasterizer.dir/flags.make

CMakeFiles/Rasterizer.dir/main.cpp.o: CMakeFiles/Rasterizer.dir/flags.make
CMakeFiles/Rasterizer.dir/main.cpp.o: main.cpp
CMakeFiles/Rasterizer.dir/main.cpp.o: CMakeFiles/Rasterizer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/mac/computer_graphics/Assignment3/Code/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Rasterizer.dir/main.cpp.o"
	/Users/mac/Downloads/Xcode-beta.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Rasterizer.dir/main.cpp.o -MF CMakeFiles/Rasterizer.dir/main.cpp.o.d -o CMakeFiles/Rasterizer.dir/main.cpp.o -c /Users/mac/computer_graphics/Assignment3/Code/main.cpp

CMakeFiles/Rasterizer.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/Rasterizer.dir/main.cpp.i"
	/Users/mac/Downloads/Xcode-beta.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/mac/computer_graphics/Assignment3/Code/main.cpp > CMakeFiles/Rasterizer.dir/main.cpp.i

CMakeFiles/Rasterizer.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/Rasterizer.dir/main.cpp.s"
	/Users/mac/Downloads/Xcode-beta.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/mac/computer_graphics/Assignment3/Code/main.cpp -o CMakeFiles/Rasterizer.dir/main.cpp.s

CMakeFiles/Rasterizer.dir/rasterizer.cpp.o: CMakeFiles/Rasterizer.dir/flags.make
CMakeFiles/Rasterizer.dir/rasterizer.cpp.o: rasterizer.cpp
CMakeFiles/Rasterizer.dir/rasterizer.cpp.o: CMakeFiles/Rasterizer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/mac/computer_graphics/Assignment3/Code/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Rasterizer.dir/rasterizer.cpp.o"
	/Users/mac/Downloads/Xcode-beta.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Rasterizer.dir/rasterizer.cpp.o -MF CMakeFiles/Rasterizer.dir/rasterizer.cpp.o.d -o CMakeFiles/Rasterizer.dir/rasterizer.cpp.o -c /Users/mac/computer_graphics/Assignment3/Code/rasterizer.cpp

CMakeFiles/Rasterizer.dir/rasterizer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/Rasterizer.dir/rasterizer.cpp.i"
	/Users/mac/Downloads/Xcode-beta.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/mac/computer_graphics/Assignment3/Code/rasterizer.cpp > CMakeFiles/Rasterizer.dir/rasterizer.cpp.i

CMakeFiles/Rasterizer.dir/rasterizer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/Rasterizer.dir/rasterizer.cpp.s"
	/Users/mac/Downloads/Xcode-beta.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/mac/computer_graphics/Assignment3/Code/rasterizer.cpp -o CMakeFiles/Rasterizer.dir/rasterizer.cpp.s

CMakeFiles/Rasterizer.dir/Triangle.cpp.o: CMakeFiles/Rasterizer.dir/flags.make
CMakeFiles/Rasterizer.dir/Triangle.cpp.o: Triangle.cpp
CMakeFiles/Rasterizer.dir/Triangle.cpp.o: CMakeFiles/Rasterizer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/mac/computer_graphics/Assignment3/Code/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/Rasterizer.dir/Triangle.cpp.o"
	/Users/mac/Downloads/Xcode-beta.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Rasterizer.dir/Triangle.cpp.o -MF CMakeFiles/Rasterizer.dir/Triangle.cpp.o.d -o CMakeFiles/Rasterizer.dir/Triangle.cpp.o -c /Users/mac/computer_graphics/Assignment3/Code/Triangle.cpp

CMakeFiles/Rasterizer.dir/Triangle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/Rasterizer.dir/Triangle.cpp.i"
	/Users/mac/Downloads/Xcode-beta.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/mac/computer_graphics/Assignment3/Code/Triangle.cpp > CMakeFiles/Rasterizer.dir/Triangle.cpp.i

CMakeFiles/Rasterizer.dir/Triangle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/Rasterizer.dir/Triangle.cpp.s"
	/Users/mac/Downloads/Xcode-beta.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/mac/computer_graphics/Assignment3/Code/Triangle.cpp -o CMakeFiles/Rasterizer.dir/Triangle.cpp.s

CMakeFiles/Rasterizer.dir/Texture.cpp.o: CMakeFiles/Rasterizer.dir/flags.make
CMakeFiles/Rasterizer.dir/Texture.cpp.o: Texture.cpp
CMakeFiles/Rasterizer.dir/Texture.cpp.o: CMakeFiles/Rasterizer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/mac/computer_graphics/Assignment3/Code/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/Rasterizer.dir/Texture.cpp.o"
	/Users/mac/Downloads/Xcode-beta.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Rasterizer.dir/Texture.cpp.o -MF CMakeFiles/Rasterizer.dir/Texture.cpp.o.d -o CMakeFiles/Rasterizer.dir/Texture.cpp.o -c /Users/mac/computer_graphics/Assignment3/Code/Texture.cpp

CMakeFiles/Rasterizer.dir/Texture.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/Rasterizer.dir/Texture.cpp.i"
	/Users/mac/Downloads/Xcode-beta.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/mac/computer_graphics/Assignment3/Code/Texture.cpp > CMakeFiles/Rasterizer.dir/Texture.cpp.i

CMakeFiles/Rasterizer.dir/Texture.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/Rasterizer.dir/Texture.cpp.s"
	/Users/mac/Downloads/Xcode-beta.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/mac/computer_graphics/Assignment3/Code/Texture.cpp -o CMakeFiles/Rasterizer.dir/Texture.cpp.s

# Object files for target Rasterizer
Rasterizer_OBJECTS = \
"CMakeFiles/Rasterizer.dir/main.cpp.o" \
"CMakeFiles/Rasterizer.dir/rasterizer.cpp.o" \
"CMakeFiles/Rasterizer.dir/Triangle.cpp.o" \
"CMakeFiles/Rasterizer.dir/Texture.cpp.o"

# External object files for target Rasterizer
Rasterizer_EXTERNAL_OBJECTS =

Rasterizer: CMakeFiles/Rasterizer.dir/main.cpp.o
Rasterizer: CMakeFiles/Rasterizer.dir/rasterizer.cpp.o
Rasterizer: CMakeFiles/Rasterizer.dir/Triangle.cpp.o
Rasterizer: CMakeFiles/Rasterizer.dir/Texture.cpp.o
Rasterizer: CMakeFiles/Rasterizer.dir/build.make
Rasterizer: /opt/homebrew/lib/libopencv_gapi.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_stitching.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_alphamat.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_aruco.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_bgsegm.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_bioinspired.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_ccalib.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_dnn_objdetect.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_dnn_superres.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_dpm.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_face.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_freetype.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_fuzzy.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_hfs.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_img_hash.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_intensity_transform.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_line_descriptor.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_mcc.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_quality.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_rapid.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_reg.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_rgbd.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_saliency.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_sfm.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_stereo.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_structured_light.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_superres.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_surface_matching.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_tracking.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_videostab.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_viz.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_wechat_qrcode.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_xfeatures2d.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_xobjdetect.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_xphoto.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_shape.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_highgui.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_datasets.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_plot.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_text.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_ml.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_phase_unwrapping.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_optflow.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_ximgproc.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_video.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_videoio.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_imgcodecs.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_objdetect.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_calib3d.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_dnn.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_features2d.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_flann.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_photo.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_imgproc.4.9.0.dylib
Rasterizer: /opt/homebrew/lib/libopencv_core.4.9.0.dylib
Rasterizer: CMakeFiles/Rasterizer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/Users/mac/computer_graphics/Assignment3/Code/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable Rasterizer"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Rasterizer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Rasterizer.dir/build: Rasterizer
.PHONY : CMakeFiles/Rasterizer.dir/build

CMakeFiles/Rasterizer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Rasterizer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Rasterizer.dir/clean

CMakeFiles/Rasterizer.dir/depend:
	cd /Users/mac/computer_graphics/Assignment3/Code && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/mac/computer_graphics/Assignment3/Code /Users/mac/computer_graphics/Assignment3/Code /Users/mac/computer_graphics/Assignment3/Code /Users/mac/computer_graphics/Assignment3/Code /Users/mac/computer_graphics/Assignment3/Code/CMakeFiles/Rasterizer.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/Rasterizer.dir/depend
