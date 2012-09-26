cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)


add_definitions(-std=c++0x -O3 -mfpmath=sse -mmmx -msse -msse2 -msse3 -fPIC)
#add_definitions(-DUNIX)
#add_definitions( -O3 -mfpmath=sse -mmmx -msse -msse2 -msse3 -fPIC)

rosbuild_init()
rosbuild_add_boost_directories ()
rosbuild_check_for_sse ()
rosbuild_add_library(Maths include/Maths/COLOR.cpp include/Maths/MATRIX4X4.cpp include/Maths/VECTOR2D.cpp include/Maths/VECTOR3D.cpp include/Maths/VECTOR4D.cpp include/Maths/PLANE.cpp)

#set(CMAKE_CXX_FLAGS "-O3" )

#set (CMAKE_C_COMPILER /opt/intel/bin/icc)
#set (CMAKE_CXX_COMPILER /opt/intel/bin/icpc)

#set (CMAKE_C_EXECUTABLE /opt/intel/bin/xild)
#set (CMAKE_CXX_EXECUTABLE /opt/intel/bin/xild)

#set (CMAKE_C_FLAGS   "-msse3 -ip  -no-prec-div         -parallel -O3 " )
#set (CMAKE_CXX_FLAGS "-msse3 -ip  -no-prec-div         -parallel -O3 " )

#set (CMAKE_EXE_LINKER_FLAGS  "-Wl,-rpath,/opt/intel/lib -L/opt/intel/lib" )

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_executable(KinectTracker src/3DPuppetry.cpp src/Puppet.cpp src/CloudTracker.cpp src/GLManager.cpp)
target_link_libraries(KinectTracker  -lboost_system -lboost_filesystem)
target_link_libraries(KinectTracker X11)
target_link_libraries(KinectTracker Xi)
target_link_libraries(KinectTracker Xmu)
target_link_libraries(KinectTracker glut)
target_link_libraries(KinectTracker GLEW)
target_link_libraries(KinectTracker GLU)
target_link_libraries(KinectTracker GL)
target_link_libraries(KinectTracker m)
target_link_libraries(KinectTracker Maths)
target_link_libraries(KinectTracker assimp)

