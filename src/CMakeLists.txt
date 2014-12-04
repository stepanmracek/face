cmake_minimum_required(VERSION 3.0)
project(face)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

option (3DFACE_EXTRAS "Compile additional face extras" OFF)
option (3DFACE_SOFTKINETIC "Compile softKinetic wrapper" OFF)
option (3DFACE_KINECT "Compile Kinect wrapper" OFF)
option (3DFACE_OCCIPITAL "Compile Occipital wrapper" OFF)
option (3DFACE_OPENMP "Use OpenMP Parallelization" OFF)

add_subdirectory(faceCommon)
add_subdirectory(appAutoTrainer)
add_subdirectory(appBenchmark)
add_subdirectory(faceSensors)

if (3DFACE_SOFTKINETIC)
	add_subdirectory(softKinetic)
	add_subdirectory(appSimpleSoftKineticScan)
endif()

if (3DFACE_KINECT)
	add_subdirectory(kinect)
endif()

if (3DFACE_OCCIPITAL)
	add_subdirectory(occipital)
endif()

if (3DFACE_EXTRAS)
	add_subdirectory(faceExtras)
endif()

if (3DFACE_EXTRAS AND 3DFACE_SOFTKINETIC AND 3DFACE_KINECT AND 3DFACE_OCCIPITAL)
	add_subdirectory(appScan)
	add_subdirectory(appDemo)
endif()


