cmake_minimum_required(VERSION 3.0)
project(face)

if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
	message(STATUS "Facelib is top-level CMakeLists.txt file")
	SET(3DFACE_TOPLEVEL TRUE)

	message(STATUS "Setting output to ${CMAKE_BINARY_DIR}/Bin/")
	SET(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/Bin/${CMAKE_BUILD_TYPE} CACHE PATH "Library output path")
	SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/Bin/${CMAKE_BUILD_TYPE} CACHE PATH "Executable output path")

	if (NOT MSVC)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	endif()
else()
	SET(3DFACE_TOPLEVEL FALSE)
endif()

option (3DFACE_EXTRAS "Compile additional face extras" ON)
option (3DFACE_SOFTKINETIC "Compile softKinetic wrapper" ON)
option (3DFACE_KINECT "Compile Kinect wrapper" OFF)
option (3DFACE_OCCIPITAL "Compile Occipital wrapper" OFF)
option (3DFACE_REALSENSE "Compile RealSense wrapper" OFF)

# OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
	message(STATUS "OpenMP found: ${OpenMP_CXX_FLAGS}")
	set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(OpenCV REQUIRED)

if(MSVC)
	# Definition for MSVC to not define min and max macros
	add_definitions(-DNOMINMAX)
	find_library(PocoFoundation
		NAMES PocoFoundation
		HINTS ${POCO_ROOT}/lib)
	find_library(zlib
		NAMES zdll
		HINTS ${ZLIB_ROOT}/lib)
	find_library(dlib
		NAMES dlib
		HINTS ${DLIB_ROOT}/lib)
else()
	set(PocoFoundation PocoFoundation)
	set(zlib z)
endif()

add_subdirectory(faceCommon)
add_subdirectory(appAutoTrainer)
add_subdirectory(appBenchmark)
add_subdirectory(faceSensors)
add_subdirectory(dummySensor)

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

if (3DFACE_REALSENSE)
	add_subdirectory(realSense)
endif()

if (3DFACE_EXTRAS)
	add_subdirectory(faceExtras)
	add_subdirectory(appEvaluation)
	add_subdirectory(appEvaluationGUI)
	add_subdirectory(appScan)
	add_subdirectory(appDemo)
endif()
