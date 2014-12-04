cmake_minimum_required(VERSION 3.0)
project(F3D.appScan)

find_package(Qt5Widgets)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

file(GLOB_RECURSE ${PROJECT_NAME}.Sources "src/*.cpp")
add_executable (${PROJECT_NAME} ${${PROJECT_NAME}.Sources})

if (3DFACE_OPENMP)
	SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-fopenmp")
	SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-fopenmp")
endif()

target_link_libraries(${PROJECT_NAME}
	F3D.faceExtras
	F3D.softKinetic
	F3D.occipital
	F3D.kinect
)