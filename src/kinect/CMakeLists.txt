cmake_minimum_required(VERSION 3.0)
project(F3D.sensor.kinect)

file(GLOB_RECURSE ${PROJECT_NAME}.Sources "src/*.cpp")
add_library (${PROJECT_NAME} SHARED ${${PROJECT_NAME}.Sources})

target_include_directories(${PROJECT_NAME}
	PUBLIC include "/usr/include/libfreenect/"
	PRIVATE src
)

target_link_libraries(${PROJECT_NAME}
	F3D.faceCommon
	F3D.faceSensors
	freenect
	freenect_sync
)
