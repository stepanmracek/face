cmake_minimum_required(VERSION 3.0)
project(F3D.sensor.dummy)

file(GLOB_RECURSE ${PROJECT_NAME}.Sources "src/*.cpp")
file(GLOB_RECURSE ${PROJECT_NAME}.Headers "src/*.h")
add_library (${PROJECT_NAME} SHARED ${${PROJECT_NAME}.Sources} ${${PROJECT_NAME}.Headers})

target_include_directories(${PROJECT_NAME}
	PRIVATE src
)

target_link_libraries(${PROJECT_NAME}
	F3D.faceCommon
	F3D.faceSensors
)
