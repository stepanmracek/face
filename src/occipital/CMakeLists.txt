cmake_minimum_required(VERSION 3.0)
project(F3D.occipital)

include_directories("${PROJECT_SOURCE_DIR}")

file(GLOB_RECURSE ${PROJECT_NAME}.Sources "src/*.cpp")
add_library (${PROJECT_NAME} SHARED ${${PROJECT_NAME}.Sources})

target_include_directories(${PROJECT_NAME} 
	PUBLIC include
	PRIVATE src
)

target_link_libraries(${PROJECT_NAME}
	F3D.faceCommon
	F3D.faceSensors
	OpenNI2
)