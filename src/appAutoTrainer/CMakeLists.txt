cmake_minimum_required(VERSION 3.0)
project(F3D.appAutoTrainer)

file(GLOB_RECURSE ${PROJECT_NAME}.Sources "src/*.cpp")
file(GLOB_RECURSE ${PROJECT_NAME}.Headers "src/*.h")
add_executable (${PROJECT_NAME}
	${${PROJECT_NAME}.Sources}
	${${PROJECT_NAME}.Headers}
)

target_link_libraries(${PROJECT_NAME}
	F3D.faceCommon
)
