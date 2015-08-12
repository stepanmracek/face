cmake_minimum_required(VERSION 3.0)
project(F3D.appBenchmark)

file(GLOB_RECURSE ${PROJECT_NAME}.Sources "src/*.cpp")
add_executable (${PROJECT_NAME} ${${PROJECT_NAME}.Sources})

target_link_libraries(${PROJECT_NAME}
	F3D.faceCommon
)

install(TARGETS ${PROJECT_NAME}
	RUNTIME DESTINATION .)
