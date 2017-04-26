cmake_minimum_required(VERSION 3.0)
project(F3D.faceCommon)

file(GLOB_RECURSE ${PROJECT_NAME}.Sources "src/*.cpp")
add_library (${PROJECT_NAME} SHARED ${${PROJECT_NAME}.Sources})

if (3DFACE_OPENMP)
	SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-fopenmp")
endif()

target_include_directories(${PROJECT_NAME}
	PUBLIC include
	PRIVATE src
)

target_link_libraries(${PROJECT_NAME}
	${POCOFOUNDATION_LIBS}
	${OpenCV_LIBS}
)
