cmake_minimum_required(VERSION 3.0)
project(F3D.faceCommon)
add_definitions(-DFACECOMMON_COMPILING)

file(GLOB_RECURSE ${PROJECT_NAME}.Sources "src/*.cpp")
file(GLOB_RECURSE ${PROJECT_NAME}.Headers "include/*.h")
add_library (${PROJECT_NAME} SHARED ${${PROJECT_NAME}.Sources} ${${PROJECT_NAME}.Headers})

target_include_directories(${PROJECT_NAME}
	PUBLIC include
	PUBLIC ${POCO_ROOT}/include
	PUBLIC ${ZLIB_ROOT}/include
	PUBLIC ${DLIB_ROOT}/include
	PRIVATE src
)

target_link_libraries(${PROJECT_NAME}
	PUBLIC ${OpenCV_LIBS}
	PUBLIC ${PocoFoundation}
	PUBLIC ${zlib}
	PRIVATE ${dlib}
)

if (NOT 3DFACE_TOPLEVEL)
	installDefault(${PROJECT_NAME} lib lib)
endif()

