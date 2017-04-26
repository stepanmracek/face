cmake_minimum_required(VERSION 3.0)
project(F3D.sensor.realSense)
add_definitions(-DREALSENSE_COMPILING)

file(GLOB_RECURSE ${PROJECT_NAME}.Sources "src/*.*")
file(GLOB_RECURSE ${PROJECT_NAME}.Headers "include/*.h")

add_library (${PROJECT_NAME} SHARED ${${PROJECT_NAME}.Sources} ${${PROJECT_NAME}.Headers})

target_include_directories(${PROJECT_NAME} 
	PRIVATE src
	PUBLIC include "${RSSDK_ROOT}/include"
)

find_library(LIB_RS_PXC libpxc "${RSSDK_ROOT}/lib/Win32")

message(STATUS "LIB_RS_PXC: ${LIB_RS_PXC}")

target_link_libraries(${PROJECT_NAME}
	F3D.faceCommon
	F3D.faceSensors
	${LIB_RS_PXC}
	${dlib}
)

if (NOT 3DFACE_TOPLEVEL)
	installDefault(${PROJECT_NAME} lib lib)
endif()
