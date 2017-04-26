cmake_minimum_required(VERSION 3.0)
project(F3D.faceSensors)
add_definitions(-DFACESENSORS_COMPILING)

file(GLOB_RECURSE ${PROJECT_NAME}.Sources "src/*.cpp")
file(GLOB_RECURSE ${PROJECT_NAME}.Headers "include/*.h")

add_library (${PROJECT_NAME} SHARED ${${PROJECT_NAME}.Sources} ${${PROJECT_NAME}.Headers})

target_include_directories(${PROJECT_NAME}
	PUBLIC include
)

target_link_libraries(${PROJECT_NAME}
	F3D.faceCommon
)

if (NOT 3DFACE_TOPLEVEL)
	installDefault(${PROJECT_NAME} lib lib)
endif()