cmake_minimum_required(VERSION 3.0)
project(F3D.appDemo)

find_package(Qt5OpenGL)
find_package(Qt5Widgets)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
#set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOUIC ON)
#set(CMAKE_AUTORCC ON)

file(GLOB_RECURSE ${PROJECT_NAME}.Sources "src/*.cpp")
file(GLOB_RECURSE ${PROJECT_NAME}.Headers "src/*.h")
file(GLOB_RECURSE ${PROJECT_NAME}.UI "src/*.ui")
file(GLOB_RECURSE ${PROJECT_NAME}.Resources "resources/*.qrc")

qt5_wrap_ui(${PROJECT_NAME}.UI.Headers ${${PROJECT_NAME}.UI})
qt5_wrap_cpp(${PROJECT_NAME}.Headers.MOC ${${PROJECT_NAME}.Headers})
qt5_add_resources(${PROJECT_NAME}.Resources.cpp ${${PROJECT_NAME}.Resources})

add_executable (${PROJECT_NAME}
	${${PROJECT_NAME}.Sources}
	${${PROJECT_NAME}.Headers}
	${${PROJECT_NAME}.Headers.MOC}
	${${PROJECT_NAME}.UI.Headers}
	${${PROJECT_NAME}.Resources.cpp}
)

target_link_libraries(${PROJECT_NAME}
	F3D.faceCommon
	F3D.faceExtras
	F3D.faceSensors
	Qt5::Widgets
	Qt5::OpenGL
)

target_include_directories(${PROJECT_NAME}
	PRIVATE src
)
