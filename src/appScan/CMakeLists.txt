cmake_minimum_required(VERSION 3.0)
project(F3D.appScan)

find_package(Qt5Widgets)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

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
	F3D.faceExtras
	F3D.faceSensors
	${dlib}
)
