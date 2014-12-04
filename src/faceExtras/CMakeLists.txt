cmake_minimum_required(VERSION 3.0)
project(F3D.faceExtras)

find_package(Qt5OpenGL)
find_package(Qt5Widgets)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Unfortunately, autoMOC can't be used
# Otherwise "undefined reference to `vtable for..." will raise when linking this library
#set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

file(GLOB_RECURSE ${PROJECT_NAME}.Sources "src/*.cpp")
file(GLOB_RECURSE ${PROJECT_NAME}.Headers "include/*.h")
#file(GLOB_RECURSE ${PROJECT_NAME}.UI "src/*.ui")

#qt5_wrap_ui(${PROJECT_NAME}.UI.Headers ${${PROJECT_NAME}.UI})
qt5_wrap_cpp(${PROJECT_NAME}.Headers.MOC ${${PROJECT_NAME}.Headers})

add_library (${PROJECT_NAME} SHARED
	${${PROJECT_NAME}.Sources}
        ${${PROJECT_NAME}.Headers.MOC}
#        ${${PROJECT_NAME}.UI.Headers}
)

target_include_directories(${PROJECT_NAME}
	PUBLIC include
	PUBLIC "/usr/include/qwt/"
        PRIVATE src
)

target_link_libraries(${PROJECT_NAME}
	PUBLIC qwt
	PUBLIC F3D.faceCommon
	Qt5::Widgets
	Qt5OpenGL
)
