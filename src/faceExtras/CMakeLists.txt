cmake_minimum_required(VERSION 3.0)
project(F3D.faceExtras)
add_definitions(-DFACEEXTRAS_COMPILING)

find_package(Qt5OpenGL)
find_package(Qt5Widgets)
find_package(Qt5Svg)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Unfortunately, autoMOC can't be used
# Otherwise "undefined reference to `vtable for..." will raise when linking this library
#set(CMAKE_AUTOMOC ON)

# We can't use autouic as well. Otherwise UI files are not shown in Visual Studio
#set(CMAKE_AUTOUIC ON)

file(GLOB_RECURSE ${PROJECT_NAME}.Sources "src/*.cpp")
file(GLOB_RECURSE ${PROJECT_NAME}.Headers "include/*.h")
file(GLOB_RECURSE ${PROJECT_NAME}.UI "src/*.ui")
file(GLOB_RECURSE ${PROJECT_NAME}.Resources "resources/*.qrc")

qt5_wrap_ui(${PROJECT_NAME}.UI.Headers ${${PROJECT_NAME}.UI})
qt5_wrap_cpp(${PROJECT_NAME}.Headers.MOC ${${PROJECT_NAME}.Headers})
qt5_add_resources(${PROJECT_NAME}.Resources.cpp ${${PROJECT_NAME}.Resources})

add_library (${PROJECT_NAME} SHARED
	${${PROJECT_NAME}.Sources}
	${${PROJECT_NAME}.Headers}
	${${PROJECT_NAME}.Headers.MOC}
	${${PROJECT_NAME}.UI.Headers}
	${${PROJECT_NAME}.Resources.cpp}
)

if(MSVC)
	find_library(qwt
		NAMES qwt
		HINTS ${QWT_ROOT}/lib)
else()
	set(qwt qwt)
endif()

target_include_directories(${PROJECT_NAME}
	PUBLIC include
	PUBLIC "/usr/include/qwt/"
	PUBLIC "${QWT_ROOT}/include"
	PUBLIC "${QWT_ROOT}/include/qwt"
	PRIVATE src
)

target_link_libraries(${PROJECT_NAME}
	PUBLIC ${qwt}
	PUBLIC F3D.faceCommon
	PUBLIC F3D.faceSensors
	Qt5::Widgets
	Qt5::OpenGL
	Qt5::Svg
)

if(MSVC)
	target_link_libraries(${PROJECT_NAME}
		PRIVATE OpenGL32
		PRIVATE GlU32
	)
endif()