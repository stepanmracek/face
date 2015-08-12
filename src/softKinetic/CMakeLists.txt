cmake_minimum_required(VERSION 3.0)
project(F3D.sensor.softKinetic)

file(GLOB_RECURSE ${PROJECT_NAME}.Sources "src/*.cpp")
add_library (${PROJECT_NAME} SHARED ${${PROJECT_NAME}.Sources})

add_definitions(-DSKAPI_EXPORTS)

if (3DFACE_TOPLEVEL)
	SET(TargetDevice "x86")
endif()

target_include_directories(${PROJECT_NAME} 
	PRIVATE src
	PUBLIC include /opt/softkinetic/DepthSenseSDK/include/
)

if(MSVC)
	unset(LIB_DS CACHE)
	find_library(LIB_DS DepthSense)
	target_include_directories(${PROJECT_NAME}
		PUBLIC include ${DSSDK_ROOT}/include
	)
else()
	find_library(LIB_DS DepthSense /opt/softkinetic/DepthSenseSDK/lib/)
	find_library(LIB_DS_PLUGINS DepthSensePlugins /opt/softkinetic/DepthSenseSDK/lib/)
	find_library(LIB_DS_TURBOJPEG turbojpeg /opt/softkinetic/DepthSenseSDK/lib/)
endif()

message(STATUS "LIB_DS: ${LIB_DS}")

target_link_libraries(${PROJECT_NAME}
	F3D.faceCommon
	F3D.faceSensors
	${LIB_DS}
	${dlib}
)

if(NOT MSVC)
	target_link_libraries(${PROJECT_NAME}
		${LIB_DS_PLUGINS}
		${LIB_DS_TURBOJPEG}
	)
endif()

if(${TargetDevice} STREQUAL "lindevice")
	target_link_libraries(${PROJECT_NAME}
		TBS)
endif()

if (NOT 3DFACE_TOPLEVEL)
	installDefault(${PROJECT_NAME} lib lib)
endif()
