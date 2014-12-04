cmake_minimum_required(VERSION 3.0)
project(F3D.softKinetic)

file(GLOB_RECURSE ${PROJECT_NAME}.Sources "src/*.cpp")
add_library (${PROJECT_NAME} SHARED ${${PROJECT_NAME}.Sources})

target_include_directories(${PROJECT_NAME} 
	PRIVATE src
	PUBLIC include /opt/softkinetic/DepthSenseSDK/include/
)

find_library(LIB_DS DepthSense /opt/softkinetic/DepthSenseSDK/lib/)
find_library(LIB_DS_PLUGINS DepthSensePlugins /opt/softkinetic/DepthSenseSDK/lib/)
find_library(LIB_DS_TURBOJPEG turbojpeg /opt/softkinetic/DepthSenseSDK/lib/)

target_link_libraries(${PROJECT_NAME}
	F3D.faceCommon
	F3D.faceSensors
	${LIB_DS}
	${LIB_DS_PLUGINS}
	${LIB_DS_TURBOJPEG}
)
