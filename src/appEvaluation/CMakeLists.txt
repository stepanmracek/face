cmake_minimum_required(VERSION 3.0)
project(F3D.appEvaluation)

find_package(Qt5Widgets)

file(GLOB_RECURSE ${PROJECT_NAME}.Sources "src/*.cpp")
add_executable (${PROJECT_NAME} ${${PROJECT_NAME}.Sources})

target_link_libraries(${PROJECT_NAME}
        F3D.faceExtras
        F3D.faceSensors
)
