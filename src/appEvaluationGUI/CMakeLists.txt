cmake_minimum_required(VERSION 3.0)
project(F3D.appEvaluationGUI)

find_package(Qt5OpenGL)
find_package(Qt5Widgets)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

file(GLOB_RECURSE ${PROJECT_NAME}.Sources "src/*.cpp")
add_executable (${PROJECT_NAME} ${${PROJECT_NAME}.Sources})

target_link_libraries(${PROJECT_NAME}
        F3D.faceExtras
)
