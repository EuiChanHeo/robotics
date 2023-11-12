cmake_minimum_required(VERSION 3.5)
project(EE3100704-examples LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)


#add_library(${PROJECT_NAME}
#        src/kinova.cpp
#)

find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5Widgets REQUIRED)

include_directories(
        include
        ${Qt5Core_INCLUDE_DIRS}
        ${Qt5Quick_INCLUDE_DIRS}
        ${Qt5Widgets_INCLUDE_DIRS}
)

install(
        DIRECTORY include/
        DESTINATION include
)

add_executable(kinova src/kinova.cpp
        ../tools/src/cubicTrajectoryGenerator.cpp
        ../tools/src/robotController.cpp
        ../tools/src/setTime.cpp
        ../tools/src/setObstacle.cpp
)
target_link_libraries(kinova
        raisim
        pthread
)


add_executable(canine src/canine.cpp
        ../tools/src/cubicTrajectoryGenerator.cpp
        ../tools/src/robotController.cpp
        ../tools/src/setTime.cpp
        ../tools/src/setObstacle.cpp
        src/robot_UI/robot_ui/mainwindow.cpp
)

target_link_libraries(canine
        raisim
        pthread
        Qt5::Core
        Qt5::Quick
        Qt5::Widgets
)

add_executable(robot src/robot.cpp
        ../tools/src/cubicTrajectoryGenerator.cpp
        ../tools/src/robotController.cpp
        ../tools/src/setTime.cpp
        ../tools/src/setObstacle.cpp
)

target_link_libraries(robot
        raisim
        pthread
)