CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
PROJECT(EE3100704-projects)

set(RAISIM_PATH "$ENV{HOME}/Library/raisimLib/raisim/linux")
include_directories(${RAISIM_PATH}/include)
link_directories(${RAISIM_PATH}/lib)

include_directories(tools/include)
include_directories(/usr/include/python3.8)
include_directories(/home/percy/matplotlib-cpp)

find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})

include_directories(
        ${EIGEN3_INCLUDE_DIRS}
)

add_subdirectory(examples)
find_package(Python3 REQUIRED COMPONENTS Development)
#target_include_directories(canine PRIVATE ${Python3_INCLUDE_DIRS})
#target_link_libraries(canine PRIVATE ${Python3_LIBRARIES})