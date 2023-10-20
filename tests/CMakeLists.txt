cmake_minimum_required(VERSION 3.15)

project(6dofRobotControllerUnitTests)

set(EXECUTABLE ${PROJECT_NAME})

add_executable(${EXECUTABLE})


#Add Catch2 v2.x as a submodule
include(FetchContent)

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v2.x 
    GIT_PROGRESS TRUE
    GIT_SHALLOW TRUE
    GIT_SHALLOW_DEPTH 1
)
FetchContent_MakeAvailable(Catch2)

target_link_libraries(${EXECUTABLE} PRIVATE 
    Catch2::Catch2
)

target_sources(${EXECUTABLE} PRIVATE
    test_default.cpp
    test_add.cpp
)

target_include_directories(${EXECUTABLE} PRIVATE
    Catch2/single_include
)

# Set the C++ standard (e.g., C++11 or C++14)
target_compile_features(${EXECUTABLE} PRIVATE cxx_std_17)

