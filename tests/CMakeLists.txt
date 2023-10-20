cmake_minimum_required(VERSION 3.15)

project(6dofRobotControllerUnitTests)

set(EXECUTABLE ${PROJECT_NAME})

add_executable(${EXECUTABLE})

add_subdirectory(Catch2)

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

