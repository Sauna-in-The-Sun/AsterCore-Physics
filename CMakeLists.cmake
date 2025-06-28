# CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(AsterCore)

set(SOURCES 
    src/PhysicsWorld.cpp
    src/Collision.cpp
)

include_directories(include)
add_library(astercore STATIC ${SOURCES})