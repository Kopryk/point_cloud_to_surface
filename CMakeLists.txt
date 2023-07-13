cmake_minimum_required(VERSION 3.10)

project(point_cloud_to_surface)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_CXX_EXTENSIONS Off)


add_subdirectory(utils)
add_subdirectory(renderer)
add_subdirectory(surface_reconstruction)

add_subdirectory(shared)
