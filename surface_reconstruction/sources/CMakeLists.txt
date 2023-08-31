
set(SURFACE_RECONSTRUCTION_SOURCES 
	${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt
	${SURFACE_RECONSTRUCTION_SOURCES})

add_subdirectory(weighted_poisson_equation)
add_subdirectory(divergence)
add_subdirectory(scalar_field)
add_subdirectory(scale)
add_subdirectory(marching_cubes)
add_subdirectory(normal_estimation)
add_subdirectory(voxelization)
add_subdirectory(voxelization_with_average)


set(SURFACE_RECONSTRUCTION_SOURCES 
	${SURFACE_RECONSTRUCTION_SOURCES}
	PARENT_SCOPE)