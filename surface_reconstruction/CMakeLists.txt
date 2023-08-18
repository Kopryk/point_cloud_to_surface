add_subdirectory(sources)

add_library(surface_reconstruction STATIC ${SURFACE_RECONSTRUCTION_SOURCES})

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}"
        PREFIX "Implementation"
        FILES ${SURFACE_RECONSTRUCTION_SOURCES})


set_target_properties(surface_reconstruction PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(surface_reconstruction PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/sources")

target_link_libraries(surface_reconstruction INTERFACE utils)


# Include OpenCL headers
set(OpenCL_INCLUDE_DIR  ${CMAKE_SOURCE_DIR}/external/opencl-headers)
target_include_directories(surface_reconstruction PUBLIC ${OpenCL_INCLUDE_DIR})

#set(OpenCL_INCLUDE_DIR /path/to/opencl/include)
#set(OpenCL_LIBRARY /path/to/opencl/library)

# Include OpenCL cpp bindings
set(OpenCL_CLHPP_INCLUDE_DIR  ${CMAKE_SOURCE_DIR}/external/opencl-clhpp/include)
target_include_directories(surface_reconstruction PUBLIC ${OpenCL_CLHPP_INCLUDE_DIR})

# Find OpenCL package
# somehow it doesnt work, so OpenCL_LIBRARY has to be hardcoded for now
set(OpenCL_LIBRARY "C:/Program Files (x86)/Intel/oneAPI/compiler/2023.0.0/windows/lib/OpenCL.lib")
find_package(OpenCL  REQUIRED)

# Link with OpenCL library
target_link_libraries(surface_reconstruction PRIVATE ${OpenCL_LIBRARIES})
