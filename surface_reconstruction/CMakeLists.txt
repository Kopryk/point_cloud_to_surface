
add_subdirectory(sources)

add_library(surface_reconstruction STATIC ${SURFACE_RECONSTRUCTION_SOURCES})
set_target_properties(surface_reconstruction PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(surface_reconstruction PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/sources")
