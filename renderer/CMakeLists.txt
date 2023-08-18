
add_subdirectory(sources)
add_library(renderer STATIC ${RENDERER_SOURCES})

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}"
        PREFIX "Implementation"
        FILES ${RENDERER_SOURCES})

set_target_properties(renderer PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(renderer PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/sources")

target_link_libraries(renderer INTERFACE utils)
