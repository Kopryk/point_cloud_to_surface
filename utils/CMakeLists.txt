add_subdirectory(sources)


add_library(utils INTERFACE ${UTILS_SOURCES})
set_target_properties(utils PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(utils INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/sources")

