set(RENDERER_SOURCES 
	${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt
	${RENDERER_SOURCES}
)

message("0")
message("${RENDERER_SOURCES}")
add_subdirectory(renderer)

set(RENDERER_SOURCES ${RENDERER_SOURCES} PARENT_SCOPE)


message("0")
message("${RENDERER_SOURCES}")

