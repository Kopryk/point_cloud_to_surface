set(RENDERER_SOURCES 
	${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt
	${RENDERER_SOURCES}
)

add_subdirectory(graphics)

message("Renderer sources")
message("${RENDERER_SOURCES}")

set(RENDERER_SOURCES ${RENDERER_SOURCES} PARENT_SCOPE)

