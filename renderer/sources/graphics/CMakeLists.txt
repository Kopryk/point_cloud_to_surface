
add_subdirectory(utils)
add_subdirectory(vertex)
add_subdirectory(mesh)

set(RENDERER_SOURCES 
	${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt
	${CMAKE_CURRENT_SOURCE_DIR}/Camera.h
	${CMAKE_CURRENT_SOURCE_DIR}/Camera.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Display.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Display.h
	${CMAKE_CURRENT_SOURCE_DIR}/FrameBuffer.h
	${CMAKE_CURRENT_SOURCE_DIR}/FileDialog.h
	${CMAKE_CURRENT_SOURCE_DIR}/GraphicsApplication.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/GraphicsApplication.h
	${CMAKE_CURRENT_SOURCE_DIR}/Renderer.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Renderer.h
	${CMAKE_CURRENT_SOURCE_DIR}/Shader.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Shader.h
	${CMAKE_CURRENT_SOURCE_DIR}/TaskManager.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/TaskManager.h
	${RENDERER_SOURCES}
)

set(RENDERER_SOURCES ${RENDERER_SOURCES} PARENT_SCOPE)
