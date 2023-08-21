set(RENDERER_SOURCES 
	${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt
    ${CMAKE_SOURCE_DIR}/external/imgui/imgui.cpp
    ${CMAKE_SOURCE_DIR}/external/imgui/imgui.h
    ${CMAKE_SOURCE_DIR}/external/imgui/imgui_internal.h
    ${CMAKE_SOURCE_DIR}/external/imgui/imgui_tables.cpp
    ${CMAKE_SOURCE_DIR}/external/imgui/imgui_widgets.cpp
    ${CMAKE_SOURCE_DIR}/external/imgui/imstb_rectpack.h
    ${CMAKE_SOURCE_DIR}/external/imgui/imstb_textedit.h
    ${CMAKE_SOURCE_DIR}/external/imgui/imstb_truetype.h
    ${CMAKE_SOURCE_DIR}/external/imgui/imgui_draw.cpp
    ${CMAKE_SOURCE_DIR}/external/imgui/imconfig.h
    ${CMAKE_SOURCE_DIR}/external/imgui/backends/imgui_impl_glfw.h
    ${CMAKE_SOURCE_DIR}/external/imgui/backends/imgui_impl_glfw.cpp
    ${CMAKE_SOURCE_DIR}/external/imgui/backends/imgui_impl_opengl3.h
    ${CMAKE_SOURCE_DIR}/external/imgui/backends/imgui_impl_opengl3.cpp
    ${CMAKE_SOURCE_DIR}/external/imgui/backends/imgui_impl_opengl3_loader.h
)

add_subdirectory(graphics)

message("Renderer sources")
message("${RENDERER_SOURCES}")

set(RENDERER_SOURCES ${RENDERER_SOURCES} PARENT_SCOPE)

