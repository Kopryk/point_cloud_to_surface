
add_subdirectory(sources)


add_library(renderer STATIC ${RENDERER_SOURCES})

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}"
        PREFIX "Implementation"
        FILES ${RENDERER_SOURCES})


# link with glew
target_include_directories(renderer PUBLIC "${CMAKE_SOURCE_DIR}/external/glew-2.2.0/include")
target_link_libraries(renderer PUBLIC "${CMAKE_SOURCE_DIR}/external/glew-2.2.0/lib/Release/x64/glew32.lib")

set(IMGUI_SOURCES 
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

target_sources(renderer PUBLIC ${IMGUI_SOURCES})

# for now manual glfw compilation
# create build folder inside glfw dir and run
# cmake ..
# cmake --build .

target_include_directories(renderer PUBLIC "${CMAKE_SOURCE_DIR}/external/glfw/include")
# Link against GLFW library
target_link_libraries(renderer PUBLIC "${CMAKE_SOURCE_DIR}/external/glfw/build/src/Debug/glfw3.lib")

set_target_properties(renderer PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(renderer PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/sources")
target_include_directories(renderer PUBLIC "${CMAKE_SOURCE_DIR}/external/glm")

target_link_libraries(renderer INTERFACE utils)


# ImGui is header only library
target_include_directories(renderer PUBLIC "${CMAKE_SOURCE_DIR}/external/imgui")
target_include_directories(renderer PUBLIC "${CMAKE_SOURCE_DIR}/external/imgui/backends")


target_link_libraries(renderer PUBLIC opengl32)