
add_subdirectory(sources)
add_library(renderer STATIC ${RENDERER_SOURCES})

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}"
        PREFIX "Implementation"
        FILES ${RENDERER_SOURCES})


# link with glew
target_include_directories(renderer PUBLIC "${CMAKE_SOURCE_DIR}/external/glew-2.2.0/include")
target_link_libraries(renderer PUBLIC "${CMAKE_SOURCE_DIR}/external/glew-2.2.0/lib/Release/x64/glew32.lib")

add_custom_command(
    TARGET renderer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/external/glew-2.2.0/bin/Release/x64/glew32.dll 
        $<TARGET_FILE_DIR:renderer>
)

# Add GLFW as a subdirectory
target_include_directories(renderer PUBLIC "${CMAKE_SOURCE_DIR}/external/glfw/include")
# Link against GLFW library
target_link_libraries(renderer PUBLIC glfw)

set_target_properties(renderer PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(renderer PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/sources")
target_include_directories(renderer PUBLIC "${CMAKE_SOURCE_DIR}/external/glm")

target_link_libraries(renderer INTERFACE utils)


# ImGui is header only library
target_include_directories(renderer PUBLIC "${CMAKE_SOURCE_DIR}/external/imgui")
target_include_directories(renderer PUBLIC "${CMAKE_SOURCE_DIR}/external/imgui/backends")
