
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

add_executable(${PROJECT_NAME} main.cpp)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
	target_compile_definitions(${PROJECT_NAME} INTERFACE DEBUG=1)
else()
	target_compile_definitions(${PROJECT_NAME} INTERFACE DEBUG=0)
endif()

target_link_libraries(${PROJECT_NAME} renderer)
target_link_libraries(${PROJECT_NAME} surface_reconstruction)

add_custom_command(
    TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/external/glew-2.2.0/bin/Release/x64/glew32.dll 
        $<TARGET_FILE_DIR:${PROJECT_NAME}>
)
