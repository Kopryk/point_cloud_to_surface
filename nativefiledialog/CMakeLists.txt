cmake_minimum_required(VERSION 3.10)

project(nativefiledialog)

include_directories(src/include)

add_library(nfd
    src/nfd_common.c
	src/nfd_win.cpp
)


if(WIN32)
    target_link_libraries(nfd ole32 uuid shell32)
endif()