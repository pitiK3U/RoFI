cmake_minimum_required(VERSION 3.6)
project(rofi)

if(${LIBCXX})
    set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -stdlib=libc++")
endif(${LIBCXX})

add_library(isoreconfig STATIC bfs.cpp)
target_include_directories(isoreconfig INTERFACE .)
target_link_libraries(isoreconfig PUBLIC configuration legacy-configuration)

add_executable(rofi-isoreconfig main.cpp)
target_link_libraries(rofi-isoreconfig PUBLIC isoreconfig configuration legacy-configuration dimcli)
