cmake_minimum_required(VERSION 3.10)

project(SLAM_SERVER)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# main
set(SOURCES
    main.cpp
)

# automatyczne zbieranie cpp z communication/
file(GLOB COMMUNICATION_SOURCES
    communication/*.cpp
)

# target
add_executable(server
    ${SOURCES}
    ${COMMUNICATION_SOURCES}
)

add_executable(matlab_publisher
    matlab_publisher.cpp
)

# include headers
target_include_directories(server PRIVATE
    ${CMAKE_SOURCE_DIR}/communication
)

if(WIN32)
    target_link_libraries(server PRIVATE ws2_32)
    target_link_libraries(matlab_publisher PRIVATE ws2_32)
endif()
