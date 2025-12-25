cmake_minimum_required(VERSION 3.10)
project(OneInchPiCam LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(RAWUDP_ENABLE_OPENCV "Enable OpenCV preview support if found" ON)

add_library(rawudp_protocol INTERFACE)
target_include_directories(rawudp_protocol INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_library(rawudp_common
    src/generator.cpp
    src/reassembler.cpp
    src/preview.cpp
)
target_link_libraries(rawudp_common PUBLIC rawudp_protocol)
target_include_directories(rawudp_common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

if(RAWUDP_ENABLE_OPENCV)
    find_package(OpenCV QUIET COMPONENTS core highgui imgproc)
    if(OpenCV_FOUND)
        target_compile_definitions(rawudp_common PUBLIC RAWUDP_HAVE_OPENCV)
        target_link_libraries(rawudp_common PUBLIC ${OpenCV_LIBS})
        target_include_directories(rawudp_common PUBLIC ${OpenCV_INCLUDE_DIRS})
    endif()
endif()

add_executable(rawudp_sender src/sender.cpp)
target_link_libraries(rawudp_sender PRIVATE rawudp_common)

add_executable(rawudp_receiver src/receiver.cpp)
target_link_libraries(rawudp_receiver PRIVATE rawudp_common)
