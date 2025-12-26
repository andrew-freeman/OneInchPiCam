cmake_minimum_required(VERSION 3.10)
project(OneInchPiCam LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(RAWUDP_ENABLE_OPENCV "Enable OpenCV preview support if found" ON)
option(RAWUDP_ENABLE_LIBCAMERA "Enable libcamera frame source if found" ON)
option(RAWUDP_ENABLE_GSTREAMER "Enable GStreamer frame source if found" ON)

add_library(rawudp_protocol INTERFACE)
target_include_directories(rawudp_protocol INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_library(rawudp_common
    src/generator.cpp
    src/frame_decoder.cpp
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

add_executable(rawudp_sender
    src/sender.cpp
    src/frame_source.cpp
    src/libcamera_source.cpp
    src/gstreamer_source.cpp
)
target_link_libraries(rawudp_sender PRIVATE rawudp_common)

add_executable(rawudp_receiver src/receiver.cpp)
target_link_libraries(rawudp_receiver PRIVATE rawudp_common)

if(RAWUDP_ENABLE_LIBCAMERA)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(LIBCAMERA libcamera)
        if(LIBCAMERA_FOUND)
            target_compile_definitions(rawudp_sender PRIVATE RAWUDP_HAVE_LIBCAMERA)
            target_include_directories(rawudp_sender PRIVATE ${LIBCAMERA_INCLUDE_DIRS})
            target_link_libraries(rawudp_sender PRIVATE ${LIBCAMERA_LIBRARIES})
        endif()
    endif()
endif()

if(RAWUDP_ENABLE_GSTREAMER)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(GSTREAMER gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)
        if(GSTREAMER_FOUND)
            target_compile_definitions(rawudp_sender PRIVATE RAWUDP_HAVE_GSTREAMER)
            target_include_directories(rawudp_sender PRIVATE ${GSTREAMER_INCLUDE_DIRS})
            target_link_libraries(rawudp_sender PRIVATE ${GSTREAMER_LIBRARIES})
        endif()
    endif()
endif()
