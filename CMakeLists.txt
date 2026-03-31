cmake_minimum_required(VERSION 3.10)
project(capture_session)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(OpenCV REQUIRED COMPONENTS core imgcodecs videoio)
find_package(Threads REQUIRED)

# Find libserialport
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIALPORT REQUIRED libserialport)

include_directories(${LIBSERIALPORT_INCLUDE_DIRS})

add_executable(capture_session
    src/main.cpp
    src/ubx_parser.cpp
)

target_link_libraries(capture_session
    ${OpenCV_LIBS}
    ${LIBSERIALPORT_LIBRARIES}
    Threads::Threads
)
