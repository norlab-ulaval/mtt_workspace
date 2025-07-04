cmake_minimum_required(VERSION 3.5)
project(mtt_mock_server)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

add_executable(mtt_mock_server mtt_mock_server.cpp)

target_link_libraries(mtt_mock_server Threads::Threads)

install(TARGETS mtt_mock_server DESTINATION bin)
