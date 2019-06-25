cmake_minimum_required(VERSION 2.8.7)

enable_language(CXX)
set(CMAKE_CXX_STANDARD 17) # TODO: Downgrade to 14
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic")
find_package(Boost)
include_directories( ${Boost_INCLUDE_DIR} )


add_executable(main main.cpp)


