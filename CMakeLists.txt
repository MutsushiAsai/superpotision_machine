cmake_minimum_required(VERSION 3.5.0)
project(superposition VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(Boost_USE_MULTITHREADED ON) 

find_package(Armadillo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(SFML REQUIRED COMPONENTS graphics window system)


file(GLOB SOURCES
    "src/*.cpp" 
)

file(GLOB HEADERS
    "include/*.h" 
)
add_executable(superposition ${SOURCES} ${HEADERS})

target_link_libraries(superposition Boost::system)
target_link_libraries(superposition sfml-graphics sfml-window sfml-system sfml-audio)
target_include_directories(superposition PRIVATE ${ARMADILLO_INCLUDE_DIRS})
target_include_directories(superposition PRIVATE ${Boost_INCLUDE_DIRS})
target_include_directories(superposition PRIVATE include)

include(CTest)
enable_testing()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})

include(CPack)
