cmake_minimum_required( VERSION 3.0 )

project( SCHNEE_LIB CXX)

# Retrieve source files
file( GLOB incList "${CMAKE_CURRENT_SOURCE_DIR}/inc/*.h" )
file( GLOB inc2List "${CMAKE_CURRENT_SOURCE_DIR}/inc/*.hpp" )
file( GLOB srcList "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" )

add_library( SCHNEE_LIB ${srcList} ${incList} ${inc2List} )

# Target program
project( SCHNEE_EXEC CXX)

add_executable( SCHNEE_MAIN "schnee_main.cpp")
target_include_directories( SCHNEE_MAIN PRIVATE ${incList} ${inc2List})
target_link_libraries( SCHNEE_MAIN SCHNEE_LIB )

add_executable( SCHNEE_TEST "schnee_test.cpp")
target_link_libraries( SCHNEE_TEST SCHNEE_LIB )
