cmake_minimum_required( VERSION 3.0 )

project( SCHNEE CXX)

# Retrieve source files
file( GLOB incList "${CMAKE_CURRENT_SOURCE_DIR}/schnee/inc/*.h" )
file( GLOB inc2List "${CMAKE_CURRENT_SOURCE_DIR}/schnee/inc/*.hpp" )
file( GLOB srcList "${CMAKE_CURRENT_SOURCE_DIR}/schnee/src/*.cpp" )

add_library( SCHNEE_LIB ${srcList} ${incList} ${inc2List} )
target_include_directories(SCHNEE_LIB PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/schnee/inc)

# Target program
add_executable( Schnee "${CMAKE_CURRENT_SOURCE_DIR}/core/schnee_main.cpp")
target_include_directories( Schnee PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/schnee/inc)
target_link_libraries( Schnee SCHNEE_LIB )


add_executable( Schnee_tests "${CMAKE_CURRENT_SOURCE_DIR}/core/schnee_test.cpp")
target_include_directories( Schnee_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/schnee/inc)
target_link_libraries( Schnee_tests SCHNEE_LIB )
