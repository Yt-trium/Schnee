cmake_minimum_required( VERSION 3.0 )

project( Schnee CXX)

# Retrieve source files
file( GLOB incList "${CMAKE_CURRENT_SOURCE_DIR}/inc/*.h" )
file( GLOB inc2List "${CMAKE_CURRENT_SOURCE_DIR}/inc/*.hpp" )
file( GLOB srcList "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" )

# Target program
add_executable( ${PROJECT_NAME} ${srcList} ${incList} ${inc2List} )
