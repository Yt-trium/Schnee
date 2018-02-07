cmake_minimum_required( VERSION 3.0 )

project( Schnee CXX)

# Retrieve source files
file( GLOB incList "${CMAKE_CURRENT_SOURCE_DIR}/sources/*.h" )
file( GLOB inc2List "${CMAKE_CURRENT_SOURCE_DIR}/sources/*.hpp" )
file( GLOB srcList "${CMAKE_CURRENT_SOURCE_DIR}/sources/*.cpp" )

# Target program
add_executable( ${PROJECT_NAME} ${srcList} ${incList} ${inc2List} )
