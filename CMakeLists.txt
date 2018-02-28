cmake_minimum_required( VERSION 3.0 )

project( SCHNEE CXX)

SET(CMAKE_CXX_FLAGS "-O2")

# DISTANT
# ---------------------
set(distant_inc_dir "${CMAKE_CURRENT_SOURCE_DIR}/distant/includes")

# DEPENDENCIES
# ---------------------
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# SCHNEE LIBRARY
# ---------------------
set(schnee_dir "${CMAKE_CURRENT_SOURCE_DIR}/schnee")
set(schnee_inc_dir "${schnee_dir}/inc")
set(schnee_src_dir "${schnee_dir}/src")

# Retrieve source files
file( GLOB schnee_headers "${schnee_inc_dir}/*.h" "${schnee_inc_dir}/*.hpp")
file( GLOB schnee_sources "${schnee_src_dir}/*.cpp" )

add_library( SCHNEE_LIB
	${schnee_sources}
	${schnee_headers})
target_link_libraries(SCHNEE_LIB
	Eigen3::Eigen)
target_include_directories(SCHNEE_LIB PRIVATE
	${schnee_inc_dir}
	${distant_inc_dir}
	)

# EXECS
# ---------------------

# Main
add_executable( Schnee "${CMAKE_CURRENT_SOURCE_DIR}/core/schnee_main.cpp")
target_include_directories( Schnee PRIVATE
	${schnee_inc_dir}
	${distant_inc_dir}
	)
target_link_libraries( Schnee SCHNEE_LIB
	Eigen3::Eigen)

# Tests
add_executable( Schnee_tests "${CMAKE_CURRENT_SOURCE_DIR}/core/schnee_test.cpp")
target_include_directories( Schnee_tests PRIVATE
	${schnee_inc_dir}
	${distant_inc_dir}
	)
target_link_libraries( Schnee_tests SCHNEE_LIB
	Eigen3::Eigen)
