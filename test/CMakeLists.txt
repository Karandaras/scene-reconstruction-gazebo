include_directories (
  ${PROJECT_SOURCE_DIR}/gazebo 
  ${PROJECT_BINARY_DIR}/gazebo 
  ${ODE_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${OGRE_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/test/gtest/include
  ${PROJECT_SOURCE_DIR}/test/gtest
  ${PROJECT_SOURCE_DIR}/test
)

link_directories(  
  ${ogre_library_dirs}
  ${Boost_LIBRARY_DIRS} 
  ${ODE_LIBRARY_DIRS}
)

configure_file (test_config.h.in ${PROJECT_BINARY_DIR}/test_config.h)

# Build gtest
add_library(gtest gtest/src/gtest-all.cc)
add_library(gtest_main gtest/src/gtest_main.cc)
target_link_libraries(gtest_main gtest)
set(GTEST_LIBRARY "${PROJECT_BINARY_DIR}/test/libgtest.a")
set(GTEST_MAIN_LIBRARY "${PROJECT_BINARY_DIR}/test/libgtest_main.a")

execute_process(COMMAND cmake -E remove_directory ${CMAKE_BINARY_DIR}/test_results)
execute_process(COMMAND cmake -E make_directory ${CMAKE_BINARY_DIR}/test_results)
include_directories(${GTEST_INCLUDE_DIRS})

add_subdirectory(regression)
add_subdirectory(unit)
