cmake_minimum_required(VERSION 2.8)

message(STATUS "=====================================================")
message(STATUS "configuring build for the simple balancing simulation")
message(STATUS "=====================================================")

project(disneysimple)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-enum-compare")
message(STATUS "CXX_FLAGS = ${CMAKE_CXX_FLAGS}")


message(STATUS "=====================================================")
message(STATUS "configuring dependancies...")
message(STATUS "=====================================================")

find_package(OpenGL)
if (OPENGL_FOUND)
message(STATUS "OPENGL FOUND")
else (OPENGL_FOUND)
message(STATUS "OPENGL NOT FOUND")
endif ()

find_package(GLUT)
if (GLUT_FOUND)
  message(STATUS "GLUT FOUND")
  message(STATUS "GLUT_LIBRARIES = ${GLUT_LIBRARIES}")
else (GLUT_FOUND)
  message(STATUS "GLUT NOT FOUND")
endif ()

find_package( Boost COMPONENTS date_time system filesystem thread chrono log log_setup)
message(STATUS "Boost_INCLUDE_DIR =  ${Boost_INCLUDE_DIRS}")
message(STATUS "Boost_LIBRARIES =  ${Boost_LIBRARIES}")
include_directories(${Boost_INCLUDE_DIRS})

find_package(Qt4 4.6.0 REQUIRED QtCore QtGui QtXml QtOpenGL)
include(${QT_USE_FILE})
message(STATUS "QT_USE_FILE    = ${QT_USE_FILE}")
message(STATUS "QT_INCLUDES    = ${QT_INCLUDES}")
message(STATUS "QT_LIBRARIES   = ${QT_LIBRARIES}")

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package(Eigen REQUIRED)
message(STATUS "Eigen_INCLUDE_DIRS = ${Eigen_INCLUDE_DIRS}")
include_directories(${Eigen_INCLUDE_DIRS})

message(STATUS "=====================================================")
message(STATUS "configuring the project")
message(STATUS "=====================================================")

file(GLOB disneysimple_srcs "*.cpp")
file(GLOB disneysimple_hdrs "*.h")

qt4_wrap_cpp(disneysimple_moc_files window.h glwidget.h)

add_executable(disneysimple
  ${disneysimple_srcs} ${disneysimple_hdrs} ${disneysimple_moc_files}
  )

target_link_libraries(disneysimple ${OPENGL_LIBRARIES})
target_link_libraries(disneysimple ${GLUT_LIBRARIES})
target_link_libraries(disneysimple ${QT_LIBRARIES})
target_link_libraries(disneysimple ${Boost_LIBRARIES})
target_link_libraries(disneysimple glog)

set_target_properties(disneysimple PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

