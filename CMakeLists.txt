cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE "Release")

message(STATUS "=====================================================")
message(STATUS "configuring build for the simple balancing simulation")
message(STATUS "=====================================================")

#####################################################################
#           Explicit macro setup for debug configuration
#####################################################################
# enable or disable debugging, default is Release
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "Release")
ENDIF(NOT CMAKE_BUILD_TYPE)

message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")

IF(CMAKE_BUILD_TYPE MATCHES "Release")
  message(STATUS "Set the release mode")
  ADD_DEFINITIONS(-DNDEBUG )
  ADD_DEFINITIONS(-DBOOST_UBLAS_NDEBUG )
ENDIF (CMAKE_BUILD_TYPE MATCHES "Release")

project(disneylearning)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-enum-compare")
message(STATUS "CXX_FLAGS = ${CMAKE_CXX_FLAGS}")


file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/glog")
message(STATUS "Create ${CMAKE_BINARY_DIR}/glog directory")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/captures")
message(STATUS "Create ${CMAKE_BINARY_DIR}/captures directory")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/captures")
message(STATUS "Copy ${CMAKE_SOURCE_DIR}/app/icon.png ${CMAKE_BINARY_DIR}")
file(COPY ${CMAKE_SOURCE_DIR}/app/icon.png DESTINATION ${CMAKE_BINARY_DIR})

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

find_package( Boost COMPONENTS date_time system filesystem thread chrono log log_setup program_options signals serialization)
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

find_package(TBB REQUIRED)
message(STATUS "TBB_INCLUDE_DIRS = ${TBB_INCLUDE_DIRS}")
message(STATUS "TBB_LIBRARIES = ${TBB_LIBRARIES}")

message(STATUS "=====================================================")
message(STATUS "configuring the project")
message(STATUS "=====================================================")

include_directories(".")

file(GLOB disneylearning_utl_srcs "utils/*.cpp")
file(GLOB disneylearning_utl_hdrs "utils/*.h")
file(GLOB disneylearning_sim_srcs "simulation/*.cpp")
file(GLOB disneylearning_sim_hdrs "simulation/*.h")
file(GLOB disneylearning_lrn_srcs "learning/*.cpp")
file(GLOB disneylearning_lrn_hdrs "learning/*.h")
file(GLOB disneylearning_app_srcs "app/*.cpp")
file(GLOB disneylearning_app_hdrs "app/*.h")


# file(GLOB disneylearning_srcs "*.cpp")
# file(GLOB disneylearning_hdrs "*.h")

qt4_wrap_cpp(disneylearning_moc_files app/Window.h app/GLWidget.h)
message(STATUS "qt4_wrap_cpp expands the moc files: ${disneylearning_moc_files}")

add_executable(disneylearning
  ${disneylearning_utl_srcs} ${disneylearning_utl_hdrs}
  ${disneylearning_sim_srcs} ${disneylearning_sim_hdrs}
  ${disneylearning_lrn_srcs} ${disneylearning_lrn_hdrs}
  ${disneylearning_app_srcs} ${disneylearning_app_hdrs}
  ${disneylearning_moc_files}
  )

target_link_libraries(disneylearning ${OPENGL_LIBRARIES})
target_link_libraries(disneylearning ${GLUT_LIBRARIES})
target_link_libraries(disneylearning ${QT_LIBRARIES})
target_link_libraries(disneylearning ${Boost_LIBRARIES})
target_link_libraries(disneylearning glog)
target_link_libraries(disneylearning shark)
target_link_libraries(disneylearning Box2D)
# target_link_libraries(disneylearning GPc gfortran blas lapack)
target_link_libraries(disneylearning gp)
target_link_libraries(disneylearning ${TBB_LIBRARIES})
target_link_libraries(disneylearning tinyxml2)
target_link_libraries(disneylearning nlopt)


set_target_properties(disneylearning PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

