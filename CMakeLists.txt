cmake_minimum_required (VERSION 2.6)

project (robot_environment6dof)

if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_DEBUG "-g -std=c++11 -Og -DEIGEN_NO_DEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++11 -O3 -DEIGEN_NO_DEBUG -flto")
SET(BOOST_MIN_VERSION "1.54.0")
SET(PYTHON_MIN_VERSION "2.7.0")
include(FindPkgConfig)
include(GNUInstallDirs)

set(INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/include")
############################# LOAD PYTHON #############################
if(PKG_CONFIG_FOUND)    
    pkg_check_modules(PYTHON python-2.7)
    if(NOT PYTHON_FOUND)
       message(FATAL_ERROR "Python could not be found")
    endif()
    include_directories(${PYTHON_INCLUDE_DIRS})
    link_directories(${PYTHON_LIBRARY_DIRS})    
endif()

############################# LOAD BOOST #############################
find_package(Boost ${BOOST_MIN_VERSION} REQUIRED COMPONENTS python system thread timer random)
if (NOT Boost_FOUND)
    set(Boost_INCLUDE_DIRS "${BOOST_INCLUDE_DIRS}")
    IF ( NOT Boost_INCLUDE_DIRS )
        message(FATAL_ERROR "Please point the environment variable BOOST_INCLUDE_DIRS to the include directory of Boost")
    ENDIF()
endif ()
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

############################# LOAD FCL #############################
if(PKG_CONFIG_FOUND)
    pkg_check_modules(FCL fcl)
    if(NOT FCL_FOUND)
       message(FATAL_ERROR "FCL could not be found")
    endif()
    include_directories(${FCL_INCLUDE_DIRS})
    link_directories(${FCL_LIBRARY_DIRS}) 
    message("-- FCL LIB DIRS ${FCL_LIBRARY_DIRS}")    
endif()

############################# LOAD EIGEN #############################
if(PKG_CONFIG_FOUND)
    pkg_check_modules(EIGEN eigen3)
    if(NOT EIGEN_FOUND)
       message(FATAL_ERROR "EIGEN could not be found")
    endif()
    include_directories(${EIGEN_INCLUDE_DIRS})
    link_directories(${EIGEN_LIBRARY_DIRS})    
endif()

############################# LOAD TINYXML #############################
if(PKG_CONFIG_FOUND)    
    pkg_check_modules(TINYXML tinyxml)
    if(NOT TINYXML_FOUND)
       message(FATAL_ERROR "TINYXML could not be found")
    endif()
    include_directories(${TINYXML_INCLUDE_DIRS}) 
    link_directories(${TINYXML_LIBRARY_DIRS})     
endif()

############################# LOAD ROBOTS #############################
if(PKG_CONFIG_FOUND)    
    pkg_check_modules(ROBOTS robots6dof)
    if(NOT ROBOTS_FOUND)
       message(FATAL_ERROR "ROBOTS could not be found")
    endif()
    include_directories(${ROBOTS_INCLUDE_DIRS})
    link_directories(${ROBOTS_LIBRARY_DIRS})     
endif()

add_library(robot_environment6dof SHARED 
            ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_environment.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/Obstacle.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/Terrain.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/BoxObstacle.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/SphereObstacle.cpp)
            
target_link_libraries (robot_environment6dof
                       ${Boost_LIBRARIES}
                       ${PYTHON_LIBRARIES}                       
                       ${FCL_LIBRARIES}
                       ${TINYXML_LIBRARIES}
                       ${ROBOTS_LIBRARIES})
                       
set(pkg_conf_file_in "${CMAKE_CURRENT_SOURCE_DIR}/robot_environment6dof.pc.in")
set(pkg_conf_file_out "${CMAKE_CURRENT_BINARY_DIR}/robot_environment6dof.pc")
configure_file("${pkg_conf_file_in}" "${pkg_conf_file_out}" @ONLY)

install(TARGETS robot_environment6dof DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(FILES ${INCLUDE_DIR}/robot_environment.hpp  DESTINATION include/robot_environment)
install(FILES ${INCLUDE_DIR}/Obstacle.hpp  DESTINATION include/robot_environment)
install(FILES ${INCLUDE_DIR}/BoxObstacle.hpp  DESTINATION include/robot_environment)
install(FILES ${INCLUDE_DIR}/SphereObstacle.hpp  DESTINATION include/robot_environment)
install(FILES ${INCLUDE_DIR}/Terrain.hpp  DESTINATION include/robot_environment)
install(FILES "${pkg_conf_file_out}" DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig/ COMPONENT pkgconfig)
