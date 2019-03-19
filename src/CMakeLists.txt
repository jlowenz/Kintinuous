cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

project(Kintinuous LANGUAGES CXX CUDA)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")

set(CMAKE_CXX_STANDARD 14)

# macro(REMOVE_VTK_DEFINITIONS)
#   get_directory_property(_dir_defs COMPILE_DEFINITIONS)
#   message("COMPILE DEFS: ${_dir_defs}")
#   set(_vtk_definitions)
#   set(_defs)
#   foreach(_item ${_dir_defs})
#     if(_item MATCHES "vtk*")
#       list(APPEND _vtk_definitions ${_item})
#     else()
#       list(APPEND _defs ${_item})
#     endif()
#   endforeach()
#   remove_definitions(${_vtk_definitions})
#   set_property(DIRECTORY PROPERTY COMPILE_DEFINITIONS ${_defs})
# endmacro(REMOVE_VTK_DEFINITIONS)

find_package(OpenNI2 REQUIRED)
find_package(Pangolin 0.1 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ZLIB REQUIRED)
find_package(CUDA 10.1 REQUIRED)
find_package(DLib REQUIRED)
find_package(DBoW2 REQUIRED)
find_package(DLoopDetector REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(iSAM REQUIRED)
find_package(Boost COMPONENTS thread serialization REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${ZLIB_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIR})
include_directories(${DLib_INCLUDE_DIRS})
include_directories(${DBoW2_INCLUDE_DIRS})
include_directories(${OPENNI2_INCLUDE_DIR})
include_directories(${DLoopDetector_INCLUDE_DIRS})
include_directories(${SUITESPARSE_INCLUDE_DIRS})
include_directories(${ISAM_INCLUDE_DIRS})

file(GLOB utils utils/*.cpp utils/*.h*)
file(GLOB frontend frontend/*.cpp frontend/*.h*)
file(GLOB cuda frontend/cuda/*.cu)
file(GLOB containers frontend/cuda/containers/*.cpp)
file(GLOB backend backend/*.cpp backend/*.h*)
file(GLOB pnp backend/pnp/*.cpp backend/pnp/*.h*)

set(CUDA_ARCH_BIN "30 35 50 52" CACHE STRING "Specify 'real' GPU arch to build binaries for, BIN(PTX) format is supported. Example: 1.3 2.1(1.3) or 13 21(13)")
set(CUDA_ARCH_PTX "" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")              

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})
include(CudaComputeTargetFlags.cmake)                  
APPEND_TARGET_ARCH_FLAGS()

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}  "-Xcompiler;-fPIC;--shared;--cudart=shared;")           
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ftz=true;--prec-div=false;--prec-sqrt=false") 

#CUDA_COMPILE(cuda_objs ${cuda})   

add_definitions(${CUDA_DEFINITIONS})

#Include only after compiling with nvcc, this stops vtk polluting nvcc's definitions with whitespace
find_package(PCL 1.7 QUIET REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -msse -msse2 -msse3 -O3 -std=c++11")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -std=c++11")

#REMOVE_VTK_DEFINITIONS()
add_library(util ${utils})
add_library(frontend ${frontend} ${cuda} ${containers})
target_link_libraries(frontend ${PCL_LIBRARIES}
  ${ZLIB_LIBRARY}
  ${Pangolin_LIBRARIES}
  ${Boost_LIBRARIES} 
  ${CUDA_LIBRARIES}
  ${OpenCV_LIBS}
  ${DLib_LIBS} 
  ${DBoW2_LIBS}
  ${OPENNI2_LIBRARY}
  ${SUITESPARSE_LIBRARIES}
  ${ISAM_LIBRARIES})
add_library(backend ${backend} ${pnp})

message("Sources: ${srcs}")

add_executable(Kintinuous  MainController.cpp Kintinuous.cpp PangoVis.cpp)
add_executable(KintinuousRW  MainControllerRW.cpp KintinuousRW.cpp PangoVisRW.cpp)
add_executable(tdp_test tdp_test.cpp)

add_library(lvtest vtest.cpp)
add_executable(vtest vtest_main.cpp)

target_link_libraries(vtest lvtest ${Boost_LIBRARIES})

target_link_libraries(tdp_test
  util
  frontend
  backend
  ${PCL_LIBRARIES}
  ${ZLIB_LIBRARY}
  ${Pangolin_LIBRARIES}
  ${Boost_LIBRARIES} 
  ${CUDA_LIBRARIES}
  ${OpenCV_LIBS}
  ${DLib_LIBS} 
  ${DBoW2_LIBS}
  ${OPENNI2_LIBRARY}
  ${SUITESPARSE_LIBRARIES}
  ${ISAM_LIBRARIES}
  )
             
target_link_libraries(Kintinuous
                      util
                      frontend
                      backend
                      ${PCL_LIBRARIES}
                      ${ZLIB_LIBRARY}
                      ${Pangolin_LIBRARIES}
                      ${Boost_LIBRARIES} 
                      ${CUDA_LIBRARIES}
                      ${OpenCV_LIBS}
                      ${DLib_LIBS} 
                      ${DBoW2_LIBS}
                      ${OPENNI2_LIBRARY}
                      ${SUITESPARSE_LIBRARIES}
                      ${ISAM_LIBRARIES}
                      )

target_link_libraries(KintinuousRW
                      util
                      frontend
                      backend
                      ${PCL_LIBRARIES}
                      ${ZLIB_LIBRARY}
                      ${Pangolin_LIBRARIES}
                      ${Boost_LIBRARIES} 
                      ${CUDA_LIBRARIES}
                      ${OpenCV_LIBS}
                      ${DLib_LIBS} 
                      ${DBoW2_LIBS}
                      ${OPENNI2_LIBRARY}
                      ${SUITESPARSE_LIBRARIES}
                      ${ISAM_LIBRARIES}
                      )
                   
install(TARGETS Kintinuous KintinuousRW RUNTIME DESTINATION bin)
