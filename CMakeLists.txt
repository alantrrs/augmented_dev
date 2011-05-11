cmake_minimum_required(VERSION 2.4.6)

#just to avoid the warning
if(COMMAND cmake_policy)
     cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

#set project name
project(STANDALONE)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
#PCL
find_package(PCL 1.0 REQUIRED)
#OpenNI
find_package(OpenNI REQUIRED)
#OpenCV
find_package(OpenCV REQUIRED)
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#set the include directory
include_directories (
		${PROJECT_SOURCE_DIR}/include
		${OpenCV_INCLUDE_DIRS}
		${OPENNI_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
					)
#add a library to the project
add_library(myfuncs src/myfuncs.cpp)
add_library(Xn_sensor src/Xn_sensor.cpp)
#add_library(poseestimator src/poseestimator.cpp)
#add an executable
add_executable(sensor_viewer src/sensor_viewer.cpp)
#add_executable(model_extractor src/model_extractor.cpp)
add_executable(example src/example.cpp)

#link the libraries
target_link_libraries(sensor_viewer Xn_sensor ${OpenCV_LIBS} ${OPENNI_LIBRARY})
#target_link_libraries(model_extractor Xn_sensor myfuncs poseestimator ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_link_libraries(example Xn_sensor ${PCL_LIBRARIES})

