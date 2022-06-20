cmake_minimum_required(VERSION 3.0.0)
project(FastARAP VERSION 0.1.0)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

if(MSVC)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++17")
else()
	set(CMAKE_CXX_STANDARD 17)
endif(MSVC)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(include)
file(GLOB HEADER_FILES ${PROJECT_SOURCE_DIR}/*.h )
file(GLOB SOURCE_FILES ${PROJECT_SOURCE_DIR}/*.cpp )

add_compile_definitions(_USE_MATH_DEFINES)

message(STATUS "Resource Files: " ${SOURCE_FILES})
message(STATUS "Header Files: " ${HEADER_FILES})

#EIGEN
find_package(Eigen3 REQUIRED)

#OpenMesh
if(UNIX)
	list (APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
endif()
find_package(OpenMesh REQUIRED)

find_package(PythonInterp REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

add_subdirectory(pybind11)

#add_executable(Test ${HEADER_FILES} ${SOURCE_FILES} ${UI_FILES} ${RESOURCE_FILES})
#set_property (TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)

#target_include_directories(FastARAP PUBLIC ${PROJECT_SOURCE_DIR} ${CERES_INCLUDE_DIRS} ${OPENMESH_INCLUDE_DIRS})

pybind11_add_module(FastARAP SHARED BaseObject.cpp ARAPMeshDeformation.cpp FastARAP.cpp)
#add_executable(FastARAP ${HEADER_FILES} ${SOURCE_FILES})


set_property (TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
target_include_directories(FastARAP PRIVATE ${PROJECT_SOURCE_DIR} ${OPENMESH_INCLUDE_DIRS})
target_link_libraries(FastARAP PRIVATE Eigen3::Eigen ${OPENMESH_LIBRARIES} )
#set_property(TARGET ${SOURCE_FILES} PROPERTY POSITION_INDEPENDENT_CODE ON)

#Ceres
#find_package(Ceres REQUIRED)

#add_executable(Vocalfold ${HEADER_FILES} ${SOURCE_FILES})

#set_property (TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)

#target_link_libraries(Vocalfold Eigen3::Eigen ${CERES_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${OPENMESH_LIBRARIES})

