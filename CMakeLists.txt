cmake_minimum_required(VERSION 3.0.0)
project(FastARAP VERSION 0.1.0)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(PYBIND_ACTIVE TRUE)

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

if (PYBIND_ACTIVE)
	find_package(PythonInterp REQUIRED)
	include_directories(${PYTHON_INCLUDE_DIRS})
	add_subdirectory(pybind11)
	pybind11_add_module(FastARAP SHARED ARAP.cpp FastARAP.cpp)
endif()

if (NOT PYBIND_ACTIVE)
	add_executable(FastARAP ${HEADER_FILES} ${SOURCE_FILES})
endif()

set_property (TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
target_include_directories(FastARAP PRIVATE ${PROJECT_SOURCE_DIR})
target_link_libraries(FastARAP PRIVATE Eigen3::Eigen)
#set_property(TARGET ${SOURCE_FILES} PROPERTY POSITION_INDEPENDENT_CODE ON)

#Ceres
#find_package(Ceres REQUIRED)

#set_property (TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)

#target_link_libraries(Vocalfold Eigen3::Eigen ${CERES_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${OPENMESH_LIBRARIES})

