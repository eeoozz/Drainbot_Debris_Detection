set (CMAKE_CXX_STANDARD 11)
cmake_minimum_required(VERSION 2.8.9)
project(obstacle_detectv7)
set(CMAKE_BUILD_TYPE Release)

#Bring the headers into the project
include_directories(include)

#However, the file(GLOB...) allows for wildcard additions:
file(GLOB SOURCES "src/*.cpp")


find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package( OpenCV REQUIRED )

#Generate the shared library from the sources
add_library(obstacle_detectv7 SHARED ${SOURCES})

#Set the location for library installation -- i.e., /usr/lib in this case
# not really necessary in this example. Use "sudo make install" to apply
install(TARGETS obstacle_detectv7 DESTINATION /usr/lib)
target_link_libraries (obstacle_detectv7 ${PCL_LIBRARIES} ${OpenCV_LIBS})
