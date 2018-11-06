cmake_minimum_required ( VERSION 2.6 FATAL_ERROR)   
project(pcd2bin)                                        #建立的工程名
                                                                      
find_package(PCL 1.3 REQUIRED)  

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARIES_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(pcd2bin pcd2bin.cpp)

target_link_libraries(pcd2bin ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})