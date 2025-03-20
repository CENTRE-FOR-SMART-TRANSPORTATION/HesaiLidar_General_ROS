cmake_minimum_required(VERSION 2.8)
project(ImuSDK)

find_package( Boost REQUIRED  COMPONENTS thread)

# set (CMAKE_CXX_FLAGS "-fPIC --std=c++11")
ADD_COMPILE_OPTIONS(-std=c++14 )

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_library( ${PROJECT_NAME} SHARED
    src/imuSDK.cc
)

target_link_libraries(${PROJECT_NAME} 
    ${Boost_LIBRARIES}
    Boost::thread
)

# Install headers
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
install(TARGETS ImuSDK DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

endif(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})

