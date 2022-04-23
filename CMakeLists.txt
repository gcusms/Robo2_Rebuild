cmake_minimum_required(VERSION 3.0.0)

project(Robocon_2022_CV_R2)

add_executable(${PROJECT_NAME} wolf_r2.cpp)
add_definitions(-DAPI_EXPORTS)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O0 -g -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED -pthread -lstdc++")


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#Realsense
find_package(realsense2 REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE ${realsense2_LIBRARY})


#("----------------------OPENVINO----------------------")
include_directories(
    #OpenVINO推理引擎的头文件
    ~/intel/openvino_2021/deployment_tools/inference_engine/include/
    ~/intel/openvino_2021/deployment_tools/ngraph/include/
)

#查找必要的依赖包
find_package(OpenCV REQUIRED)
set(InferenceEngine_DIR "~/intel/openvino_2021/deployment_tools/inference_engine/share")
find_package(InferenceEngine)
set(ngraph_DIR "~/intel/openvino_2021/deployment_tools/ngraph")
# find_package(ngraph REQUIRED)
set(ngraph_LIBRARIES "~/intel/openvino_2021/deployment_tools/ngraph/lib/libngraph.so")
set(ngraph_INCLUDE_DIRS "~/intel/openvino_2021/deployment_tools/ngraph/include/")



# 编译detector.so black_box
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/infer)
add_library (detector STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/infer/detector.cpp)
target_include_directories(detector
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
        PRIVATE ${OpenCV_INCLUDE_DIR}
        PRIVATE ${InferenceEngine_INCLUDE_DIRS}
        # PUBLIC ${ngraph_INCLUDE_DIRS}
)
target_link_libraries(detector
        ${InferenceEngine_LIBRARIES}
        # ${ngraph_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME} 
        PRIVATE
        detector
        )    

#("----------------------FMT----------------------")
#fmt
find_package(fmt REQUIRED)
target_link_libraries(${PROJECT_NAME} 
                        PRIVATE
                        fmt::fmt)




#("----------------------WOLF_VISION----------------------")
include_directories(
 ${CMAKE_SOURCE_DIR}/include/wolfVision
 ${CMAKE_SOURCE_DIR}/include/streamer
 ${CMAKE_SOURCE_DIR}/include/infer
 ${CMAKE_SOURCE_DIR}/include/slovePnp
 ${CMAKE_SOURCE_DIR}/pluglibrary
)

#("----------------------SERIAL----------------------")
# new serial
set(serial_SRCS
    devices/serial/src/serial.cc
    devices/serial/src/impl/unix.cc
    devices/serial/src/impl/list_ports/list_ports_linux.cc
    devices/serial/serial/serial.h
    devices/serial/serial/v8stdint.h
)
include_directories(devices/serial)
include_directories(devices/serial/src)

add_library(serial 
        STATIC
        ${serial_SRCS})
target_link_libraries(${PROJECT_NAME} 
                    PRIVATE
                    serial)

# ("************************OPENCV****************************")
#opencv
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )


#("----------------------LIBARARY_ADD----------------------")
add_library(wolfvision  ${CMAKE_SOURCE_DIR}/src/wolfvision.cpp)
add_library(detect  ${CMAKE_SOURCE_DIR}/src/infer/detector.cpp)

target_link_libraries(${PROJECT_NAME}
                        PRIVATE
                        wolfvision
                        detect
                        ${OpenCV_LIBS}
                        )



#("----------------------LIBARARY_ADD----------------------")
# FILE(GLOB_RECURSE TEST_SRC "${CMAKE_SOURCE_DIR}/test/*.cpp" "${CMAKE_SOURCE_DIR}/test/*.h")
# add_executable(TEST ${TEST_SRC})
# target_link_libraries(TEST
#                       PRIVATE
#                       test)
enable_testing()


#//TEST/
set(TEST_MODE 1)
if(${TEST_MODE})
set(PROJECTNAME_TEST Test)
add_executable(${PROJECTNAME_TEST} 
                ${CMAKE_SOURCE_DIR}/test/streamer_test.cpp
                )       

target_link_libraries(${PROJECTNAME_TEST}
                PRIVATE
                ${OpenCV_LIBS}
                ${realsense2_LIBRARY}
                )
endif()