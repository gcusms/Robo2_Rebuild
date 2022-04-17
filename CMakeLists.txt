cmake_minimum_required(VERSION 3.0.0)

project(Robocon_2022_CV_R2)

add_definitions(-DAPI_EXPORTS)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O0 -g -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED -pthread")
# add_definitions(-DRELEASE)

add_executable(Robocon_2022_CV_R2 main.cpp)
#include the hpp
include_directories(${PROJECT_SOURCE_DIR}/streamer)
include_directories(${PROJECT_SOURCE_DIR}/solvePnp)
include_directories(${PROJECT_SOURCE_DIR}/infer)
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/wolfVisionR2)

include_directories(
    #OpenVINO推理引擎的头文件
    /home/sms/intel/openvino_2021/deployment_tools/inference_engine/include/
    /home/sms/intel/openvino_2021/deployment_tools/ngraph/include/
)
#查找必要的依赖包
#OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} 
                        PRIVATE
                        ${OpenCV_LIBS})

set(InferenceEngine_DIR "/home/sms/intel/openvino_2021/deployment_tools/inference_engine/share")
find_package(InferenceEngine)
set(ngraph_DIR "/home/sms/intel/openvino_2021/deployment_tools/ngraph")
find_package(ngraph REQUIRED)
set(ngraph_LIBRARIES "/home/sms/intel/openvino_2021/deployment_tools/ngraph/lib/libngraph.so")
set(ngraph_INCLUDE_DIRS "/home/sms/intel/openvino_2021/deployment_tools/ngraph/include/")

# # 编译detector.so black_box
add_library (detector SHARED ${CMAKE_CURRENT_SOURCE_DIR}/infer/detector.cpp)
target_include_directories(detector
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
        PUBLIC ${OpenCV_INCLUDE_DIR}
        PUBLIC ${InferenceEngine_INCLUDE_DIRS}
        PUBLIC ${ngraph_INCLUDE_DIRS}
)
target_link_libraries(detector
        PRIVATE
        ${InferenceEngine_LIBRARIES}
        ${ngraph_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME} 
            PRIVATE
            detector
                )    



#Realsense
find_package(realsense2 REQUIRED)
target_link_libraries(Robocon_2022_CV_R2 
                PRIVATE
                ${realsense2_LIBRARY})



#fmt
find_package(fmt REQUIRED)
target_link_libraries(Robocon_2022_CV_R2 
                        PRIVATE
                        fmt::fmt)

# new serial
set(serial_SRCS
    devices/serial/src/serial.cc
    devices/serial/src/impl/unix.cc
    devices/serial/src/impl/list_ports/list_ports_linux.cc
    devices/serial/serial/serial.h
    devices/serial/serial/v8stdint.h
)
include_directories(devices/serial)
add_library(serial ${serial_SRCS})
target_link_libraries(${PROJECT_NAME} 
                    PRIVATE
                    serial)

set(INFER_DIR 
    infer/detector.cpp)
add_library(infer_get ${INFER_DIR})
target_link_libraries(${PROJECT_NAME} 
                    PRIVATE
                    infer_get)


set(CONFIG_FILE_PATH ${PROJECT_SOURCE_DIR}/configs)
set(SOURCE_PATH ${PROJECT_SOURCE_DIR})
target_compile_definitions(Robocon_2022_CV_R2 PRIVATE "CONFIG_FILE_PATH=\"${CONFIG_FILE_PATH}\"")
target_compile_definitions(Robocon_2022_CV_R2 PRIVATE "SOURCE_PATH=\"${PROJECT_SOURCE_DIR}\"")



