cmake_minimum_required(VERSION 3.0.0)

project(Robocon_2022_CV_R2)

add_executable(${PROJECT_NAME} wolf_r2.cpp)
add_definitions(-DAPI_EXPORTS)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O0 -g -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED -pthread")


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#("----------------------OPENVINO----------------------")
include_directories(
    #OpenVINO推理引擎的头文件
    /home/sms/intel/openvino_2021/deployment_tools/inference_engine/include/
    /home/sms/intel/openvino_2021/deployment_tools/ngraph/include/
    )
set(InferenceEngine_DIR "/home/sms/intel/openvino_2021/deployment_tools/inference_engine/share")
find_package(InferenceEngine)
set(ngraph_DIR "/home/sms/intel/openvino_2021/deployment_tools/ngraph")
find_package(ngraph REQUIRED)
set(ngraph_LIBRARIES "/home/sms/intel/openvino_2021/deployment_tools/ngraph/lib/libngraph.so")
set(ngraph_INCLUDE_DIRS "/home/sms/intel/openvino_2021/deployment_tools/ngraph/include/")

# # 编译detector.so black_box
add_library (detector SHARED ${CMAKE_SOURCE_DIR}/src/infer/detector.cpp)


#("----------------------FMT----------------------")
#fmt
find_package(fmt REQUIRED)
target_link_libraries(Robocon_2022_CV_R2 
                        PRIVATE
                        fmt::fmt)


#("----------------------realsense2----------------------")           
#Realsense
find_package(realsense2 REQUIRED)
target_link_libraries(Robocon_2022_CV_R2 
                PRIVATE
                ${realsense2_LIBRARY})


#("----------------------WOLF_VISION----------------------")
include_directories(
 ${CMAKE_SOURCE_DIR}/include/wolfVision
 ${CMAKE_SOURCE_DIR}/include/streamer
 ${CMAKE_SOURCE_DIR}/include/infer
 ${CMAKE_SOURCE_DIR}/include/slovePnp
 ${CMAKE_SOURCE_DIR}/plublibrary
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
add_library(serial ${serial_SRCS})
target_link_libraries(${PROJECT_NAME} 
                    PRIVATE
                    serial)


#("----------------------LIBARARY_ADD----------------------")
add_library(wolfvision  ${CMAKE_SOURCE_DIR}/src/wolfvision.cpp)
add_library(detect  ${CMAKE_SOURCE_DIR}/src/infer/detector.cpp)


target_link_libraries(${PROJECT_NAME}
                         PRIVATE
                         wolfvision
                         detect
                      )
target_include_directories(detector
          PUBLIC 
          ${CMAKE_CURRENT_SOURCE_DIR}
          ${OpenCV_INCLUDE_DIR}
          ${InferenceEngine_INCLUDE_DIRS}
          ${ngraph_INCLUDE_DIRS}
          ${InferenceEngine_LIBRARIES}
          ${ngraph_LIBRARIES}
          )
target_link_libraries(${PROJECT_NAME} 
          PRIVATE
          detector
)  


