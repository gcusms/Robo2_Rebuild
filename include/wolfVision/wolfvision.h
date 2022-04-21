#include <iostream>
#include <thread>
#include <chrono>
#include <exception>
#include <memory>

// #include "cv-helpers.hpp"
#include "mjpeg_streamer.hpp"
#include "serial.hpp"
// for convenience
#include "opencv4/opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "librealsense2/rs.hpp"
#include "detector.h"
using MJPEGStreamer = nadjieb::MJPEGStreamer;
using namespace rs2;
using namespace drawspace;

struct DetectObject
{
 int cube_middle_detect_times{0};
 int cude_front_detect_times{0};
 const float error_angle{2.5f};
 const float error_distance{150.5f};
}__attribute__((packed));

class WolfRobot_R2
{
private:
 /* data */
public:
 WolfRobot_R2();
 ~WolfRobot_R2();
 void serialReceive();  // receive the serial messange
 void serialSend();     // send message for the serial 
 void detectionObject(pipeline &pipe, pipeline_profile &profile);      // detect the object
private:
 bool node_judge_ = false;
 bool detect_judge_ = false;
 RoboInf robo_inf;
 RoboCmd robo_cmd;
 std::unique_ptr<RoboSerial> serial_;
private:
 cv::Mat src_img;
 
 DetectObject detect_parameters;  // 控制参数
 
 Detector *detector = nullptr ;  // 检测对象声明

 vector<Detector::Object> detected_objects;  // 检测结果存储写入对象声明

public:
 const bool nodeJudgeReturn(){return this->node_judge_;}

};

