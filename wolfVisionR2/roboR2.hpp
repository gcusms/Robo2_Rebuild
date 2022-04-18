#include <iostream>
#include <librealsense2/rs.hpp>
#include <fmt/core.h>
#include <opencv2/opencv.hpp>

#include "devices/serial/serial.hpp"
#include "solvePnP/solvePnP.hpp"
#include "streamer/mjpeg_streamer.hpp"
#include "streamer/streamer_impl.hpp"
#include "infer/detector.h"


#include "../utils.hpp"
#include "../cv-helpers.hpp"

using namespace std;
using namespace cv;
using namespace rs2;


static int cube_middle_detect_times{0};
static int cude_front_detect_times{0};

constexpr int cube_1_min_area = 15000;
constexpr int cube_1_max_area = 20000;

constexpr int cube_2_min_area = 30000;
constexpr int cube_2_max_area = 39000;

constexpr int cube_3_min_area = 43800;
constexpr int cube_3_max_area = 51000;

constexpr int cube_4_min_area = 68000;
constexpr int cube_4_max_area = 75500;

constexpr int cube_5_min_area = 79000;
constexpr int cube_5_max_area = 90000;

constexpr float cube_target_yaw_angle_offset = -2.f;
constexpr float cube_target_distance_offset = 130.5f;
constexpr float cube_target_yaw_angle_errors_range = 2.0f;
constexpr float cube_target_distance_errors_range = 2.5f;
constexpr int cube_targeted_detect_flag_times = 5;
constexpr int cube_target_echo_uart_cmd_sleep_time = 20000;

class RobotR2{
public:
 RobotR2();
 void spin();
 void uartWrite();
 void uartRead();
 void cubeDetect();
 void stop();
 ~RobotR2();
private:
 bool end_node_ = bool{false};

private:
 RoboInf robo_inf;
 std::unique_ptr<RoboStreamer> streamer_;    
 std::unique_ptr<RoboSerial> serial_robot_;

private:
    Detector *detector;          // 对象检测
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};

private:
    std::string _input_name;  
    //参数区
    std::string _xml_path = "../model/best.xml";       // OpenVINO模型xml文件路径
    std::string _bin_path =  "../model/best.bin";       // OpenVINO模型bin文件路径
    float _cof_threshold = 0.4f;       // 置信度阈值,计算方法是框置信度乘以物品种类置信度
    float _nms_area_threshold = 0.5f;  // nms最小重叠面积阈值
private:    
    pipeline pipe;               // 创建数据管道
};



RobotR2::RobotR2(){
 this->detector = new Detector(_xml_path,_bin_path,_cof_threshold,_nms_area_threshold);
 serial_robot_ = std::make_unique<RoboSerial>("/dev/ros_tty", 115200);
 streamer_ = std::make_unique<RoboStreamer>();
}

void RobotR2::cubeDetect()
{
 cv::Mat src_img;
 usleep(1);
 rs2::align align_to(RS2_STREAM_COLOR);
 pipeline_profile profile = pipe.start();
 cv::Point2f pnp_angle;

 while (true)try
 {

  // 堵塞程序直到新的一帧捕获
  frameset frameset = pipe.wait_for_frames();

  // 获取颜色图
  rs2::video_frame video_src = frameset.get_color_frame();

  // 获取深度图
  rs2::depth_frame depth_src = frameset.get_depth_frame();
  const int color_width = video_src.as<video_frame>().get_width();
  const int color_height = video_src.as<video_frame>().get_height();

  src_img = cv::Mat(Size(color_width, color_height), CV_8UC3,
             (void *)video_src.get_data(), Mat::AUTO_STEP);
  vector<Detector::Object> detected_objects;

  auto t = (double)cv::getTickCount();
  if(!detector->process_frame(src_img, detected_objects))
  {
    cv::cvtColor(src_img,src_img,COLOR_BGR2RGB);
    if (!src_img.empty()) {
      cv::imshow("interface", src_img);
      cv::waitKey(1);
    }
    continue;
  }
  // convert image
  cv::cvtColor(src_img,src_img,COLOR_BGR2RGB);
  // sor the cube_vector
  std::sort(detected_objects.begin(), detected_objects.end());
  cv::Rect temp_rect_ = detected_objects.at(0).rect;
  switch (robo_inf.catch_cube_mode_status.load())
  {
/****************angle*********************************/
    case CatchMode::spin:
    {
      cv::putText(src_img,
              "spin mode",
              cv::Point(50, 50),
              cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 150, 255), 1);
      pnp_angle.y = src_img.cols * 0.57 - (temp_rect_ .x + temp_rect_.width * 0.5);
      if (cube_middle_detect_times < cube_targeted_detect_flag_times)
      {
        if (pnp_angle.y < cube_target_yaw_angle_errors_range &&
            pnp_angle.y > -cube_target_yaw_angle_errors_range)
        {
          cube_middle_detect_times++;
        }
        else
        {
          RoboSpinCmdUartBuff uart_temp_spin_cmd;
          uart_temp_spin_cmd.yaw_angle = pnp_angle.y;
          std::cout << "uart_temp_spin_cmd.yaw_angle" << uart_temp_spin_cmd.yaw_angle << "\n";
          serial_robot_->write((uint8_t *)&uart_temp_spin_cmd, sizeof(uart_temp_spin_cmd));
        }
      }
      else
      {
        RoboSpinCmdUartBuff uart_temp_spin_cmd;
        uart_temp_spin_cmd.yaw_angle = 0.f;
        for (int i = 0; i < 3; i++)
        {
          serial_robot_->write((uint8_t *)&uart_temp_spin_cmd, sizeof(uart_temp_spin_cmd));
          std::cout << "send yaw 0\n";
          usleep(cube_target_echo_uart_cmd_sleep_time);
        }
        cube_middle_detect_times = 0;
        robo_inf.catch_cube_mode_status.store(CatchMode::go);
      }
      break;
    }

/***************direct********************************/
    case CatchMode::go:
    {
      cv::putText(src_img,
                  "go straight mode",
                  cv::Point(50, 50),
                  cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 150, 255), 1);
      // judge the cube if in the center of frame
      // sand the message in ten times
      if (cude_front_detect_times < cube_targeted_detect_flag_times)
      {
        cv::Rect object_rect(temp_rect_.x + temp_rect_.width *0.5 - 50,
                             temp_rect_.y + temp_rect_.height - 100,
                             100, 100); // 底部与目标重叠且居中，固定大小的 temp_rect_ 用以 pnp

        float select_cube_dis = cube_target_distance_offset - pnp_angle.x;
        // if the angles in around the range
        if (select_cube_dis < cube_target_distance_errors_range &&
            select_cube_dis > -cube_target_distance_errors_range)
        {
          cude_front_detect_times++;
        }
        else
        {
          RoboGoCmdUartBuff uart_temp_go_cmd;
          uart_temp_go_cmd.distance = select_cube_dis;
          std::cout << "uart_temp_go_cmd.distance:" << uart_temp_go_cmd.distance << "\n";
          serial_robot_->write((uint8_t *)&uart_temp_go_cmd, sizeof(uart_temp_go_cmd));
        }
      }
      else
      {
        // send  0 meters message for the Electronically controlled then the robo ready to  catch the cube for three time
        RoboGoCmdUartBuff uart_temp_go_cmd;
        uart_temp_go_cmd.distance = 0.f;
        std::cout << "cathch_mode" << std::endl;
        for (int i = 0; i < 3; i++)
        {
          serial_robot_->write((uint8_t *)&uart_temp_go_cmd, sizeof(uart_temp_go_cmd));
          std::cout << "send stop \n";
          usleep(cube_target_echo_uart_cmd_sleep_time);
        }
        robo_inf.catch_cube_mode_status.store(CatchMode::catch_cube);
        cude_front_detect_times = 0;
      }
      break;
    }

/***************catch******************************/
    case CatchMode::catch_cube:
    {
      std::cout << temp_rect_.area() << std::endl;
      cv::putText(src_img,
                  "catch cube mode",
                  cv::Point(50, 50),
                  cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 150, 255), 1);
      RoboCatchCmdUartBuff uart_temp_catch_cmd;
      // make the state of the cube 
      if (detected_objects.at(0).id == 0 ||
          detected_objects.at(0).id == 3)
      {
        uart_temp_catch_cmd.cube_state = CUBE_UP;
      }
      else if (detected_objects.at(0).id == 1 ||
               detected_objects.at(0).id == 4)
      {
        uart_temp_catch_cmd.cube_state = CUBE_DOWN;
      }
      else if (detected_objects.at(0).id == 2 ||
               detected_objects.at(0).id == 5)
      {
        uart_temp_catch_cmd.cube_state = CUBE_STAND;
      }
      // standing mode judge the age
      if(uart_temp_catch_cmd.cube_state == CUBE_STAND) {
      // get the type of the cube by judging that area
        if (temp_rect_.area() > cube_1_min_area &&
            temp_rect_.area() < cube_1_max_area)
        {
          uart_temp_catch_cmd.cube_type = CUBE_1;
        }

        if (temp_rect_.area() > 21000 &&
            temp_rect_.area() < 33000)
        {
          uart_temp_catch_cmd.cube_type = CUBE_2;
        }
        else if (temp_rect_.area() > cube_3_min_area &&
                temp_rect_.area() < cube_3_max_area)
        {
          uart_temp_catch_cmd.cube_type = CUBE_3;
        }
        else if (temp_rect_.area() > cube_4_min_area &&
                temp_rect_.area() < cube_4_max_area)
        {
          uart_temp_catch_cmd.cube_type = CUBE_4;
        }
        if (uart_temp_catch_cmd.cube_state == CUBE_STAND)
        {
          uart_temp_catch_cmd.cube_type = CUBE_UNCERTAIN;
        }
      }
      // get the type of the cube by judging that area
      if (temp_rect_.area() > cube_1_min_area &&
          temp_rect_.area() < cube_1_max_area)
      {
        uart_temp_catch_cmd.cube_type = CUBE_1;
      }

      if (temp_rect_.area() > cube_2_min_area &&
          temp_rect_.area() < cube_2_max_area)
      {
        uart_temp_catch_cmd.cube_type = CUBE_2;
      }
      else if (temp_rect_.area() > cube_3_min_area &&
               temp_rect_.area() < cube_3_max_area)
      {
        uart_temp_catch_cmd.cube_type = CUBE_3;
      }
      else if (temp_rect_.area() > cube_4_min_area &&
               temp_rect_.area() < cube_4_max_area)
      {
        uart_temp_catch_cmd.cube_type = CUBE_4;
      }
      if (uart_temp_catch_cmd.cube_state == CUBE_STAND)
      {
        uart_temp_catch_cmd.cube_type = CUBE_UNCERTAIN;
      }
      // send the true message to the serial for three times
      if (uart_temp_catch_cmd.cube_type != 0x00)
      {
        for (int i = 0; i < 3; i++)
        {
          serial_robot_->write((uint8_t *)&uart_temp_catch_cmd, sizeof(uart_temp_catch_cmd));
          std::cout << "catch, temp_rect_ size:" << temp_rect_.area() << "  temp_rect_ type:"
                    << (int)uart_temp_catch_cmd.cube_type << "  temp_rect_ state:"
                    << (int)uart_temp_catch_cmd.cube_state << "\n";
          usleep(cube_target_echo_uart_cmd_sleep_time);
        }
      }
      robo_inf.catch_cube_mode_status.store(CatchMode::detect_mode);
      break;
    }
/*******************detect_**************************/
    case CatchMode::detect_mode:
    {
    cv::putText(src_img,
                "detect cube mode",
                cv::Point(50, 50),
                cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 150, 255), 1);
      RoboCubeStateUartBuff uart_temp_state_cmd;
      if (detected_objects.at(0).id == 0 ||
          detected_objects.at(0).id == 3)
      {
        uart_temp_state_cmd.cube_status = CUBE_UP;
      }
      else if (detected_objects.at(0).id == 1 ||
               detected_objects.at(0).id == 4)
      {
        uart_temp_state_cmd.cube_status = CUBE_DOWN;
      }
      else if (detected_objects.at(0).id == 2 ||
               detected_objects.at(0).id == 5)
      {
        uart_temp_state_cmd.cube_status = CUBE_STAND;
      }
      uart_temp_state_cmd.cube_status = CUBE_DOWN;
      uart_temp_state_cmd.cube_type = 0x03;
      if (uart_temp_state_cmd.cube_status != 0x00)
      {
      for (int i = 0; i < 3; i++)
        {
          serial_robot_->write((uint8_t *)&uart_temp_state_cmd, sizeof(uart_temp_state_cmd));
          std::cout << (int)uart_temp_state_cmd.cube_type << "  temp_rect_ state:"
                    << (int)uart_temp_state_cmd.cube_status << "\n";
          usleep(cube_target_echo_uart_cmd_sleep_time);
        }
      }
      robo_inf.catch_cube_mode_status.store(CatchMode::off);

      break;
    }
    default:
      break;
  }

  cv::rectangle(src_img,
            temp_rect_,
            cv::Scalar(0, 0, 0),
            2,
            LINE_8,
          0); 
  cv::putText(src_img,detected_objects.at(0).name,
              Point(temp_rect_.x + 20,temp_rect_.y - 10),cv::FONT_HERSHEY_COMPLEX,
                  0.7,
                  cv::Scalar(100,100,255),
                  0.5,
                cv::LINE_4);

  cv::rectangle(src_img, temp_rect_, cv::Scalar(0, 150, 255), 2);
  cv::putText(src_img, 
              std::to_string(detected_objects.at(0).id),
              cv::Point(temp_rect_.x, temp_rect_.y - 1),
              cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 150, 255), 1);

#ifndef RELEASE
      if (!src_img.empty()) {
        std::vector<uchar> buff_bgr;
        cv::imencode(".jpg", src_img, buff_bgr,params);
        streamer_->publish("/tpcm",
                              std::string(buff_bgr.begin(), buff_bgr.end()));
      }
#endif

// fps caculate
  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency() ;
  auto fps = 1.0 / t;
  fmt::print("FPS:{}\n",fps); 
  cv::waitKey(1);
 }
 catch(const std::exception& e)
 {
  std::cerr << e.what() << '\n';
 }
 
 
}


void RobotR2::uartRead()
{
 while (true)try
 {
  if(serial_robot_->isOpen()) {
      serial_robot_->ReceiveInfo(robo_inf);
  } else {
      serial_robot_->open();
  }
  std::this_thread::sleep_for(std::chrono::seconds(5));
 }
 catch(const std::exception& e)
 {
  serial_robot_->close();
  static int serial_read_excepted_times{0};
  if (serial_read_excepted_times++ > 3) {
  std::this_thread::sleep_for(std::chrono::seconds(5));
    fmt::print(">>>> read serial excepted to many times, sleep 10s.<<<<\n");
    serial_read_excepted_times = 0;
  }
  fmt::print(">>>> serial exception: {}<<<<\n",e.what());
  std::this_thread::sleep_for(std::chrono::seconds(5));
 }
}


void RobotR2::spin() {
 std::thread detectThread(std::bind(&RobotR2::cubeDetect,this));
 std::thread uart_thread(std::bind(&RobotR2::uartRead,this));    
 if(!detectThread.joinable()) {
     detectThread.detach();
 }
 if(!uart_thread.joinable()) {
     uart_thread.detach();
 }
    
}
void RobotR2::stop()
{

}
RobotR2::~RobotR2(){

}