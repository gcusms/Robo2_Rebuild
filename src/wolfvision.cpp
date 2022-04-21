#include "wolfvision.h"

std::string idntifier_green_serial = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "serial");
std::string idntifier_red_serial   = fmt::format(fg(fmt::color::red)   | fmt::emphasis::bold, "serial");
std::string function_begin   = fmt::format(fg(fmt::color::green)   | fmt::emphasis::bold, "begin");


WolfRobot_R2::WolfRobot_R2()
{
    // this->serial_ = std::make_unique<RoboSerial>("/dev/ttyS9", 115200);  // to change
}
void WolfRobot_R2::serialReceive()
{
    while (!this->node_judge_) try {
      if(this->serial_->isOpen()) {
        this->serial_->ReceiveInfo(this->robo_inf);
      }
      std::this_thread::sleep_for(std::chrono::microseconds(1));
    } catch (const std::exception &e) {
      std::this_thread::sleep_for(std::chrono::seconds(1)); 
    }
}
void WolfRobot_R2::serialSend()
{
    while (!this->node_judge_) try {
      if(this->serial_->isOpen()) {
        this->serial_->serialSendInf(this->robo_cmd);
      } else {
        this->serial_->open();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    } catch (const std::exception &e) {
      this->serial_->close();
      static int serial_read_excepted_times{0};
      if (++serial_read_excepted_times > 3) {
        fmt::print("[{}] serial excepted to many times, sleep 10s.\n",
                   idntifier_red_serial);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        serial_read_excepted_times = 0;
      }
      fmt::print("[{}] {}\n",
                 idntifier_red_serial, e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}



void WolfRobot_R2::detectionObject(pipeline &pipe, pipeline_profile &profile) {
   // 检测对象声明
  string xml_path = "../model/best.xml";
  string bin_path = "../model/best.bin";
  float conf_detect_ = 0.7f;
  float conf_frame = 0.45;
  this->detector = new Detector(xml_path,bin_path,conf_detect_, conf_frame);
  profile = pipe.start();
  while (!this->detect_judge_)
  {   
    // 堵塞程序直到新的一帧捕获
    frameset frameset = pipe.wait_for_frames();
    // 获取颜色图
    video_frame video_src = frameset.get_color_frame();
    const int color_width = video_src.as<video_frame>().get_width();
    const int color_height = video_src.as<video_frame>().get_height();
    this->src_img = cv::Mat(Size(color_width, color_height), CV_8UC3,
                (void *)video_src.get_data(), Mat::AUTO_STEP);
    if(!this->detector->process_frame(this->src_img, this->detected_objects))
    {
      cvtColor(this->src_img,this->src_img,COLOR_BGR2RGB);
      if (!this->src_img.empty()) {
        cv::imshow("interface", this->src_img);
        cv::waitKey(1);
      }
      continue;
    }
    cvtColor(this->src_img,this->src_img,cv::COLOR_BGR2RGB);
    // 排序分类
    std::sort(detected_objects.begin(), detected_objects.end());
    drawRectangle_black(this->src_img,detected_objects.at(0).rect);
    drawLabelRect(this->src_img,detected_objects.at(0).name,detected_objects.at(0).rect);
    // 定义临时举行存储输出结果 (Rect)
    cv::Rect temp_rect = detected_objects.at(0).rect;
    switch (robo_inf.catch_cube_mode_status.load())
    {
    // 角度调整
    case CatchMode::spin:
    {
      drawEntitle(this->src_img,"spin");
      float error_angle_temp = std::abs(temp_rect.x + temp_rect.width*0.5);
      if( error_angle_temp < this->detect_parameters.error_angle) {
        this->detect_parameters.cube_middle_detect_times++;
        if(this->detect_parameters.cube_middle_detect_times > 3) {
          RoboSpinCmdUartBuff uart_temp_spin_cmd;
          uart_temp_spin_cmd.yaw_angle = 0.f;
          for (int i = 0; i < 3; i++)
          {
            this->serial_->write((uint8_t *)&uart_temp_spin_cmd, sizeof(uart_temp_spin_cmd));
            std::cout << "send yaw 0\n";
            usleep(20000);
          }
          this->detect_parameters.cube_middle_detect_times = 0;
          this->robo_inf.catch_cube_mode_status.store(CatchMode::go);
        } 
      } else {
        RoboSpinCmdUartBuff uart_temp_spin_cmd;
        uart_temp_spin_cmd.yaw_angle = error_angle_temp;
        std::cout << "uart_temp_spin_cmd.yaw_angle" << uart_temp_spin_cmd.yaw_angle << "\n";
        this->serial_->write((uint8_t *)&uart_temp_spin_cmd, sizeof(uart_temp_spin_cmd));
      }
      break;
    }
    // 执行直行
    case CatchMode::go:
    {
      drawEntitle(this->src_img,"go");
      float detect_distance = std::abs((temp_rect.y + temp_rect.height*0.5) - this->src_img.rows);
      if(detect_distance < this->detect_parameters.error_distance) {
        this->detect_parameters.cude_front_detect_times ++;
        if(detect_parameters.cude_front_detect_times > 3) {
          for (int i = 0; i < 3; i++)
          {
            RoboGoCmdUartBuff uart_temp_go_cmd;
            uart_temp_go_cmd.distance = 0.f;
            this->serial_->write((uint8_t *)&uart_temp_go_cmd, sizeof(uart_temp_go_cmd));
            std::cout << "uart_temp_go_cmd.distance:" << uart_temp_go_cmd.distance << "\n";
          }
          detect_parameters.cude_front_detect_times = 0;
          this->robo_inf.catch_cube_mode_status.store(CatchMode::catch_cube);
          usleep(20000);
        }
      } else {
        RoboGoCmdUartBuff uart_temp_go_cmd;
        uart_temp_go_cmd.distance = detect_distance;
        std::cout << "uart_temp_go_cmd.distance:" << uart_temp_go_cmd.distance << "\n";
        this->serial_->write((uint8_t *)&uart_temp_go_cmd, sizeof(uart_temp_go_cmd));
      }
      break;
    }
    // 抓取
    case CatchMode::catch_cube:
    {
      drawEntitle(this->src_img,"catch");
      Cathbuff_reset catch_uart;
      catch_uart.cube_id = this->detected_objects.at(0).id;
      for (int i = 0; i < 3; i++)
      {
        this->serial_->write((uint8_t*)&catch_uart,sizeof(catch_uart));
      }
      usleep(20000);
    }
    default:
      fmt::print("{}\n",this->robo_inf.catch_cube_mode_status.load());
      break;
    }
  }
}

// 析构内存
WolfRobot_R2::~WolfRobot_R2()
{
  delete this->detector;
  detector = nullptr;
}


