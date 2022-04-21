#include "wolfvision.h"

#define AUTO_DETECT 0
WolfRobot_R2 R2;
void serialR2_send(){ R2.serialSend();}
void serialR2_read() { R2.serialReceive();}
void launcherR2_spin(){
  fmt::print(fmt::format(fg(fmt::color::green) | fmt::emphasis::bold,"---LODE THE LAUNCHER MODE----\n"));
  pipeline pipe; // 创建数据管道
  pipeline_profile profile;// start() 函数返回数据管道的 profile
  R2.detectionObject(pipe,profile);
}
int main() {
  fmt::print(fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "ROBOT_R2\n"));
  std::thread serial_read(serialR2_read);
  std::thread detect_ojbect(launcherR2_spin);
  // std::thread serial_send(serialR2_send);
  if(serial_read.joinable()){
    serial_read.detach();
  }
  if(detect_ojbect.joinable()) {
    detect_ojbect.detach();
  }
  if(std::getchar() == 0) {
    fmt::print("the key hitting\n");
  }
  return 0;
}