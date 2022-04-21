#include "serial/serial.h"
#include "utils.hpp"

class RoboSerial : public serial::Serial {
 public:
  RoboSerial(std::string port, unsigned long baud) {
    auto timeout = serial::Timeout::simpleTimeout(serial::Timeout::max());
    this->setPort(port);
    this->setBaudrate(baud);
    this->setTimeout(timeout);
    try { 
      this->open();
      fmt::print(fg(fmt::color::green) | fmt::emphasis::bold,"Serial init successed.\n") ;
    } catch(const std::exception& e) {
      fmt::print(fg(fmt::color::red) | fmt::emphasis::bold,"Serial init failed, {}.\n", e.what());
    }
  }

  void ReceiveInfo(RoboInf &robo_inf) {
    RoboInfUartBuff uart_buff_struct;
    uint8_t uart_S_flag;
    this->read(&uart_S_flag, 1);
    while (uart_S_flag != 'S')
      this->read(&uart_S_flag, 1);
    this->read((uint8_t *)&uart_buff_struct, sizeof(uart_buff_struct));
    if (uart_buff_struct.mode == NOTHING) {
      if (robo_inf.catch_cube_mode_status.load() != CatchMode::off)
        robo_inf.catch_cube_mode_status.store(CatchMode::off);
    } else if (uart_buff_struct.mode == AUTO_MODE) {
      if (robo_inf.catch_cube_mode_status.load() == CatchMode::off)
        robo_inf.catch_cube_mode_status.store(CatchMode::spin);
    } else if (uart_buff_struct.mode == MANUAL_MODE) {
      if (robo_inf.catch_cube_mode_status.load() == CatchMode::off)
        robo_inf.catch_cube_mode_status.store(CatchMode::catch_cube);
    }else if (uart_buff_struct.mode == DETECT_MODE) {
        robo_inf.catch_cube_mode_status.store(CatchMode::catch_cube);
    }
  }

  // 串口发送
  void serialSendInf(RoboCmd &robo_inf_send_temp)
  {
    RoboInfNewUartBUff robo_send_inf;
    robo_send_inf.cube_color = robo_inf_send_temp.cube_color.load();
    robo_send_inf.cube_state = robo_inf_send_temp.cube_state.load();
    robo_send_inf.distance = robo_inf_send_temp.distance.load();
    robo_send_inf.yaw_angle = robo_inf_send_temp.yaw_angle.load();
    this->write((uint8_t*)&robo_send_inf,sizeof(robo_send_inf));
  }
};