#include <atomic>
#include "opencv2/imgproc/imgproc.hpp"

#include "fmt/color.h"
#include "fmt/core.h"

#define AUTO_MODE 0x01
#define MANUAL_MODE 0x02
#define DETECT_MODE 0x03
#define JUDGE_MODE 0x04
#define NOTHING 0x05

#define CUBE_1 0x01 // min
#define CUBE_2 0x02
#define CUBE_3 0x03
#define CUBE_4 0x04
#define CUBE_5 0x05
#define CUBE_UNCERTAIN 0X06
#define CUBE_UP    0x01
#define CUBE_DOWN  0x02
#define CUBE_STAND 0x03

#define SPIN_SIGN         0x01
#define GO_SIGN           0X02
#define CATCH_SIGN        0X03
#define RETURN_CUBE_STATE 0X04
#define SUSTAIN_SENDING  0X05


enum CatchMode {
  off = 0,
  spin,
  go,
  catch_cube,
  detect_mode
};

struct RoboInf {
  std::atomic<uint8_t> mode {0x00};
  std::atomic<CatchMode> catch_cube_mode_status {CatchMode::off};
  std::atomic<uint8_t> color_self {0x00};  // 己方机器人颜色
};


// send R2 spin command
struct RoboSpinCmdUartBuff {
  uint8_t S_flag = 'S';
  uint8_t cmd_type = SPIN_SIGN;
  float yaw_angle = 0.f;
  uint8_t E_flag = 'E';
} __attribute__((packed));

// send R2 spin command
struct RoboGoCmdUartBuff {
  uint8_t S_flag = 'S';
  uint8_t cmd_type = GO_SIGN;
  float distance = 0.f;
  uint8_t E_flag = 'E';
} __attribute__((packed));

// send R2 catch command
// cube_state: 0x01 - yellow, 0x02 - white, 0x03 - stand
// cube_type: 0x01 - 0x05
struct RoboCatchCmdUartBuff {
  uint8_t S_flag = 'S';
  uint8_t cmd_type = CATCH_SIGN;
  uint8_t cube_state = 0x00;
  uint8_t cube_type = 0x00;
  uint8_t E_flag = 'E';
} __attribute__((packed));

struct Cathbuff_reset {
  uint8_t S_flag = 'S';
  uint8_t cmd_type = CATCH_SIGN;
  uint8_t cube_id = 0x00;
  uint8_t E_flag = 'E';
} __attribute__((packed));

struct RoboTypeCmdUartBuff {
  uint8_t S_flag = 'S';
  uint8_t cmd_type = RETURN_CUBE_STATE;
  int cube_type = 0;
  uint8_t E_flag = 'E';
} __attribute__((packed));


// send R2 cube status
// 0x01 white 0x02 yellow 
// cube_type: 0x01 - 0x05
struct RoboCubeStateUartBuff {
  uint8_t S_flag = 'S';
  uint8_t cmd_type = RETURN_CUBE_STATE;
  uint8_t cube_status = 0x00;
  uint8_t cube_type = 0x00;
  uint8_t E_flag = 'E';
} __attribute__((packed));

//uart recive
struct RoboInfUartBuff {
  uint8_t mode = NOTHING;
} __attribute__((packed));


/*********NEW_SEND*********/

struct RoboCmd {
  std::atomic<float> distance;
  std::atomic<float> yaw_angle;
  std::atomic<uint8_t> cube_color;
  std::atomic<uint8_t> cube_state;
};
/**
 * COLOR RED 0X01 BLUE 0X02
 * STATE up 0x01 erect 0x02 down 0x03
 * distance
 * yaw_angle
*/
struct RoboInfNewUartBUff {
  uint8_t S_flag = (unsigned)'S';
  uint8_t cmd_type = SUSTAIN_SENDING;
  uint8_t cube_color = 0x00;
  uint8_t cube_state = 0x00;
  float distance = 0.f;
  float yaw_angle = 0.f;
  uint8_t E_flag = (unsigned)'E';
} __attribute__((packed));



namespace drawspace {

  inline void drawRectangle_black(cv::Mat &input_img, cv::Rect &rect) {
    cv::rectangle(
              input_img,
              rect,
              cv::Scalar(0, 0, 0),
              2,
              cv::LINE_8,
              0); 
  }

  inline void drawRectangle_pink(cv::Mat &input_img_, cv::Rect &rect_) {
    cv::rectangle(
              input_img_,
              rect_,
              cv::Scalar(255, 0, 255),
              2,
              cv::LINE_8,
              0); 
  }

  inline void drawLabelRect(cv::Mat &input_img_temp,std::string &msg_,cv::Rect &__rect) {
    cv::putText(
      input_img_temp,
      msg_,
      cv::Point(__rect.x + 20,__rect.y - 10),
      cv::FONT_HERSHEY_COMPLEX,
      0.7,
      cv::Scalar(100,100,255),
      0.5,
      cv::LINE_4
      );
  }

  inline void drawEntitle(cv::Mat &input_img,std::string msg) {
    cv::putText(
      input_img,
      msg,
      cv::Point(100,100),
      cv::FONT_HERSHEY_COMPLEX,
      0.7,
      cv::Scalar(100,100,255),
      0.5,
      cv::LINE_4
      );
  }

}
