
#include "streamer_test.h"


int main()
{
 streamer.start(8080);
 pipeline pipe;                           // 创建数据管道
    // start() 函数返回数据管道的 profile
 pipeline_profile profile = pipe.start();
 std::cout << "CAMERA" << std::endl;
 while (streamer.isRunning())
 try
 {
          // 堵塞程序直到新的一帧捕获
  frameset frameset = pipe.wait_for_frames();  
  rs2::video_frame video_src = frameset.get_color_frame(); 

    // 获取颜色图的尺寸，用于转成 Mat 格式并显示
  const int color_width = video_src.as<video_frame>().get_width();
  const int color_height = video_src.as<video_frame>().get_height();

          // 转成 Mat 类型
  cv::Mat frame(cv::Size(color_width, color_height), CV_8UC3,             
                (void*)video_src.get_data(),cv::Mat::AUTO_STEP);
  // http://localhost:8080/bgr
  std::vector<uchar> buff_bgr;
  cv::imencode(".jpg", frame, buff_bgr, params);
  streamer.publish("/bgr", std::string(buff_bgr.begin(), buff_bgr.end()));
  cv::imshow("frame",frame);
  if(cv::waitKey(1) == 'q')
  {
   break;
  };
 }
 catch(const std::exception& e)
 {
  std::cerr << e.what() << '\n';
 }
 

 streamer.stop();
}