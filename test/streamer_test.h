#include <iostream>
#include "mjpeg_streamer.hpp"
#include "opencv4/opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <librealsense2/rs.hpp>

using namespace rs2;
using MJPEGStreamer = nadjieb::MJPEGStreamer;
cv::VideoCapture capture(0);
cv::Mat src;
std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};
MJPEGStreamer streamer;
void ViderStreamer();  // Video Up