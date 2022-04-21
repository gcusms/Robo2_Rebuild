#ifndef DETECTOR_H
#define DETECTOR_H
#include <opencv2/opencv.hpp>
#include <inference_engine.hpp>
#include <iostream>
#include <chrono>
#include <opencv2/dnn/dnn.hpp>
#include <cmath>


using namespace std;
using namespace cv;
using namespace InferenceEngine;

class Detector
{
public:
    struct Object{
        float prob;
        std::string name;
        cv::Rect rect;
        int id;
        int dis_cols;
        int dis_rows;
        int dis_;
        bool operator < (const Object &y) const
        {
            return dis_rows < y.dis_rows; 
        }
    };
    Detector(const std::string& path_xml,const std::string& path_bin,float& conf_detect,float &conf_frame);
    ~Detector();
    //初始化
    bool init();
    //释放资源
    bool uninit();
    //处理图像获取结果
    bool process_frame(Mat& inframe,vector<Object> &detected_objects);

private:
    double sigmoid(double x);
    vector<int> get_anchors(int net_grid);
    bool parse_yolov5(const Blob::Ptr &blob,int net_grid,float cof_threshold,
        vector<Rect>& o_rect,vector<float>& o_rect_cof,vector<int>& input_label);
    Rect detet2origin(const Rect& dete_rect,float rate_to,int top,int left);
    //存储初始化获得的可执行网络
    ExecutableNetwork _network;
    OutputsDataMap _outputinfo;
    string _input_name;
    //参数区
    string _xml_path;                             //OpenVINO模型xml文件路径
    string _bin_path;                             //OpenVINO模型bin文件路径
    double _cof_threshold;                //置信度阈值,计算方法是框置信度乘以物品种类置信度
    double _nms_area_threshold;  //nms最小重叠面积阈值
};
#endif