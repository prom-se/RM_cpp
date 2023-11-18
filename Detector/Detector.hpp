
#ifndef RM_CPP_DETECTOR_HPP
#define RM_CPP_DETECTOR_HPP
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <chrono>
#include "../Rune/Rune.hpp"

//灯条
struct LightBar{
    int nums = 0;
    std::vector<std::vector<cv::Point2f>> position;
    std::vector<cv::Size2f> size;
    std::vector<double> theta;
    std::vector<bool> matched;
};

//装甲板
struct Armor{
    int nums = 0;
    int best_index = 0;
    std::vector<std::vector<cv::Point2f>> pix_position;
    struct LightBar leftBar, rightBar;
    std::vector<std::string> number;
    std::vector<cv::Size2f> size;
    std::vector<double> theta;
    std::vector<char> type;
    std::vector<cv::Point2f> center;
    std::vector<double> pix_distance;
};



class Detector {
private:
    cv::VideoWriter writer;
public:
    long now_time = 0;
    double delta_time = 0;
    double fps = 0;
    Detector()=default;
    Rune rune;
    cv::Mat src = cv::Mat(1024, 1280, CV_8UC3);
    cv::Mat show = cv::Mat(1024, 1280, CV_8UC3);
    cv::Mat roi = cv::Mat(640, 800, CV_8UC3);
    cv::Mat dst = cv::Mat(1024, 1280, CV_8UC1);
    cv::Mat gray = cv::Mat(1024, 1280, CV_8UC1);

    cv::Mat roi_bin = cv::Mat(640, 800, CV_8UC1);
    std::string target_color = "blue"; // "red" or "blue"
    bool isRune = false; //是否检测大符
    int ThresholdValue = 170; //二值化阈值
    struct LightBar susBar;
    struct Armor Armor;
    cv::Mat Target_tvec;
    cv::Mat Target_rvec;
    double Target_dis{};
    double yaw{}, pitch{}, offset_pitch{}, offset_y{};
    std::string serMsg,readMsg;
    std::string num_classes[9] = {"1", "2", "3", "4", "5", "Outpost", "Guard", "Base", "Negative"};
    int frames = 0;

    //绘图
    void draw();
    cv::Mat drawed = cv::Mat(1024, 1280, CV_8UC3);

    //显示图像
    void debug(long &start_time, cv::Mat &show_mat, bool show_flag);
    //log
    void log();

    //检测装甲板
    bool detect();

    //寻找疑似灯条
    bool findLightBar();

    //配对灯条
    bool matchLightBar();

    //数字识别
    bool number_classify();
    cv::dnn::Net net = cv::dnn::readNetFromONNX("../model/mlp.onnx");
    ~Detector() = default;

    bool containTest(int i, int j);
};


#endif //RM_CPP_DETECTOR_HPP
