#ifndef RM_CPP_RUNE_HPP
#define RM_CPP_RUNE_HPP

#include <opencv2/opencv.hpp>
#include <openvino/runtime/core.hpp>
#include <iostream>

using namespace ov;


struct rune_targets{
    int index_R;
    int index_Target;
    std::vector<int> index_Activatied;
    std::vector<int> index_ignore;
    std::vector<cv::Point2f> center;
    std::vector<std::vector<cv::Point2f>> pix_position;
    std::string color;

};

class Rune {
public:
    std::vector<std::string> class_names = {"Blue_r", "Blue_target", "Activated", "Red_r", "Red_target"};
    Core core;
//    CompiledModel model = core.compile_model("../model/small_v8.xml","CPU");
    CompiledModel model = core.compile_model("../model/nano_v8.xml","CPU");
//    CompiledModel model = core.compile_model("../model/best.xml","CPU");
    InferRequest infer_request = model.create_infer_request();
    Tensor input = infer_request.get_input_tensor();
    Tensor output = infer_request.get_output_tensor();
    Shape tensor_shape = input.get_shape();
    cv::Mat image = cv::Mat::zeros(cv::Size(1280, 1280), CV_8UC3);

    //预处理
    void prepare(cv::Mat &src);

    //推理
    void infer_v5();
    void infer_v8();

    std::vector<int> indexes;
    std::vector<cv::Rect> boxes;
    std::vector<int> classIds;
    std::vector<float> confidences;

    //寻找目标
    void findTarget();
    void findR();
    struct rune_targets targets;

    Rune();
    ~Rune() = default;

};


#endif //RM_CPP_RUNE_HPP
