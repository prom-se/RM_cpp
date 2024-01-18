#include "Rune.hpp"

using namespace ov;

void Rune::prepare(cv::Mat &src) {
    src(cv::Rect(src.cols/2-640,src.rows/2-512,1280,1024)).copyTo(image(cv::Rect(0,0,1280,1024)));
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

    //缩放图片并归一化
    cv:: Mat blob_image;
    cv::resize(image,blob_image,cv::Size(640, 640));
    blob_image.convertTo(blob_image, CV_32F);
    blob_image = blob_image / 255.0;

    // 将图片数据填充到模型输入节点中
    // 原有图片数据为 HWC格式，模型输入节点要求的为 CHW 格式
    auto *input_tensor_data = input.data<float>();
    for (size_t c = 0; c < 3; c++) {
        for (size_t h = 0; h < 640; h++) {
            for (size_t w = 0; w < 640; w++) {
                input_tensor_data[c * 640 * 640 + h * 640 + w] = blob_image.at<cv::Vec<float, 3>>(h, w)[c];
            }
        }
    }
}

void Rune::infer_v5() {
    infer_request.infer();

    const auto *output_buffer = output.data<const float>();
    cv::Mat det_output(25200, 10, CV_32F, (float*)output_buffer);

    for(int i = 0; i < det_output.rows; i++) {
        float confidence = det_output.at<float>(i, 4);
        if (confidence < 0.5) {
            continue;
        }
        cv::Mat classes_scores = det_output.row(i).colRange(5, 10);
        cv::Point classIdPoint;
        double score;
        minMaxLoc(classes_scores, nullptr, &score, nullptr, &classIdPoint);


        // 置信度 0～1之间
        if (score > 0.6){
            float cx = det_output.at<float>(i, 0);
            float cy = det_output.at<float>(i, 1);
            float ow = det_output.at<float>(i, 2);
            float oh = det_output.at<float>(i, 3);
            int x = static_cast<int>((cx - 0.5 * ow) * 2);
            int y = static_cast<int>((cy - 0.5 * oh) * 2);
            int width = static_cast<int>(ow * 2);
            int height = static_cast<int>(oh * 2);
            cv::Rect box;
            box.x = x;
            box.y = y;
            box.width = width;
            box.height = height;

            boxes.emplace_back(box);
            classIds.emplace_back(classIdPoint.x);
            confidences.emplace_back((float)score);
        }
    }
    // NMS
    cv::dnn::NMSBoxes(boxes, confidences, 0.5, 0.5, indexes);
}

void Rune::infer_v8() {
    infer_request.infer();

    const auto *output_buffer = output.data<const float>();
    cv::Mat det_output(9, 8400, CV_32F, (float *)output_buffer);

    for(int i = 0; i < det_output.cols; i++) {
        cv::Mat classes_scores = det_output.col(i).rowRange(4, 9);
        cv::Point classIdPoint;
        double score;
        minMaxLoc(classes_scores, nullptr, &score, nullptr, &classIdPoint);
        // 置信度 0～1之间
        if (score > 0.6){
            float cx = det_output.at<float>(0, i);
            float cy = det_output.at<float>(1, i);
            float ow = det_output.at<float>(2, i);
            float oh = det_output.at<float>(3, i);
            int x = static_cast<int>((cx - 0.5 * ow) * 2);
            int y = static_cast<int>((cy - 0.5 * oh) * 2);
            int width = static_cast<int>(ow * 2);
            int height = static_cast<int>(oh * 2);
            cv::Rect box;
            box.x = x;
            box.y = y;
            box.width = width;
            box.height = height;

            boxes.emplace_back(box);
            classIds.emplace_back(classIdPoint.y);
            confidences.emplace_back((float)score);
        }
    }
    // NMS
    cv::dnn::NMSBoxes(boxes, confidences, 0.6, 0.6, indexes);
}
void Rune::findTarget() {
    int R_index,Target_index;
    if(targets.color=="red"){
        R_index = 3;
        Target_index = 4;
    }
    else if(targets.color=="blue"){
        //R_index = 3;//debug 3
        R_index = 0;
        Target_index = 1;
    }
    int i = 0;
    targets.index_R = -1; targets.index_Target = -1;
    for (int index : indexes) {
        if(boxes[index].y<300)continue;
        if(boxes[index].x<300)continue;
        // if(classIds[index]== R_index){
        //     targets.index_R = i;
        // }
        if(classIds[index]== Target_index){
            if(boxes[index].area()<2500)continue;
            targets.index_Target = i;
        }
        else if(classIds[index]== 2){
            targets.index_Activatied.emplace_back(i);
        }
        else{
            targets.index_ignore.emplace_back(i);
        }
        targets.center.emplace_back(boxes[index].x+boxes[index].width/2, boxes[index].y+boxes[index].height/2);
        std::vector<cv::Point2f> position = {
                cv::Point2f((float)boxes[index].x,(float)boxes[index].y),
                cv::Point2f((float)boxes[index].x,(float)boxes[index].y+(float)boxes[index].height),
                cv::Point2f((float)boxes[index].x+(float)boxes[index].width,(float)boxes[index].y),
                cv::Point2f((float)boxes[index].x+(float)boxes[index].width,(float)boxes[index].y+(float)boxes[index].height)};
        targets.pix_position.emplace_back(position);
        i++;
    }
}

void Rune::findR() {
    cv::Mat gray,bin;
    cv::cvtColor(image,gray,cv::COLOR_RGB2GRAY);
    cv::threshold(gray,bin,180,255,cv::THRESH_BINARY);
    cv::Mat ele = cv::getStructuringElement(cv::MORPH_ERODE,cv::Size(3,3));
    cv::erode(bin,bin,ele);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(bin,contours,hierarchy,cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for(const auto & contour : contours){
        cv::RotatedRect rect = cv::minAreaRect(contour);
        double k = rect.size.width/rect.size.height;
        if(rect.center.y<100) continue;
        if(rect.center.x<300) continue;
        if(k>1.5||k<0.66) continue;
        cv::Rect bRect=cv::boundingRect(contour);
        if(bRect.area()>2500||bRect.area()<100)continue;
        cv::Scalar bgr = cv::mean(image(bRect));
        std::string lightColor = bgr.val[2]-bgr.val[0]>=0 ? "blue":"red";
        if(lightColor=="red")continue;
        
        std::vector<cv::Point2f> pts = {
            cv::Point2f(bRect.x,bRect.y),
            cv::Point2f(bRect.x,bRect.y+bRect.height),
            cv::Point2f(bRect.x+bRect.width,bRect.y),
            cv::Point2f(bRect.x+bRect.width,bRect.y+bRect.height)};
        targets.pix_position.emplace_back(pts);
        targets.center.emplace_back(bRect.x+bRect.width/2,bRect.y+bRect.height/2);
        targets.index_R=targets.pix_position.size()-1;
    }
}

Rune::Rune() {
//    core.set_property("CPU", hint::inference_precision("f16"));
}

