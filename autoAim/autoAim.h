//
// Created by jsz on 12/19/19.
//

#ifndef ROBOTBASE_AUTOAIM_H
#define ROBOTBASE_AUTOAIM_H

#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include "../control.h"
#include "armor.h"
struct OtherParam {
    uint8_t color = 0;
    uint8_t mode = 0;
};



class ArmorDetector {
public:
    ArmorDetector() {
        t_start_ = cv::getTickCount();
    }

    ~ArmorDetector() {}

    int armorTask(cv::Mat &img, OtherParam other_param);

    bool DetectArmor(cv::Mat& img, cv::Point3f &target_3d, cv::Mat &depth_img, cv::Rect roi);
public:
    int color_th_ = 16;
    int gray_th_ = 60;
private:
    bool makeRectSafe(cv::Rect &rect, cv::Size size) {
        if (rect.x < 0)
            rect.x = 0;
        if (rect.x + rect.width > size.width)
            rect.width = size.width - rect.x;
        if (rect.y < 0)
            rect.y = 0;
        if (rect.y + rect.height > size.height)
            rect.height = size.height - rect.y;
        if (rect.width <= 0 || rect.height <= 0)
            return false;
        return true;
    }

    cv::Rect GetRoi(const cv::Mat &img);

private:

    double t_start_;

    cv::Rect last_target_;
    int lost_count = 0;
    int detect_count = 0;

    uint8_t color_;
    uint8_t mode_;
};

#endif //ROBOTBASE_AUTOAIM_H
