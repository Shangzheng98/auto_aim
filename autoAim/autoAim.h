//
// Created by jsz on 12/19/19.
//

#ifndef ROBOTBASE_AUTOAIM_H
#define ROBOTBASE_AUTOAIM_H

#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <librealsense2/rs.hpp>
#include "../control.h"
#include "armor.h"

struct _OtherParam {
    uint8_t color = 0; //我方车辆颜色，0是蓝色，1是红色。用于图像预处理
    uint8_t mode = 0;
};
typedef _OtherParam OtherParam;


class ArmorDetector {
public:
    ArmorDetector() {
        t_start_ = cv::getTickCount();
    }

    ~ArmorDetector() = default;

    int armorTask(cv::Mat &img, rs2::frameset frames, OtherParam other_param);

    bool DetectArmor(cv::Mat &img, cv::Point3f &target_3d, rs2::frameset frames, cv::Rect roi);

public:
    int color_th_ = 16;
    int gray_th_ = 60;
private:
    bool makeRectSafe(cv::Rect &rect, const cv::Size &size) {
        if (rect.x < 0)
            rect.x = 0;
        if (rect.x + rect.width > size.width)
            rect.width = size.width - rect.x;
        if (rect.y < 0)
            rect.y = 0;
        if (rect.y + rect.height > size.height)
            rect.height = size.height - rect.y;
        return !(rect.width <= 0 || rect.height <= 0);
    }

    cv::Rect GetRoi(const cv::Mat &img);

private:

    double t_start_;

    cv::Rect last_target_;
    int lost_count = 0;
    int detect_count = 0;

    uint8_t color_{};
    uint8_t mode_{};

private:
    // 判断大小装甲板类型相关参数
    std::list<bool> history_;
    int filter_size_ = 5;
    bool is_small_{};

};


#endif //ROBOTBASE_AUTOAIM_H
