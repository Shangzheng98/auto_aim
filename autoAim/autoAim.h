//
// Created by jsz on 12/19/19.
//

#ifndef ROBOTBASE_AUTOAIM_H
#define ROBOTBASE_AUTOAIM_H

#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include "../control.h"

struct OtherParam {
    uint8_t color = 0;
    uint8_t mode = 0;
};

class LED_bar {
public:
    LED_bar() : matched(false) {}

    LED_bar(const cv::RotatedRect &R) {
        rect.angle = R.angle;
        rect.center = R.center;
        rect.size = R.size;
        matched = false;
    }

    cv::RotatedRect rect;
    bool matched;
    size_t match_index;
    float match_factor;
};

class armor {
public:
    armor();

    armor(const LED_bar &left, const LED_bar &right);

    void draw_rect();

    void draw_spot();

    int get_average_intensity(const cv::Mat &img);

    void max_match(std::vector<LED_bar> &LEDs, size_t i, size_t j);

    bool is_suitable_size;

    LED_bar led_bar[2];
    float error_angle;
    cv::Point2i center;
    cv::Rect2i rect;
    int average_intensity;

};

class ArmorDetector {
public:
    ArmorDetector() {
        t_start_ = cv::getTickCount();
    }

    ~ArmorDetector() {}

    int armorTask(cv::Mat &img, OtherParam other_param);

    int DetectArmor(cv::Mat& img, cv::Point3f &target_3d, cv::Mat &depth_img, cv::Rect roi);

    void DetectLights(const cv::Mat &src, std::vector<cv::RotatedRect> &lights);

    void FilterLights(std::vector<cv::RotatedRect> &lights);

    void PossibleArmors(const std::vector<cv::RotatedRect> &lights, std::vector<armor> &armors);

    void FilterArmors(std::vector<armor> &armors);

    armor SlectFinalArmor(std::vector<armor> &armors);

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
