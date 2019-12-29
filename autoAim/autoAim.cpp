//
// Created by jsz on 12/19/19.
//

#include "autoAim.h"

using namespace cv;
using namespace std;

double calc_distance(Point2f p1, Point2f p2) {
    return pow(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2), 0.5);
}


cv::Rect ArmorDetector::GetRoi(const cv::Mat &img) {
    Size img_size = img.size();
    Rect rect_tmp = last_target_;
    Rect rect_roi;
    if (rect_tmp.x == 0 || rect_tmp.y == 0
        || rect_tmp.width == 0 || rect_tmp.height == 0
        || lost_count >= 15 || detect_count % 100 == 0) {
        last_target_ = Rect(0, 0, img_size.width, img_size.height);
        rect_roi = Rect(0, 0, img_size.width, img_size.height);
        return rect_roi;
    } else {
        float scale = 2;
        if (lost_count < 30)
            scale = 3;
        else if (lost_count <= 60)
            scale = 4;
        else if (lost_count <= 120)
            scale = 5;

        int w = int(rect_tmp.width * scale);
        int h = int(rect_tmp.height * scale);
        int x = int(rect_tmp.x - (w - rect_tmp.width) * 0.5f);
        int y = int(rect_tmp.y - (h - rect_tmp.height) * 0.5f);

        rect_roi = Rect(x, y, w, h);

        if (makeRectSafe(rect_roi, img_size) == false) {
            rect_roi = Rect(0, 0, img_size.width, img_size.height);
        }
    }
    return rect_roi;
}

bool ArmorDetector::DetectArmor(cv::Mat &img, cv::Point3f &target_3d, cv::Mat &depth_img, cv::Rect roi) {
    Mat roi_image = img(roi);
    Point2f offset_roi_point(roi.x, roi.y);
    vector<LED_bar> LED_bars;
    Mat binary_brightness_img, binary_color_img, gray;
    cvtColor(roi_image, gray, COLOR_BGR2GRAY);
    vector<Mat> BGR_channels;
    split(roi_image, BGR_channels);
    Mat result_img;
    if (color_ = 0) // blue
    {
        subtract(BGR_channels[2], BGR_channels[1], result_img);
    } else {
        subtract(BGR_channels[0], BGR_channels[2], result_img);
    }
    threshold(gray, binary_brightness_img, gray_th_, 255, THRESH_BINARY);
    threshold(result_img, binary_color_img, color_th_, 255, THRESH_BINARY);

#ifdef SHOW_BINART
    imshow("binary_brightness_img", binary_brightness_img);
    imshow("binary_color_img", binary_color_img);
#endif
    vector<vector<Point> > contours_light;
    vector<vector<Point> > contours_brightness;
    findContours(binary_color_img, contours_light, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    findContours(binary_brightness_img, contours_brightness, RETR_EXTERNAL, CHAIN_APPROX_NONE);

    for (auto &contours_brightnes : contours_brightness) {
        double area = contourArea(contours_brightnes);
        if (area < 20 || area > 1e5) {
            continue;
        }
        for (const auto &j : contours_light) {
            if (pointPolygonTest(j, contours_brightnes[0], false) >= 0.0) {
                double length = arcLength(contours_brightnes, true);
                if (length > 15 && length < 4000) {
                    RotatedRect RRect = fitEllipse(contours_brightnes);
#ifdef SHOW_LIGHT_CONTOURS
                    Point2f rect_point[4];
                    RRect.points(rect_point);
                    for (int i = 0; i < 4; i++) {
                        line(img, rect_point[i] + offset_roi_point, rect_point[(i + 1) % 4] + offset_roi_point,
                             Scalar(255, 0, 255), 1);
                    }
#endif
                    if (RRect.angle > 90.0f) {
                        RRect.angle = RRect.angle - 180.0f;
                    }
#ifdef SHOW_LIGHT_CONTOURS
                    putText(img, to_string(RRect.angle), RRect.center + Point2f(2, 2) + offset_roi_point,
                            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
#endif
                    if (fabs(RRect.angle) <= 30) {
                        LED_bar r(RRect);
                        LED_bars.emplace_back(r);

                    }
                }
                break;
            }
        }
    }

    //==========================================possible armor=========================================

    for (size_t i = 0; i < LED_bars.size(); i++) {
        for (size_t j = i + 1; j < LED_bars.size(); j++) {
            armor temp_armor(LED_bars.at(i), LED_bars.at(j));
            if (temp_armor.error_angle < 8.0f)
            {
                if (temp_armor.is_suitable_size())
                {
                    if(temp_armor.get_average_intensity(gray)< 50 )
                    {
                        temp_armor.max_match(LED_bars, i, j);
                    }
                }
            }
        }
    }



    //====================================find final armors============================================
    vector<armor> final_armor_list;
    for(size_t i = 0; i < LED_bars.size() ; i++)
    {
        if(LED_bars.at(i).matched)
        {
            LED_bars.at(LED_bars.at(i).match_index).matched = false; //clear another matching flag
            armor arm_tmp( LED_bars.at(i), LED_bars.at(LED_bars.at(i).match_index));
            final_armor_list.push_back(arm_tmp);
        }
    }

    float dist=1e8;
    bool found_flag = false;
    armor target;
    Point2f roi_center(roi.width/2, roi.height/2);
    float dx,dy;
    for (size_t i = 0; i < final_armor_list.size() ; i++ )
    {
#ifdef FAST_DISTANCE
        dx = fabs(final_armor_list.at(i).center.x - roi_center.x);
        dy = fabs(final_armor_list.at(i).center.y - roi_center.y);
#else
        dx = pow((final_armor_list.at(i).center.x - roi_center.x), 2.0f);
        dy = pow((final_armor_list.at(i).center.y - roi_center.y), 2.0f);
#endif
        if( dx + dy < dist){
            target = final_armor_list.at(i);
            dist = dx + dy;
        }
#ifdef SHOW_FINAL_ARMOR
        final_armor_list.at(i).draw_rect(img, offset_roi_point);
#endif
        found_flag = true;
    }
#ifdef SHOW_ROI
    rectangle(img,roi,Scalar(255,0,255),1);
#endif
}

int ArmorDetector::armorTask(cv::Mat &img, OtherParam other_param) {
    color_ = other_param.color;
    mode_ = other_param.mode;
#ifdef ROI_ENABLE
    Rect roi = GetRoi(img);
#else
    Size img_size = img.size();
    Rect roi = Rect(0,0, img_size.width, img_size.height);
#endif
    Point3f target_3d = {};
    DetectArmor(img, target_3d, img, roi);
}


