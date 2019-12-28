//
// Created by jsz on 12/19/19.
//

#include "autoAim.h"

using namespace cv;
using namespace std;

double calc_distance(Point2f p1, Point2f p2) {
    return pow(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2), 0.5);
}

int armor::get_average_intensity(const cv::Mat &img) {
    return 0;
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

int ArmorDetector::DetectArmor(cv::Mat& img, cv::Point3f &target_3d, cv::Mat &depth_img, cv::Rect roi)
{
    Mat roi_image = img(roi);
    Point2f  offset_roi_point(roi.x,roi.y);
    vector<LED_bar> LED_bars;
    Mat binary_brightness_img, binary_color_img, gray;
    cvtColor(roi_image,gray,COLOR_BGR2GRAY);
    vector<Mat>  BGR_channels;
    split(roi_image,BGR_channels);
    Mat result_img;
    if (color_ = 0) // blue
    {
        subtract(BGR_channels[2], BGR_channels[1], result_img);
    } else
    {
        subtract(BGR_channels[0], BGR_channels[2], result_img);
    }
    threshold(gray,binary_brightness_img,gray_th_,255, THRESH_BINARY);
    threshold(result_img, binary_color_img, color_th_, 255, THRESH_BINARY);

#ifdef SHOW_BINART
    imshow("binary_brightness_img", binary_brightness_img);
    imshow("binary_color_img", binary_color_img);
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
    DetectArmor(img,target_3d,img,roi);
}

