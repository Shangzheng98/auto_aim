//
// Created by jsz on 12/27/19.
//
#include "autoAim/autoAim.h"
#include <stdlib.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
int main() {

    rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 60);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
    rs2::pipeline_profile pipelineProfile = pipe.start(cfg);

    pipelineProfile.get_device().query_sensors()[1].set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
    pipelineProfile.get_device().query_sensors()[1].set_option(RS2_OPTION_EXPOSURE, 18);
    pipelineProfile.get_device().query_sensors()[1].set_option(RS2_OPTION_GAIN, 0);

    pipelineProfile.get_device().query_sensors()[0].set_option(RS2_OPTION_LASER_POWER, 360);
    //Instruct pipeline to start streaming with the requested configuration


    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    rs2::frameset frames;
    rs2::colorizer color_map;

    OtherParam otherParam;
    otherParam.color = 0;
    ArmorDetector armorDetector;
    while (1) {
        frames = pipe.wait_for_frames();
        rs2::frame color_frame = frames.get_color_frame();
        //rs2::frame depth_frame = frames.get_depth_frame().apply_filter(color_map);
        Mat color(Size(640, 480), CV_8UC3, (void *) color_frame.get_data(), Mat::AUTO_STEP);
        //Mat depth(Size(640, 480), CV_8UC3, (void *) depth_frame.get_data(), Mat::AUTO_STEP);
        //imshow("depth", depth);
        double time0 = static_cast<double>(getTickCount());
        armorDetector.armorTask(color, frames, otherParam);
        time0 = ((double)getTickCount() - time0) / getTickFrequency();
        std::cout << "use time is " << time0*1000<< "ms"<< std::endl;
        //imshow("src", color);
        if (waitKey(1) == 'q') {

            break;
        }
    }
    pipe.stop();
    destroyAllWindows();
    return 0;
}
