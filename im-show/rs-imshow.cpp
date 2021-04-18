// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API

int main(int argc, char * argv[]) try
{
    /// 깊이 이미지를 잘 나타내기 위해서 depth colorizer를 선언한다
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    /// 실제 device와 센서들을 묶어서 RealSense pipeline라인을 선언한다.
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    /// 기본 추천 configuration으로 streaming을 시작한다.
    // Start streaming with default recommended configuration
    pipe.start();

    using namespace cv;
    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);

    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
        /// 다음 freme을 기다려서 얻은 데이터+깊이값 데이터
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame depth = data.get_depth_frame().apply_filter(color_map);

        /// 얻은 frame 사이즈(width와 height)를 얻는다.
        // Query frame size (width and height)
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();

        /// OpenCV Mat을 만들어서 depth data를 넣는다.
        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat image(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);

        /// 화면을 업데이트 한다.
        // Update the window with new data
        imshow(window_name, image);
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}



