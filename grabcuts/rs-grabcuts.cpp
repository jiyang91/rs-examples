// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include "../cv-helpers.hpp"    // Helper functions for conversions between RealSense and OpenCV

int main(int argc, char * argv[]) try
{
    using namespace cv;
    using namespace rs2;

    /// 컬러라이져를 process block를 align
    // Define colorizer and align processing-blocks
    colorizer colorize;
    align align_to(RS2_STREAM_COLOR);


    /// 카메라 시작!
    // Start the camera
    pipeline pipe;
    pipe.start();

    /// openCV 창 열기
    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);

    /// 팽창과 침식의 구조를 얻는다. 팽창 침식은 자체 구조element가 요소로 들어가게 된다.
    /// 람다함수를 이용하여 처리
    // We are using StructuringElement for erode / dilate operations
    auto gen_element = [](int erosion_size)
    {
        return getStructuringElement(MORPH_RECT,
            Size(erosion_size + 1, erosion_size + 1),
            Point(erosion_size, erosion_size));
    };

    /// 각 erode를 키울때와 줄일때의 사이즈를 아래와 같이 선언 할 수 있음
    const int erosion_size = 3;
    auto erode_less = gen_element(erosion_size);
    auto erode_more = gen_element(erosion_size * 2);

    /// depth로 부터 마스크 작성
    /// Threshold를 하고 그 위에 small hole들을 erode를 이용하여 제거한다.
    // The following operation is taking grayscale image,
    // performs threashold on it, closes small holes and erodes the white area
    auto create_mask_from_depth = [&](Mat& depth, int thresh, ThresholdTypes type)
    {
        threshold(depth, depth, thresh, 255, type);
        dilate(depth, depth, erode_less);
        erode(depth, depth, erode_more);
    };

    /// auto-exposure stabilization을 위해서 몇 frame들을 넘어간다.
    // Skips some frames to allow for auto-exposure stabilization
    for (int i = 0; i < 10; i++) pipe.wait_for_frames();

    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
        /// frameset pipeline으로부터 기다려서 받는다
        frameset data = pipe.wait_for_frames();
        
        /// 칼라 이미지의 픽셀과 깊이 이미지의 픽셀을 맞춘다.
        /// Frameset이 공간적으로 aligned되어 있다.
        // Make sure the frameset is spatialy aligned 
        // (each pixel in depth image corresponds to the same pixel in the color image)
        frameset aligned_set = align_to.process(data);
        frame depth = aligned_set.get_depth_frame();
        auto color_mat = frame_to_mat(aligned_set.get_color_frame());

        /// 흰이미지는 가깝게 검은 건 멀게 depth 이미지를 colorize 한다
        /// 이것은 Colorizer를 통해서 histogram equalization을 하는 이점이 있다.
        // Colorize depth image with white being near and black being far
        // This will take advantage of histogram equalization done by the colorizer
        colorize.set_option(RS2_OPTION_COLOR_SCHEME, 2);
        frame bw_depth = depth.apply_filter(colorize);

        /// "가까운" 마스크 이미지를 만든다.
        // Generate "near" mask image:
        auto near = frame_to_mat(bw_depth);
        cvtColor(near, near, COLOR_BGR2GRAY);

        /// 180~255 범위의 값을 가져온다.
        /// 이것들은 historam equalization 때문에 rough하게 가까운 object들과 동일하다 
        // Take just values within range [180-255]
        // These will roughly correspond to near objects due to histogram equalization
        create_mask_from_depth(near, 180, THRESH_BINARY);

        /// "먼" 마스크 이미지를 만든다.
        // Generate "far" mask image:
        auto far = frame_to_mat(bw_depth);
        cvtColor(far, far, COLOR_BGR2GRAY);
        far.setTo(255, far == 0); // Note: 0 value does not indicate pixel near the camera, and requires special attention 
        create_mask_from_depth(far, 100, THRESH_BINARY_INV);

        /// Grabcut 알고리즘은 모든 픽셀이 마크(BGD, FGB, PR_BGD, PR_FGB)들로 
        /// 마킹된 마스크를 필요로 한다.
        /// BGD:    Background
        /// FGB:    Foreground
        /// PR_BGD: Probably_BGD
        /// PR_FGB: Probably_FGD
        // GrabCut algorithm needs a mask with every pixel marked as either:
        // BGD, FGB, PR_BGD, PR_FGB
        Mat mask;
        mask.create(near.size(), CV_8UC1); 
        mask.setTo(Scalar::all(GC_BGD)); // Set "background" as default guess
        mask.setTo(GC_PR_BGD, far == 0); // Relax this to "probably background" for pixels outside "far" region
        mask.setTo(GC_FGD, near == 255); // Set pixels within the "near" region to "foreground"

        /// Grab-Cut algorithm을 돌린다.
        // Run Grab-Cut algorithm:
        Mat bgModel, fgModel; 
        grabCut(color_mat, mask, Rect(), bgModel, fgModel, 1, GC_INIT_WITH_MASK);

        /// 알고리즘으로 얻은 마스크에 Foreground에 속하는 픽셀을 얻는다.
        // Extract foreground pixels based on refined mask from the algorithm
        Mat3b foreground = Mat3b::zeros(color_mat.rows, color_mat.cols);
        color_mat.copyTo(foreground, (mask == GC_FGD) | (mask == GC_PR_FGD));
        imshow(window_name, foreground);
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



