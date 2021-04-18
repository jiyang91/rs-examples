// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>             // for cout

// Hello RealSense example demonstrates the basics of connecting to a RealSense device
// and taking advantage of depth data
int main(int argc, char * argv[]) try
{
    ///Pipeline을 만듭니다. - 이것은 frame의 스트리밍과 프로세싱을 보조하는 top-level API입니다.
    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;

    /// 파이프라인을 나타내고 시작한다.
    // Configure and start the pipeline
    p.start();

    while (true)
    {
        /// frame이 도착할 때까지 프로그램을 막는다.
        // Block program until frames arrive
        rs2::frameset frames = p.wait_for_frames();

        /// 깊이 이미지의 한 프레임을 얻는다.
        // Try to get a frame of a depth image
        rs2::depth_frame depth = frames.get_depth_frame();

        /// 깊이 이미지의 차원정보를 얻는다.
        // Get the depth frame's dimensions
        float width = depth.get_width();
        float height = depth.get_height();

        /// 카메라로부터 이미지 중심의 물체까지의 거리를 얻는다.
        // Query the distance from the camera 
        // to the object in the center of the image
        float dist_to_center = depth.get_distance(width / 2, height / 2);

        /// 거리를 print한다.
        // Print the distance
        std::cout << "The camera is facing an object " << dist_to_center << " meters away \r";
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
