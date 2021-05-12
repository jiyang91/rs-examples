// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering

/// Capture 예제는 어떻게 depth 와 color 비디오 streams를 
/// 어떻게 capture하는지와 스크린에서 render하는지 보여준다.
// Capture Example demonstrates how to
// capture depth and color video streams and render them to the screen
int main(int argc, char * argv[]) try
{
    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
    /// 랜더링을 위한 OpenGL 창을 만든다.
    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense Capture Example");

    /// Depth data의 예쁜 visualization Depth colorizer를 선언한다.
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    /// 가능한 stream들의 streaming rates를 보여주기 위한 rates printer를 선언한다  
    // Declare rates printer for showing streaming rates of the enabled streams.
    rs2::rates_printer printer;

    /// 실제 디바이스와 센서들을 캡슐화하는, Realsense pipeline을 선언한다
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    /// 추천되는 default configuration의 streaming을 시작한다
    /// 기본 비디오 configuration은 Depth와 color stream들을 포함한다.
    /// 만약 디바이스가 IMU 데이터, Gyro, Accelerometer가 stream 가능하다면 default로 가능화 됨
    // Start streaming with default recommended configuration
    // The default video configuration contains Depth and Color streams
    // If a device is capable to stream IMU data, both Gyro and Accelerometer are enabled by default
    pipe.start();

    while (app) // Application still alive?
    {
        rs2::frameset data = pipe.wait_for_frames().    // Wait for next set of frames from the camera      /// 다음 frame들 세트를 기다린다.
                             apply_filter(printer).     // Print each enabled stream frame rate             /// 각 가능한 stream frame rate를 프린트
                             apply_filter(color_map);   // Find and colorize the depth data                 /// depth data를 찾고 colorize한다.
        
        /// Show method는, frameset에 적용할때, frame들로 쪼개고 각 frame을 gl texture들에 업로드한다.
        /// 각 texture 그것의 stream unique ID에 따라서 다른 viewport에 display한다.
        // The show method, when applied on frameset, break it to frames and upload each frame into a gl textures
        // Each texture is displayed on different viewport according to it's stream unique id
        app.show(data);
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