// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>
#include <map>
#include <chrono>
#include <mutex>
#include <thread>

///콜백 예제는 비동기적으로 어떻게 파이프라인을 쓰는지 보여준다.
// The callback example demonstrates asynchronous usage of the pipeline
int main(int argc, char * argv[]) try
{
    //rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);

    /// 카운터들
    /// Stream 이름
    /// mutex
    std::map<int, int> counters;
    std::map<int, std::string> stream_names;
    std::mutex mutex;

    /// frame callback 함수를 정의한다.
    /// callback은 센서 thread위에서 실행시킬 수 있고 
    /// multiple 센서들과 동시에 불러질 수 있다.
    /// 그러므로 공통 메모리에 대한 어떠한 수정도 lock 아래에서 되어야 한다.

    // Define frame callback
    // The callback is executed on a sensor thread and 
    // can be called simultaneously from multiple sensors
    //
    // Therefore any modification to common memory should be done under lock
    auto callback = [&](const rs2::frame& frame)
    {
        ///mutex를 lock으로 사용
        std::lock_guard<std::mutex> lock(mutex);
        /// frame을 받아서 frameset(frameset이 된 경우에 참이고 아닌경우 false인듯?)으로 fs에 전달
        if (rs2::frameset fs = frame.as<rs2::frameset>())
        {
            /// 콜백들, 모두 동기화된 stream이 하나의 frameset에 도착한다 
            // With callbacks, all synchronized stream will arrive in a single frameset
            for (const rs2::frame& f : fs)
                counters[f.get_profile().unique_id()]++;
        }
        else
        {
            /// 동기화를 경유하는 하나의 single frame을 Stream 한다.
            // Stream that bypass synchronization (such as IMU) will produce single frames
            counters[frame.get_profile().unique_id()]++;
        }
    };

    ///실제 device와 센서를 encapsulating한 RealSense Pipeline을 선언한다.
    // Declare RealSense pipeline, encapsulating the actual device and sensors.
    rs2::pipeline pipe;

    ///콜백을 통해서 streaming을 시작한다
    /// 기본 비디오 configuration은 depth와 color stream을 포함하고 있다.
    /// 만약 디바이스가 IMU 데이터를 stream할 수 있다면, 자이로와 가속도계를
    /// default로 stream할 수 있을 것이다.
    // Start streaming through the callback with default recommended configuration
    // The default video configuration contains Depth and Color streams
    // If a device is capable to stream IMU data, both Gyro and Accelerometer are enabled by default
    //
    rs2::pipeline_profile profiles = pipe.start(callback);

    /// 가능한 stream 이름들을 모은다
    // Collect the enabled streams names
    for (auto p : profiles.get_streams())
        stream_names[p.unique_id()] = p.stream_name();

    std::cout << "RealSense callback sample" << std::endl << std::endl;
    /// 무한 while loop
    while (true)
    {
        /// 1초동안 스레드를 일시중지 시키는 함수
        std::this_thread::sleep_for(std::chrono::seconds(1));
        /// mutex를 선언-- 왜?
        std::lock_guard<std::mutex> lock(mutex);

        /// stream이름과 시간을 계속 나타낸다.
        std::cout << "\r";
        for (auto p : counters)
        {
            std::cout << stream_names[p.first] << "[" << p.first << "]: " << p.second << " [frames] || ";
        }
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
