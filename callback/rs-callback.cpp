// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>
#include <map>
#include <chrono>
#include <mutex>
#include <thread>

///�ݹ� ������ �񵿱������� ��� ������������ ������ �����ش�.
// The callback example demonstrates asynchronous usage of the pipeline
int main(int argc, char * argv[]) try
{
    //rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);

    /// ī���͵�
    /// Stream �̸�
    /// mutex
    std::map<int, int> counters;
    std::map<int, std::string> stream_names;
    std::mutex mutex;

    /// frame callback �Լ��� �����Ѵ�.
    /// callback�� ���� thread������ �����ų �� �ְ� 
    /// multiple ������� ���ÿ� �ҷ��� �� �ִ�.
    /// �׷��Ƿ� ���� �޸𸮿� ���� ��� ������ lock �Ʒ����� �Ǿ�� �Ѵ�.

    // Define frame callback
    // The callback is executed on a sensor thread and 
    // can be called simultaneously from multiple sensors
    //
    // Therefore any modification to common memory should be done under lock
    auto callback = [&](const rs2::frame& frame)
    {
        ///mutex�� lock���� ���
        std::lock_guard<std::mutex> lock(mutex);
        /// frame�� �޾Ƽ� frameset(frameset�� �� ��쿡 ���̰� �ƴѰ�� false�ε�?)���� fs�� ����
        if (rs2::frameset fs = frame.as<rs2::frameset>())
        {
            /// �ݹ��, ��� ����ȭ�� stream�� �ϳ��� frameset�� �����Ѵ� 
            // With callbacks, all synchronized stream will arrive in a single frameset
            for (const rs2::frame& f : fs)
                counters[f.get_profile().unique_id()]++;
        }
        else
        {
            /// ����ȭ�� �����ϴ� �ϳ��� single frame�� Stream �Ѵ�.
            // Stream that bypass synchronization (such as IMU) will produce single frames
            counters[frame.get_profile().unique_id()]++;
        }
    };

    ///���� device�� ������ encapsulating�� RealSense Pipeline�� �����Ѵ�.
    // Declare RealSense pipeline, encapsulating the actual device and sensors.
    rs2::pipeline pipe;

    ///�ݹ��� ���ؼ� streaming�� �����Ѵ�
    /// �⺻ ���� configuration�� depth�� color stream�� �����ϰ� �ִ�.
    /// ���� ����̽��� IMU �����͸� stream�� �� �ִٸ�, ���̷ο� ���ӵ��踦
    /// default�� stream�� �� ���� ���̴�.
    // Start streaming through the callback with default recommended configuration
    // The default video configuration contains Depth and Color streams
    // If a device is capable to stream IMU data, both Gyro and Accelerometer are enabled by default
    //
    rs2::pipeline_profile profiles = pipe.start(callback);

    /// ������ stream �̸����� ������
    // Collect the enabled streams names
    for (auto p : profiles.get_streams())
        stream_names[p.unique_id()] = p.stream_name();

    std::cout << "RealSense callback sample" << std::endl << std::endl;
    /// ���� while loop
    while (true)
    {
        /// 1�ʵ��� �����带 �Ͻ����� ��Ű�� �Լ�
        std::this_thread::sleep_for(std::chrono::seconds(1));
        /// mutex�� ����-- ��?
        std::lock_guard<std::mutex> lock(mutex);

        /// stream�̸��� �ð��� ��� ��Ÿ����.
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
