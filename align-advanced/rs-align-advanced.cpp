// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>
#include "../example.hpp"
#include <imgui.h>
#include "imgui_impl_glfw.h"

#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cstring>

void render_slider(rect location, float& clipping_dist);
void remove_background(rs2::video_frame& other, const rs2::depth_frame& depth_frame, float depth_scale, float clipping_dist);
float get_depth_scale(rs2::device dev);
rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);

int main(int argc, char * argv[]) try
{
    /// GUI 를 glfw를 이용하여 만듬
    // Create and initialize GUI related objects
    window app(1280, 720, "RealSense Align (Advanced) Example"); // Simple window handling      /// 간단한 창 핸들링
    ImGui_ImplGlfw_Init(app, false);      // ImGui library intializition                        /// ImGui 라이브러리 초기화
    rs2::colorizer c;                     // Helper to colorize depth images                    /// 깊이 이미지 colorize 핼퍼
    texture renderer;                     // Helper for renderig images                         /// 이미지들 랜터링 핼퍼

    /// 쉽게 카메라를 동작시키기 위해서 Pipeline을 만든다 
    // Create a pipeline to easily configure and start the camera
    rs2::pipeline pipe;
    /// pipleline의 start를 다른 파라미터 없이 실행하면 default 설정에 따라 첫 디바이스를 시작하게 된다.
    //Calling pipeline's start() without any additional parameters will start the first device
    // with its default streams.
    //The start function returns the pipeline profile which the pipeline used to start the device
    rs2::pipeline_profile profile = pipe.start();

    /// 각 depth 카메라 깊이 pixels들의 다른 단위들을 갖고 있을 수 있다, 따라서 
    /// 파이프라인의 프로파일을 이용하여, 디바이스의 그 값을 얻는다.
    // Each depth camera might have different units for depth pixels, so we get it here
    // Using the pipeline's profile, we can retrieve the device that the pipeline uses
    float depth_scale = get_depth_scale(profile.get_device());

    /// Color stream이 없는 파이프라인 장치를 선택할 수 있다.
    /// 마약 color stream이 없다면, depth를 다른 stream에 align한다.
    //Pipeline could choose a device that does not have a color stream
    //If there is no color stream, choose to align depth to another stream
    rs2_stream align_to = find_stream_to_align(profile.get_streams());

    /// rs2::align 객체를 만든다.
    /// rs2::align은 depth frame들을 다른 frame들에 alignment할 수 있도록 한다.
    /// "align_to"는 stream type으로 depth frame들에 align하도록 한다.
    // Create a rs2::align object.
    // rs2::align allows us to perform alignment of depth frames to others frames
    //The "align_to" is the stream type to which we plan to align depth frames.
    rs2::align align(align_to);

    /// clip까지 거리를 조절하는 변수를 정의한다. 
    // Define a variable for controlling the distance to clip
    float depth_clipping_distance = 1.f;

    while (app) // Application still alive?
    {
        /// align 객체를 이용하여, frameset이 가능하기 전까지 application을 막는다
        // Using the align object, we block the application until a frameset is available
        rs2::frameset frameset = pipe.wait_for_frames();

        ///rs2::pipleline::wait_for_frames()는 장치 에러나 연결종료 시 장치를 대체할 수 있다.
        /// rs2::align이 depth에서 다른 stream으로 align하기 때문에, 그 stream이 바뀌어서는 안됩니다.
        /// wait_for_frames()가 호출된 후로는.
        // rs2::pipeline::wait_for_frames() can replace the device it uses in case of device error or disconnection.
        // Since rs2::align is aligning depth to some other stream, we need to make sure that the stream was not changed
        //  after the call to wait_for_frames();
        if (profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams()))
        {
            /// 만약 프로파일이 바뀌었다면, align 객체를 업데이트 하고, 새로운 장치의 깊이 scale을 얻습니다.
            //If the profile was changed, update the align object, and also get the new device's depth scale
            profile = pipe.get_active_profile();
            align_to = find_stream_to_align(profile.get_streams());
            align = rs2::align(align_to);
            depth_scale = get_depth_scale(profile.get_device());
        }

        /// 처리된 aligned frame을 얻습니다.
        //Get processed aligned frame
        auto processed = align.process(frameset);

        /// 다른 frame과 aligned된 depth frame을 얻기를 시도한다.
        // Trying to get both other and aligned depth frames
        rs2::video_frame other_frame = processed.first(align_to);
        rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();

        /// 하나라도 불가능하면 iteration을 반복한다.
        //If one of them is unavailable, continue iteration
        if (!aligned_depth_frame || !other_frame)
        {
            continue;
        }

        /// 양 프레임들을 remove_background로 넘긴다. 그럼 이게 배경을 "제거" 할 것이다.
        /// NOTE: 이 예제에서, 다른 프레임들의 버퍼를 교환한다. 복사하는대신에 그리고 바꾼다 그 카피로
        /// 이 행동은 실제 어플리케이션에서 추천되지 않는데 다른 프레임이 다르게 사용될 수도 있기 때문이다.
        // Passing both frames to remove_background so it will "strip" the background
        // NOTE: in this example, we alter the buffer of the other frame, instead of copying it and altering the copy
        //       This behavior is not recommended in real application since the other frame could be used elsewhere
        remove_background(other_frame, aligned_depth_frame, depth_scale, depth_clipping_distance);

        /// rendering 목적들로 창에서 차원들을 얻는다 
        // Taking dimensions of the window for rendering purposes
        float w = static_cast<float>(app.width());
        float h = static_cast<float>(app.height());

        /// 이 시점에서, "other_frame"은 바뀐 frame이다, 이것의 배경이 제거된다.
        /// 창에서의 프레임의 위치를 계산한다.
        // At this point, "other_frame" is an altered frame, stripped form its background
        // Calculating the position to place the frame in the window
        rect altered_other_frame_rect{ 0, 0, w, h };
        altered_other_frame_rect = altered_other_frame_rect.adjust_ratio({ static_cast<float>(other_frame.get_width()),static_cast<float>(other_frame.get_height()) });

        /// Aligned image를 랜더링한다.
        // Render aligned image
        renderer.render(other_frame, altered_other_frame_rect);

        /// 예제 또한 깊이 프레임을 사진-속-사진으로 랜더링했다
        /// 위치를 깊이 프레임  계산한다.
        // The example also renders the depth frame, as a picture-in-picture
        // Calculating the position to place the depth frame in the window
        rect pip_stream{ 0, 0, w / 5, h / 5 };
        pip_stream = pip_stream.adjust_ratio({ static_cast<float>(aligned_depth_frame.get_width()),static_cast<float>(aligned_depth_frame.get_height()) });
        pip_stream.x = altered_other_frame_rect.x + altered_other_frame_rect.w - pip_stream.w - (std::max(w, h) / 25);
        pip_stream.y = altered_other_frame_rect.y + (std::max(w, h) / 25);

        /// Depth 랜더링한다.(사진속 사진으로)
        // Render depth (as picture in pipcture)
        renderer.upload(c.process(aligned_depth_frame));
        renderer.show(pip_stream);
        
        /// 공급하기 ImGui 라이브러리 
        // Using ImGui library to provide a slide controller to select the depth clipping distance
        ImGui_ImplGlfw_NewFrame(1);
        render_slider({ 5.f, 0, w, h }, depth_clipping_distance);
        ImGui::Render();

    }
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

float get_depth_scale(rs2::device dev)
{
    /// Device의 센서들
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        /// 그 센서가 깊이 센서라면
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

void render_slider(rect location, float& clipping_dist)
{
    /// Control을 더 잘 보여주는 기교
    // Some trickery to display the control nicely
    static const int flags = ImGuiWindowFlags_NoCollapse
        | ImGuiWindowFlags_NoScrollbar
        | ImGuiWindowFlags_NoSavedSettings
        | ImGuiWindowFlags_NoTitleBar
        | ImGuiWindowFlags_NoResize
        | ImGuiWindowFlags_NoMove;
    const int pixels_to_buttom_of_stream_text = 25;
    const float slider_window_width = 30;

    ImGui::SetNextWindowPos({ location.x, location.y + pixels_to_buttom_of_stream_text });
    ImGui::SetNextWindowSize({ slider_window_width + 20, location.h - (pixels_to_buttom_of_stream_text * 2) });
    /// Vertical slider를 랜더링한다.
    //Render the vertical slider
    ImGui::Begin("slider", nullptr, flags);
    ImGui::PushStyleColor(ImGuiCol_FrameBg, ImColor(215.f / 255, 215.0f / 255, 215.0f / 255));
    ImGui::PushStyleColor(ImGuiCol_SliderGrab, ImColor(215.f / 255, 215.0f / 255, 215.0f / 255));
    ImGui::PushStyleColor(ImGuiCol_SliderGrabActive, ImColor(215.f / 255, 215.0f / 255, 215.0f / 255));
    auto slider_size = ImVec2(slider_window_width / 2, location.h - (pixels_to_buttom_of_stream_text * 2) - 20);
    ImGui::VSliderFloat("", slider_size, &clipping_dist, 0.0f, 6.0f, "", 1.0f, true);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Depth Clipping Distance: %.3f", clipping_dist);
    ImGui::PopStyleColor(3);

    /// bar들을 slider 옆에 display한다.
    //Display bars next to slider
    float bars_dist = (slider_size.y / 6.0f);
    for (int i = 0; i <= 6; i++)
    {
        ImGui::SetCursorPos({ slider_size.x, i * bars_dist });
        std::string bar_text = "- " + std::to_string(6-i) + "m";
        ImGui::Text("%s", bar_text.c_str());
    }
    ImGui::End();
}

void remove_background(rs2::video_frame& other_frame, const rs2::depth_frame& depth_frame, float depth_scale, float clipping_dist)
{
    const uint16_t* p_depth_frame = reinterpret_cast<const uint16_t*>(depth_frame.get_data());
    uint8_t* p_other_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(other_frame.get_data()));

    int width = other_frame.get_width();
    int height = other_frame.get_height();
    int other_bpp = other_frame.get_bytes_per_pixel();

    #pragma omp parallel for schedule(dynamic) //Using OpenMP to try to parallelise the loop
    for (int y = 0; y < height; y++)
    {
        auto depth_pixel_index = y * width;
        for (int x = 0; x < width; x++, ++depth_pixel_index)
        {
            /// 현재 픽셀의 깊이 값을 얻는다.
            // Get the depth value of the current pixel
            auto pixels_distance = depth_scale * p_depth_frame[depth_pixel_index];

            /// 깊이 값이 threshold 범위보다 크거나 invalid 한 범위에 있는지 확인.
            // Check if the depth value is invalid (<=0) or greater than the
            // threshold
            if (pixels_distance <= 0.f || pixels_distance > clipping_dist)
            {
                /// 다른 frame의 버퍼로 부터 현재 픽셀의 offset을 계산한다. 
                // Calculate the offset in other frame's buffer to current pixel
                auto offset = depth_pixel_index * other_bpp;
                /// Pixel 을 "background" 칼라로 설정함
                // Set pixel to "background" color (0x999999)
                std::memset(&p_other_frame[offset], 0x99, other_bpp);
            }
        }
    }
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
    /// streams의 백터가 주어졌을 때, depth stream과 depth에 align할 또 다른 stream을 찾는다.
    /// 잘 보이게 하기 위해서 color stream들을 최우선으로 한다.
    /// 만약 color가 가능하지 않다면, 또 다른 stream을 고른다.
    //Given a vector of streams, we try to find a depth stream and another stream to align depth with.
    //We prioritize color streams to make the view look better.
    //If color is not available, we take another stream that (other than depth)
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for (rs2::stream_profile sp : streams)
    {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream != RS2_STREAM_DEPTH)
        {
            if (!color_stream_found)         //Prefer color
                align_to = profile_stream;

            if (profile_stream == RS2_STREAM_COLOR)
            {
                color_stream_found = true;
            }
        }
        else
        {
            depth_stream_found = true;
        }
    }

    if(!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}

bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
    for (auto&& sp : prev)
    {
        /// 만약 이전 프로파일이 current에 있다면(?) (아마도 또 다른 더해진)
        //If previous profile is in current (maybe just added another)
        auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
        /// 만약 이전 stream이 current에서 찾아지지 않았다면,
        if (itr == std::end(current)) //If it previous stream wasn't found in current
        {
            return true;
        }
    }
    return false;
}
