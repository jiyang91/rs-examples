// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>
#include "../example.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"

/*
*  Spatial stream alignment의 개념을 소개하는 예제이다.
* 이 예제의 사용처를 확인하려면, align-advanced와 measure demo들을 확인하세요.
* Spatial alignment(이제부터 "align"으로 표기)의 필요는 모든 카메라 stream들이
* single viewport에서 capture되어 나오지 않는다는 사실 때문입니다.
* Align 과정은 이미지를 유저가 이미지들을 한 viewport에서 다른 것으로 translate
* 시키는 과정입니다.
* 따라서, align의 결과는 합성된 stream들이며, 몇가지 artifact들에 영향을 받습니다:
* 1. Sampling - mapping stream에서 다른 viewport는 프래임의 해상도 맞추기 위해서 
* 타겟 viewport의 해상도에 수정한다. 이런 수정은 interpolation을 통해서 downsampling이나 
* upsampling을 유발한다. 사용된 Interpolation은 non-existing 값들을 피하기 위해서 
* Nearest Neighbor의 종류를 필요로 한다.
* 
* 2. Occlusion - 결과 이미지의 몇몇 픽셀들은 Original 센서가 보지 못했던 3차원 
* 좌표들에 대응한다. 왜냐하면 이 3차원 점들은 original viewport에서 겹쳤기 때문이다.
* 이러한 픽셀들은 부적절한 격자값들을 갖게 된다.
*/

/*
 This example introduces the concept of spatial stream alignment.
 For example usecase of alignment, please check out align-advanced and measure demos.
 The need for spatial alignment (from here "align") arises from the fact
 that not all camera streams are captured from a single viewport.
 Align process lets the user translate images from one viewport to another. 
 That said, results of align are synthetic streams, and suffer from several artifacts:
 1. Sampling - mapping stream to a different viewport will modify the resolution of the frame 
               to match the resolution of target viewport. This will either cause downsampling or
               upsampling via interpolation. The interpolation used needs to be of type
               Nearest Neighbor to avoid introducing non-existing values.
 2. Occlussion - Some pixels in the resulting image correspond to 3D coordinates that the original
               sensor did not see, because these 3D points were occluded in the original viewport.
               Such pixels may hold invalid texture values.
*/

/// 이 예제는 깊이와 칼라 카메라 stream들을 가정한다.
/// 그리고 direction은 원하는 target stream을 정의한다.
// This example assumes camera with depth and color
// streams, and direction lets you define the target stream
enum class direction
{
    to_depth,
    to_color
};

/// 아래에 implemente 된 UI 랜더링을 위한 Forward definition
// Forward definition of UI rendering, implemented below
void render_slider(rect location, float* alpha, direction* dir);

int main(int argc, char * argv[]) try
{
    ///GUI 관련 객체들을 만들고 초기화 한다.
    // Create and initialize GUI related objects
    window app(1280, 720, "RealSense Align Example"); // Simple window handling     간단한 윈도우 핸들링
    ImGui_ImplGlfw_Init(app, false);      // ImGui library intializition            ImGui 라이브러리 초기화
    rs2::colorizer c;                     // Helper to colorize depth images        깊이 이미지들을 colorize하는 Helper
    texture depth_image, color_image;     // Helpers for renderig images            이미지들을 랜더링하는 Helper

    /// 쉽게 카메라를 구성하고 시작하기 위한 pipeline을 만든다
    // Create a pipeline to easily configure and start the camera
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH);
    cfg.enable_stream(RS2_STREAM_COLOR);
    pipe.start(cfg);

    /// 두 align 객체들을 정의한다. 하나는 깊이 viewport를
    /// 다른 것을 color를 align하는 것이다. 
    /// align 객체를 만드는 것은 main loop에서 사용해선 안되는
    /// expensive operation이다.  
    /// 
    // Define two align objects. One will be used to align
    // to depth viewport and the other to color.
    // Creating align object is an expensive operation
    // that should not be performed in the main loop
    rs2::align align_to_depth(RS2_STREAM_DEPTH);
    rs2::align align_to_color(RS2_STREAM_COLOR);

    float       alpha = 0.5f;               // Transparancy coefficient /// 투명 변수
    direction   dir = direction::to_depth;  // Alignment direction      /// Alignment의 방향

    while (app) // Application still alive?
    {
        /// align 객체를 사용하기 위해서, frameset 하나가 준비되기 전까지 application을 막는다
        // Using the align object, we block the application until a frameset is available
        rs2::frameset frameset = pipe.wait_for_frames();

        if (dir == direction::to_depth)
        {
            /// 모든 frame들을 depth viewport로 align한다.
            // Align all frames to depth viewport
            frameset = align_to_depth.process(frameset);
        }
        else
        {
            /// 모든 frame들을 color viewport로 align한다.
            // Align all frames to color viewport
            frameset = align_to_color.process(frameset);
        }

        /// Align된 frameset과 함께 보통과 같이 진행한다.
        // With the aligned frameset we proceed as usual
        auto depth = frameset.get_depth_frame();
        auto color = frameset.get_color_frame();
        auto colorized_depth = c.colorize(depth);

        glEnable(GL_BLEND);
        // Use the Alpha channel for blending   /// Blending을 위해서 Alpha 채널을 사용한다.
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        if (dir == direction::to_depth)
        {
            /// 깊이로 align할 때, 일단 depth 이미지를 랜더링하고
            /// color를 투명으로 위에 overlay한다.
            // When aligning to depth, first render depth image
            // and then overlay color on top with transparancy
            depth_image.render(colorized_depth, { 0, 0, app.width(), app.height() });
            color_image.render(color, { 0, 0, app.width(), app.height() }, alpha);
        }
        else
        {
            /// Color로 align할 때, 일단 color 이미지를 랜더링하고
            /// depth를 투명으로 위에 overlay한다.
            // When aligning to color, first render color image
            // and then overlay depth image on top
            color_image.render(color, { 0, 0, app.width(), app.height() });
            depth_image.render(colorized_depth, { 0, 0, app.width(), app.height() }, 1 - alpha);
        }

        glColor4f(1.f, 1.f, 1.f, 1.f);
        glDisable(GL_BLEND);
        /// UI를 랜더링 한다.
        // Render the UI:
        ImGui_ImplGlfw_NewFrame(1);
        render_slider({ 15.f, app.height() - 60, app.width() - 30, app.height() }, &alpha, &dir);
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

void render_slider(rect location, float* alpha, direction* dir)
{
    static const int flags = ImGuiWindowFlags_NoCollapse
        | ImGuiWindowFlags_NoScrollbar
        | ImGuiWindowFlags_NoSavedSettings
        | ImGuiWindowFlags_NoTitleBar
        | ImGuiWindowFlags_NoResize
        | ImGuiWindowFlags_NoMove;

    ImGui::SetNextWindowPos({ location.x, location.y });
    ImGui::SetNextWindowSize({ location.w, location.h });
    ///투명한 slider를 랜더링한다.
    // Render transparency slider:
    ImGui::Begin("slider", nullptr, flags);
    ImGui::PushItemWidth(-1);
    ImGui::SliderFloat("##Slider", alpha, 0.f, 1.f);
    ImGui::PopItemWidth();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Texture Transparancy: %.3f", *alpha);

    /// 랜더링 방향을 찾는 checkbox
    // Render direction checkboxes:
    bool to_depth = (*dir == direction::to_depth);
    bool to_color = (*dir == direction::to_color);

    if (ImGui::Checkbox("Align To Depth", &to_depth))
    {
        *dir = to_depth ? direction::to_depth : direction::to_color;
    }
    ImGui::SameLine();
    ImGui::SetCursorPosX(location.w - 140);
    if (ImGui::Checkbox("Align To Color", &to_color))
    {
        *dir = to_color ? direction::to_color : direction::to_depth;
    }

    ImGui::End();
}
