// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <array>
#include <cmath>
#include <iostream>
#include <vector>
#include "example.hpp"

/// 3차원 점을 struct로 정의
/// x, y, z 각 방향을 정의하여 객체를 만들거나
/// 각 점에 접근할 수 있도록 구성
/// 그 이상의 연산같은 것도 외부에서 쓸 것
struct point3d
{
    float f[3];

    point3d() {}
    point3d(float x, float y, float z) : f{x, y, z} {}
    float x() const { return f[0]; }
    float y() const { return f[1]; }
    float z() const { return f[2]; }
};

/// 이미지의 픽셀을 struct로 정의
/// x, y 각 점을 정의하여 객체를 만들거나
/// 각 점에 접근할 수 있도록 구성
/// 그 이상의 연산같은 것도 외부에서 쓸 것
struct pixel
{
    float f[2];

    pixel() {}
    pixel(float x, float y) : f{x, y} {}
    float x() const { return f[0]; }
    float y() const { return f[1]; }
};

/// 선들로 연결될 vertice들의 모음인 Virtual object를 정의한다.
// We define a virtual object as a collection of vertices that will be connected by lines
typedef std::array<point3d, 4> object;

static rs2_pose identity_pose();
static rs2_pose reset_object_pose(const rs2_pose &device_pose_in_world = identity_pose());
static rs2_pose pose_inverse(const rs2_pose &p);
static rs2_pose pose_multiply(const rs2_pose &ref2_in_ref1, const rs2_pose &ref3_in_ref2);
static rs2_quaternion quaternion_conjugate(const rs2_quaternion &q);
static rs2_quaternion quaternion_multiply(const rs2_quaternion &a, const rs2_quaternion &b);
static rs2_vector quaternion_rotate_vector(const rs2_quaternion &q, const rs2_vector &v);
static rs2_vector pose_transform_point(const rs2_pose &pose, const rs2_vector &p);
static rs2_vector vector_addition(const rs2_vector &a, const rs2_vector &b);
static rs2_vector vector_negate(const rs2_vector &v);

static object convert_object_coordinates(const object &obj, const rs2_pose &object_pose);

static std::vector<point3d> raster_line(const point3d &a, const point3d &b, float step);
static void render_line(const std::vector<pixel> &line, int color_code);
static void render_text(int win_height, const std::string &text);

int main(int argc, char *argv[])
try
{
    std::cout << "Waiting for device..." << std::endl;
    /// RealSense pipeline을 선언하고, 실제 device와 센서들을 캡슐화한다.
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    /// non default profile로 구성된 Pipeline을 구성하기 위한 configuration 을 만든다.
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    /// fisheye와 pose streams를 enable한다.
    // Enable fisheye and pose streams
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 1);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 2);
    /// 선택된 configuation의 pipeline을 시작한다.
    // Start pipeline with chosen configuration
    rs2::pipeline_profile pipe_profile = pipe.start(cfg);

    /// T265는 2개의 fisheye 센서들이 있고, 어떤 것이든 고를 수 있다.
    // T265 has two fisheye sensors, we can choose any of them (index 1 or 2)
    const int fisheye_sensor_idx = 1;

    /// Fisheye 내제 파라미터들을 얻는다.
    // Get fisheye sensor intrinsics parameters
    rs2::stream_profile fisheye_stream = pipe_profile.get_stream(RS2_STREAM_FISHEYE, fisheye_sensor_idx);
    rs2_intrinsics intrinsics = fisheye_stream.as<rs2::video_stream_profile>().get_intrinsics();

    rs2_extrinsics pose_to_fisheye_extrinsics = pipe_profile.get_stream(RS2_STREAM_POSE).get_extrinsics_to(fisheye_stream);

    std::cout << "Device got. Streaming data" << std::endl;

    /// OpenGL 디스플레이 윈도우와 fisheye image를 그리기 위한 texture를 생성한다.
    // Create an OpenGL display window and a texture to draw the fisheye image
    window app(intrinsics.width, intrinsics.height, "Intel RealSense T265 Augmented Reality Example");
    window_key_listener key_watcher(app);
    texture fisheye_image;

    /// 간단한 가상 object의 Vertice들을 생성한다.
    /// 이 가상 오브젝트는 (3 XYZ 20cm 길이의 축들을 묘사하는) 3차원 공간의 4개의 점들이다.
    /// 이 점들은 물체의 좌표계와 상대적인 값이다.
    // Create the vertices of a simple virtual object.
    // This virtual object is 4 points in 3D space that describe 3 XYZ 20cm long axes.
    // These vertices are relative to the object's own coordinate system.
    const float length = 0.20;
    const object virtual_object = {{
        {0, 0, 0},      // origin
        {length, 0, 0}, // X
        {0, length, 0}, // Y
        {0, 0, length}  // Z
    }};

    /// 이 변수는 world 좌표계의 virtual object의 pose를 갖는다.
    /// 일단 첫 pose frame을 얻어서 이 값을 한번 초기화 한다.
    // This variable will hold the pose of the virtual object in world coordinates.
    // We initialize it once we get the first pose frame.
    rs2_pose object_pose_in_world;
    bool object_pose_in_world_initialized = false;

    // Main loop
    while (app)
    {
        rs2_pose device_pose_in_world; // This will contain the current device pose           /// 현재 장치의 pose를 포함시킨다.
        {
            /// 카메라의 다음 set의 프레임을 기다린다.
            // Wait for the next set of frames from the camera
            auto frames = pipe.wait_for_frames();
            /// fisheye stream으로부터 frame을 얻는다.
            // Get a frame from the fisheye stream
            rs2::video_frame fisheye_frame = frames.get_fisheye_frame(fisheye_sensor_idx);
            /// Pose stream으로부터 frame을 얻는다.
            // Get a frame from the pose stream
            rs2::pose_frame pose_frame = frames.get_pose_frame();
            /// 현재 카메라 포스를 카피한다.
            // Copy current camera pose
            device_pose_in_world = pose_frame.get_pose_data();
            /// fisheye이미지를 랜더링한다.
            // Render the fisheye image
            fisheye_image.render(fisheye_frame, {0, 0, app.width(), app.height()});
            /// 현재 scope를 닫음으로 frame들을 deallocate하면, 다른 연산을 하는 동안 librealsense queue를 채우지 않는다.
            // By closing the current scope we let frames be deallocated, so we do not fill up librealsense queues while we do other computation.
        }
        /// world에서 아직 virtual object가 준비되지 않았다면, camera 앞쪽에서 지금 세팅한다.
        // If we have not set the virtual object in the world yet, set it in front of the camera now.
        if (!object_pose_in_world_initialized)
        {
            object_pose_in_world = reset_object_pose(device_pose_in_world);
            object_pose_in_world_initialized = true;
        }

        /// 현재의 device의 자세를 상대적으로 object의 자세를 계산한다.
        // Compute the pose of the object relative to the current pose of the device
        rs2_pose world_pose_in_device = pose_inverse(device_pose_in_world);
        rs2_pose object_pose_in_device = pose_multiply(world_pose_in_device, object_pose_in_world);

        /// Device 좌표계들의 물체 vertice들을 얻는다.
        // Get the object vertices in device coordinates
        object object_in_device = convert_object_coordinates(virtual_object, object_pose_in_device);

        /// object vectice들을 Device 좌표계로부터 외부의 fisheye 센서로 바꾼다.
        // Convert object vertices from device coordinates into fisheye sensor coordinates using extrinsics
        object object_in_sensor;
        for (size_t i = 0; i < object_in_device.size(); ++i)
        {
            rs2_transform_point_to_point(object_in_sensor[i].f, &pose_to_fisheye_extrinsics, object_in_device[i].f);
        }

        for (size_t i = 1; i < object_in_sensor.size(); ++i)
        {
            /// virtual object line을 1cm 길이의 조각들로 분절한다.
            // Discretize the virtual object line into smaller 1cm long segments
            std::vector<point3d> points_in_sensor = raster_line(object_in_sensor[0], object_in_sensor[i], 0.01);
            std::vector<pixel> projected_line;
            projected_line.reserve(points_in_sensor.size());
            for (auto &point : points_in_sensor)
            {
                /// 이미지안에 한 3D point 가 보인다.  만일 이 점의 fisheye sensor로 상대적인 Z 좌표계는 양수이다.
                // A 3D point is visible in the image if its Z coordinate relative to the fisheye sensor is positive.
                if (point.z() > 0)
                {
                    /// 내장함수를 이용하여  3차원 센서 좌표계를 2차원 fisheye 이미지 좌표계로 project한다.
                    // Project 3D sensor coordinates to 2D fisheye image coordinates using intrinsics
                    projected_line.emplace_back();
                    rs2_project_point_to_pixel(projected_line.back().f, &intrinsics, point.f);
                }
            }
            /// 선을 이미지에 display한다.
            // Display the line in the image
            render_line(projected_line, i);
        }

        /// 이미지에 text를 display한다.
        // Display text in the image
        render_text(app.height(), "Press spacebar to reset the pose of the virtual object. Press ESC to exit");

        /// 만약 어떤 키가 눌렸는지 확인한다.
        // Check if some key is pressed
        switch (key_watcher.get_key())
        {
        case GLFW_KEY_SPACE:
            /// 만일 user가 spacebar를 누른다면, virtual object 포즈를 초기화한다
            // Reset virtual object pose if user presses spacebar
            object_pose_in_world = reset_object_pose(device_pose_in_world);
            std::cout << "Setting new pose for virtual object: " << object_pose_in_world.translation << std::endl;
            break;
        case GLFW_KEY_ESCAPE:
            /// User가 escape를 누르면 종료
            // Exit if user presses escape
            app.close();
            break;
        }
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error &e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception &e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

rs2_pose identity_pose()
{
    /// Identity pose로 되돌린다.
    // Return an identity pose (no translation, no rotation)
    rs2_pose pose;
    pose.translation.x = 0;
    pose.translation.y = 0;
    pose.translation.z = 0;
    pose.rotation.x = 0;
    pose.rotation.y = 0;
    pose.rotation.z = 0;
    pose.rotation.w = 1;
    return pose;
}

rs2_pose reset_object_pose(const rs2_pose &device_pose_in_world)
{
    /// object 카메라 앞으로부터 50 cm 멀리 세팅한다.
    /// T265 좌표계는 여기에 정의되어 있다.
    // Set the object 50 centimeter away in front of the camera.
    // T265 coordinate system is defined here: https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md#sensor-origin-and-coordinate-system
    rs2_pose object_pose_in_device;
    object_pose_in_device.translation.x = 0;
    object_pose_in_device.translation.y = 0;
    object_pose_in_device.translation.z = -0.50;
    object_pose_in_device.rotation.x = 0;
    object_pose_in_device.rotation.y = 0;
    object_pose_in_device.rotation.z = 0;
    object_pose_in_device.rotation.w = 1;

    /// Virtual object의 pose를 camera coordinates로부터 world coordinates로 바꾼다.
    // Convert the pose of the virtual object from camera coordinates into world coordinates
    rs2_pose object_pose_in_world = pose_multiply(device_pose_in_world, object_pose_in_device);
    return object_pose_in_world;
}

rs2_pose pose_inverse(const rs2_pose &p)
{
    rs2_pose i;
    i.rotation = quaternion_conjugate(p.rotation);
    i.translation = vector_negate(quaternion_rotate_vector(i.rotation, p.translation));
    return i;
}

rs2_pose pose_multiply(const rs2_pose &ref2_in_ref1, const rs2_pose &ref3_in_ref2)
{
    rs2_pose ref3_in_ref1;
    ref3_in_ref1.rotation = quaternion_multiply(ref2_in_ref1.rotation, ref3_in_ref2.rotation);
    ref3_in_ref1.translation = vector_addition(quaternion_rotate_vector(ref2_in_ref1.rotation, ref3_in_ref2.translation), ref2_in_ref1.translation);
    return ref3_in_ref1;
}

rs2_vector pose_transform_point(const rs2_pose &pose, const rs2_vector &p)
{
    return vector_addition(quaternion_rotate_vector(pose.rotation, p), pose.translation);
}

rs2_quaternion quaternion_multiply(const rs2_quaternion &a, const rs2_quaternion &b)
{
    return rs2_quaternion{
        a.x * b.w + a.w * b.x - a.z * b.y + a.y * b.z,
        a.y * b.w + a.z * b.x + a.w * b.y - a.x * b.z,
        a.z * b.w - a.y * b.x + a.x * b.y + a.w * b.z,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    };
}

rs2_vector quaternion_rotate_vector(const rs2_quaternion &q, const rs2_vector &v)
{
    rs2_quaternion v_as_quaternion = {v.x, v.y, v.z, 0};
    rs2_quaternion rotated_v = quaternion_multiply(quaternion_multiply(q, v_as_quaternion), quaternion_conjugate(q));
    return rs2_vector{rotated_v.x, rotated_v.y, rotated_v.z};
}

rs2_quaternion quaternion_conjugate(const rs2_quaternion &q)
{
    return rs2_quaternion{-q.x, -q.y, -q.z, q.w};
}

rs2_vector vector_addition(const rs2_vector &a, const rs2_vector &b)
{
    return rs2_vector{a.x + b.x, a.y + b.y, a.z + b.z};
}

rs2_vector vector_negate(const rs2_vector &v)
{
    return rs2_vector{-v.x, -v.y, -v.z};
}

object convert_object_coordinates(const object &obj, const rs2_pose &object_pose)
{
    object transformed_obj;
    for (size_t i = 0; i < obj.size(); ++i)
    {
        rs2_vector v{obj[i].x(), obj[i].y(), obj[i].z()};
        v = pose_transform_point(object_pose, v);
        transformed_obj[i].f[0] = v.x;
        transformed_obj[i].f[1] = v.y;
        transformed_obj[i].f[2] = v.z;
    }
    return transformed_obj;
}

std::vector<point3d> raster_line(const point3d &a, const point3d &b, float step)
{
    rs2_vector direction = {b.x() - a.x(), b.y() - a.y(), b.z() - a.z()};
    float distance = std::sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
    int npoints = distance / step + 1;

    std::vector<point3d> points;
    if (npoints > 0)
    {
        direction.x = direction.x * step / distance;
        direction.y = direction.y * step / distance;
        direction.z = direction.z * step / distance;

        points.reserve(npoints);
        points.emplace_back(a);
        for (int i = 1; i < npoints; ++i)
        {
            points.emplace_back(a.x() + direction.x * i,
                                a.y() + direction.y * i,
                                a.z() + direction.z * i);
        }
    }
    return points;
}

void render_line(const std::vector<pixel> &line, int color_code)
{
    if (!line.empty())
    {
        GLfloat current_color[4];
        glGetFloatv(GL_CURRENT_COLOR, current_color);

        glLineWidth(5);
        glColor3f(color_code == 1 ? 1.f : 0.f,
                  color_code == 2 ? 1.f : 0.f,
                  color_code == 3 ? 1.f : 0.f);

        glBegin(GL_LINE_STRIP);
        for (auto &pixel : line)
        {
            glVertex3f(pixel.x(), pixel.y(), 0.f);
        }
        glEnd();

        glColor4fv(current_color);
    }
}

void render_text(int win_height, const std::string &text)
{
    GLfloat current_color[4];
    glGetFloatv(GL_CURRENT_COLOR, current_color);
    glColor3f(0, 0.5, 1);
    glScalef(2, 2, 2);
    draw_text(15, (win_height - 10) / 2, text.c_str());
    glScalef(1, 1, 1);
    glColor4fv(current_color);
}
