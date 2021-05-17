// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>
#include <mutex>
#include "example.hpp"          // Include short list of convenience functions for rendering    /// 짧은 랜더링을 위한 함수들 리스트를 포함한다.
#include <cstring>

struct short3
{
    uint16_t x, y, z;
};

#include "d435.h"

void draw_axes()
{
    glLineWidth(2);
    glBegin(GL_LINES);
    /// x,y,z 축을 그린다.
    // Draw x, y, z axes
    glColor3f(1, 0, 0); glVertex3f(0, 0, 0);  glVertex3f(-1, 0, 0);
    glColor3f(0, 1, 0); glVertex3f(0, 0, 0);  glVertex3f(0, -1, 0);
    glColor3f(0, 0, 1); glVertex3f(0, 0, 0);  glVertex3f(0, 0, 1);
    glEnd();

    glLineWidth(1);
}

void draw_floor()
{
    glBegin(GL_LINES);
    glColor4f(0.4f, 0.4f, 0.4f, 1.f);
    /// "floor" 그리드를 랜더링 한다.
    // Render "floor" grid
    /// 9번 특정 크기로 그리는 듯?
    for (int i = 0; i <= 8; i++)
    {
        glVertex3i(i - 4, 1, 0);
        glVertex3i(i - 4, 1, 8);
        glVertex3i(-4, 1, i);
        glVertex3i(4, 1, i);
    }
    glEnd();
}

void render_scene(glfw_state app_state)
{
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glColor3f(1.0, 1.0, 1.0);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, 4.0 / 3.0, 1, 40);

    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);

    glLoadIdentity();
    gluLookAt(1, 0, 5, 1, 0, 0, 0, -1, 0);

    glTranslatef(0, 0, +0.5f + app_state.offset_y*0.05f);
    glRotated(app_state.pitch, -1, 0, 0);
    glRotated(app_state.yaw, 0, 1, 0);
    draw_floor();
}

class camera_renderer
{
    std::vector<float3> positions, normals;
    std::vector<short3> indexes;
public:
    /// 카메라 그리는데 필요한 데이터 랜더러 초기화
    // Initialize renderer with data needed to draw the camera
    camera_renderer()
    {
        uncompress_d435_obj(positions, normals, indexes);
    }
    /// 계산된 각도값을 input으로 넣고 그에 따라 3D카메라 모델을 회전시킨다.
    // Takes the calculated angle as input and rotates the 3D camera model accordignly
    void render_camera(float3 theta)
    {

        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE);

        glPushMatrix();
        /// theta를 degree로 변환하여, 회전을 설정한다.
        // Set the rotation, converting theta to degrees
        glRotatef(theta.x * 180 / PI, 0, 0, -1);
        glRotatef(theta.y * 180 / PI, 0, -1, 0);
        glRotatef((theta.z - PI / 2) * 180 / PI, -1, 0, 0);

        draw_axes();
        /// 카메라 drawing을 scale한다.
        // Scale camera drawing
        glScalef(0.035, 0.035, 0.035);

        glBegin(GL_TRIANGLES);
        /// 카메라를 그린다.
        // Draw the camera
        for (auto& i : indexes)
        {
            glVertex3fv(&positions[i.x].x);
            glVertex3fv(&positions[i.y].x);
            glVertex3fv(&positions[i.z].x);
            glColor4f(0.05f, 0.05f, 0.05f, 0.3f);
        }
        glEnd();

        glPopMatrix();

        glDisable(GL_BLEND);
        glFlush();
    }

};

class rotation_estimator
{
    /// theta는 카메라의 x,y 그리고 z components의 회전 각도이다.
    // theta is the angle of camera rotation in x, y and z components
    float3 theta;
    std::mutex theta_mtx;
    /// alpha는 theta를 계산하는 gyro와 accelerometer의 부분을 나타낸다.
    /// 높은 알파는 gyro 값에 더 weight를 준다.
    /// 하지만 높은 값은 drift를 일으킬수 있다.
    /// 낮은 값은 acc에 더 가중치를 주는데,
    /// 이는 외란에 더 민감할 수 있다.
    /** alpha indicates the part that gyro and accelerometer take in computation of theta;
    * higher alpha gives more weight to gyro, 
    * but too high values cause drift; 
    * lower alpha gives more weight to accelerometer,
    * which is more sensitive to disturbances 
    */
    float alpha = 0.98;
    bool firstGyro = true;
    bool firstAccel = true;
    /// 이전 gyro 프레임의 도착 시간을 갖고있는다.
    // Keeps the arrival time of previous gyro frame
    double last_ts_gyro = 0;
public:
    /// Motion의 각도의 변화를 gyro 데이터를 기반으로 계산하는 함수 
    // Function to calculate the change in angle of motion based on data from gyro
    void process_gyro(rs2_vector gyro_data, double ts)
    {
        /// 첫 iteration에서, 카메라의 초기 위치를 세팅하기 위해서 accelerometer의 데이터만 사용한다
        if (firstGyro) // On the first iteration, use only data from accelerometer to set the camera's initial position
        {
            firstGyro = false;
            last_ts_gyro = ts;
            return;
        }
        /// 각도의 변화를 갖고, 값은 gyro로부터 계산되었다.
        // Holds the change in angle, as calculated from gyro
        float3 gyro_angle;
        /// gyro_angle을 gyro 데이터로부터 초기화 한다.
        // Initialize gyro_angle with data from gyro
        gyro_angle.x = gyro_data.x; // Pitch
        gyro_angle.y = gyro_data.y; // Yaw
        gyro_angle.z = gyro_data.z; // Roll

        /// 이전과 현재 gyro frame들의 도착 시간들의 차이값을 계산한다.
        // Compute the difference between arrival times of previous and current gyro frames
        double dt_gyro = (ts - last_ts_gyro) / 1000.0;
        last_ts_gyro = ts;

        /// 각도변화는 (gyro 측정값) * (이전 측정 시간으로부터 지난시간) 이다.
        // Change in angle equals gyro measures * time passed since last measurement
        gyro_angle = gyro_angle * dt_gyro;

        /// 계산된 각도의 변화를 현재 각도에 적용한다.
        // Apply the calculated change of angle to the current angle (theta)
        std::lock_guard<std::mutex> lock(theta_mtx);
        theta.add(-gyro_angle.z, -gyro_angle.y, gyro_angle.x);
    }

    void process_accel(rs2_vector accel_data)
    {
        // Holds the angle as calculated from accelerometer data
        float3 accel_angle;

        // Calculate rotation angle from accelerometer data
        accel_angle.z = atan2(accel_data.y, accel_data.z);
        accel_angle.x = atan2(accel_data.x, sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z));

        // If it is the first iteration, set initial pose of camera according to accelerometer data (note the different handling for Y axis)
        std::lock_guard<std::mutex> lock(theta_mtx);
        if (firstAccel)
        {
            firstAccel = false;
            theta = accel_angle;
            // Since we can't infer the angle around Y axis using accelerometer data, we'll use PI as a convetion for the initial pose
            theta.y = PI;
        }
        else
        {
            /* 
            Apply Complementary Filter:
                - high-pass filter = theta * alpha:  allows short-duration signals to pass through while filtering out signals
                  that are steady over time, is used to cancel out drift.
                - low-pass filter = accel * (1- alpha): lets through long term changes, filtering out short term fluctuations 
            */
            theta.x = theta.x * alpha + accel_angle.x * (1 - alpha);
            theta.z = theta.z * alpha + accel_angle.z * (1 - alpha);
        }
    }
    
    // Returns the current rotation angle
    float3 get_theta()
    {
        std::lock_guard<std::mutex> lock(theta_mtx);
        return theta;
    }
};


bool check_imu_is_supported()
{
    bool found_gyro = false;
    bool found_accel = false;
    rs2::context ctx;
    for (auto dev : ctx.query_devices())
    {
        // The same device should support gyro and accel
        found_gyro = false;
        found_accel = false;
        for (auto sensor : dev.query_sensors())
        {
            for (auto profile : sensor.get_stream_profiles())
            {
                if (profile.stream_type() == RS2_STREAM_GYRO)
                    found_gyro = true;

                if (profile.stream_type() == RS2_STREAM_ACCEL)
                    found_accel = true;
            }
        }
        if (found_gyro && found_accel)
            break;
    }
    return found_gyro && found_accel;
}

int main(int argc, char * argv[]) try
{
    // Before running the example, check that a device supporting IMU is connected
    if (!check_imu_is_supported())
    {
        std::cerr << "Device supporting IMU (D435i) not found";
        return EXIT_FAILURE;
    }

    // Initialize window for rendering
    window app(1280, 720, "RealSense Motion Example");
    // Construct an object to manage view state
    glfw_state app_state(0.0, 0.0);
    // Register callbacks to allow manipulation of the view state
    register_glfw_callbacks(app, app_state);

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // Add streams of gyro and accelerometer to configuration
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

    // Declare object for rendering camera motion
    camera_renderer camera;
    // Declare object that handles camera pose calculations
    rotation_estimator algo;

    // Start streaming with the given configuration;
    // Note that since we only allow IMU streams, only single frames are produced
    auto profile = pipe.start(cfg, [&](rs2::frame frame)
    {
        // Cast the frame that arrived to motion frame
        auto motion = frame.as<rs2::motion_frame>();
        // If casting succeeded and the arrived frame is from gyro stream
        if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
        {
            // Get the timestamp of the current frame
            double ts = motion.get_timestamp();
            // Get gyro measures
            rs2_vector gyro_data = motion.get_motion_data();
            // Call function that computes the angle of motion based on the retrieved measures
            algo.process_gyro(gyro_data, ts);
        }
        // If casting succeeded and the arrived frame is from accelerometer stream
        if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
        {
            // Get accelerometer measures
            rs2_vector accel_data = motion.get_motion_data();
            // Call function that computes the angle of motion based on the retrieved measures
            algo.process_accel(accel_data);
        }
    });

    // Main loop
    while (app)
    {
        // Configure scene, draw floor, handle manipultation by the user etc.
        render_scene(app_state);
        // Draw the camera according to the computed theta
        camera.render_camera(algo.get_theta());
    }
    // Stop the pipeline
    pipe.stop();

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
