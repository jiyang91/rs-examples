// This example is derived from the ssd_mobilenet_object_detection opencv demo
// and adapted to be used with Intel RealSense Cameras
// Please see https://github.com/opencv/opencv/blob/master/LICENSE

#include <opencv2/dnn.hpp>
#include <librealsense2/rs.hpp>
#include "../cv-helpers.hpp"

const size_t inWidth      = 300;
const size_t inHeight     = 300;
const float WHRatio       = inWidth / (float)inHeight;
const float inScaleFactor = 0.007843f;
const float meanVal       = 127.5;
const char* classNames[]  = {"background",
                             "aeroplane", "bicycle", "bird", "boat",
                             "bottle", "bus", "car", "cat", "chair",
                             "cow", "diningtable", "dog", "horse",
                             "motorbike", "person", "pottedplant",
                             "sheep", "sofa", "train", "tvmonitor"};

int main(int argc, char** argv) try
{
    using namespace cv;
    using namespace cv::dnn;
    using namespace rs2;
    /// Caffe 모델을 읽는다.
    Net net = readNetFromCaffe("MobileNetSSD_deploy.prototxt", 
                               "MobileNetSSD_deploy.caffemodel");

    // Start streaming from Intel RealSense Camera
    pipeline pipe;
    auto config = pipe.start();
    auto profile = config.get_stream(RS2_STREAM_COLOR)
                         .as<video_stream_profile>();
    rs2::align align_to(RS2_STREAM_COLOR);

    /// 이미지에 잡히는 형태에 따라서 그 크기가 달라짐으로 가로세로 비율에 따라서 나눈다.
    /// @todo 프로파일이 정확히 무슨역할을 하는지 모르겠음
    Size cropSize;
    if (profile.width() / (float)profile.height() > WHRatio)
    {
        cropSize = Size(static_cast<int>(profile.height() * WHRatio),
                        profile.height());
    }
    else
    {
        cropSize = Size(profile.width(),
                        static_cast<int>(profile.width() / WHRatio));
    }

    ///정해진 프로파일 사이즈에서 cropSize 만큼 뺴고 2로 나눈 위치에서 cropSize만큼의 Rect를 구한다.
    Rect crop(Point((profile.width() - cropSize.width) / 2,
                    (profile.height() - cropSize.height) / 2),
              cropSize);

    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);

    /// windows 창이 있을때,
    while (getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
        ///다음 프레임을 기다리고
        // Wait for the next set of frames
        auto data = pipe.wait_for_frames();
        /// 프레임이 공간적으로 aligned되어있음을 확인하고
        // Make sure the frames are spatially aligned
        data = align_to.process(data);

        /// color/depth frame을 얻고
        auto color_frame = data.get_color_frame();
        auto depth_frame = data.get_depth_frame();

        /// 새 depth 이미지를 받았으나, 칼라이미지가 들어오지 않았다면
        /// continue
        // If we only received new depth frame, 
        // but the color did not update, continue
        static int last_frame_number = 0;
        if (color_frame.get_frame_number() == last_frame_number) continue;
        last_frame_number = color_frame.get_frame_number();

        ///RealSense의 frame을 openCV mat 형식으로 변환한다.
        // Convert RealSense frame to OpenCV matrix:
        auto color_mat = frame_to_mat(color_frame);
        auto depth_mat = depth_frame_to_meters(depth_frame);

        /// blob(이미지상의 덩어리를 뜻함)을 얻기 위한 color 이미지 사용 dnn을 이용함
        Mat inputBlob = blobFromImage(color_mat, inScaleFactor,
                                      Size(inWidth, inHeight), meanVal, false); //Convert Mat to batch of images

        /// 읽었던 cafe 모델의 인풋을 설정한다.
        net.setInput(inputBlob, "data"); //set the network input

        /// 모델을 필터를 통해서 계산해서 내보낸다.
        Mat detection = net.forward("detection_out"); //compute output

        /// detection의 사이즈와 같이 Mat을 만든다.
        /// Mat(int rows, int cols, int type, const Scalar& s);
        Mat detectionMat(detection.size[2], detection.size[3], CV_32F,
                         detection.ptr<float>());

        /// Color와 depth frame에서 object의 위치에 맞는 crop을 자른다.
        // Crop both color and depth frames
        color_mat = color_mat(crop);
        depth_mat = depth_mat(crop);

        /// confidence Threshold?
        float confidenceThreshold = 0.8f;
        for(int i = 0; i < detectionMat.rows; i++)
        {

            /// 검출된 것 각각 
            float confidence = detectionMat.at<float>(i, 2);

            if(confidence > confidenceThreshold)
            {
                /// 검출된 object의 각 class의 확률 맵
                size_t objectClass = (size_t)(detectionMat.at<float>(i, 1));

                /// 검출된 object의 edge의 좌표를 얻기
                int xLeftBottom = static_cast<int>(detectionMat.at<float>(i, 3) * color_mat.cols);
                int yLeftBottom = static_cast<int>(detectionMat.at<float>(i, 4) * color_mat.rows);
                int xRightTop = static_cast<int>(detectionMat.at<float>(i, 5) * color_mat.cols);
                int yRightTop = static_cast<int>(detectionMat.at<float>(i, 6) * color_mat.rows);

                /// 검출된 object의 좌표를 한개의 Rect에 나타내기
                Rect object((int)xLeftBottom, (int)yLeftBottom,
                            (int)(xRightTop - xLeftBottom),
                            (int)(yRightTop - yLeftBottom));

                object = object  & Rect(0, 0, depth_mat.cols, depth_mat.rows);

                /// 검출 영역의 평균 depth를 계산하여
                /// 이것은 아주 물체의 depth를 평가하기 위한 기본적인 방법이다.
                /// 하지만 이것은 어떻게 깊이 데이터를 일반적으로 쓰는지 보여주기 위해서 
                /// 사용하였다.
                // Calculate mean depth inside the detection region
                // This is a very naive way to estimate objects depth
                // but it is intended to demonstrate how one might 
                // use depth data in general
                Scalar m = mean(depth_mat(object));

                /// 거리를 계속 기록하고 올리기 위해서 stream을 사용함
                std::ostringstream ss;
                ss << classNames[objectClass] << " ";
                ss << std::setprecision(2) << m[0] << " meters away";
                String conf(ss.str());

                /// 네모 만들어서 표시해주기
                rectangle(color_mat, object, Scalar(0, 255, 0));
                int baseLine = 0;
                /// 라벨을 달아서 표기
                Size labelSize = getTextSize(ss.str(), FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

                /// object의 좌표를 얻어서 표기
                auto center = (object.br() + object.tl())*0.5;
                center.x = center.x - labelSize.width / 2;

                /// Rectangle 그려넣기
                rectangle(color_mat, Rect(Point(center.x, center.y - labelSize.height),
                    Size(labelSize.width, labelSize.height + baseLine)),
                    Scalar(255, 255, 255), FILLED);
                /// Text 그려넣기
                putText(color_mat, ss.str(), center,
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0));
            }
        }
        /// 이미지 띄우기
        imshow(window_name, color_mat);

        /// 키입력 있으면 끄기
        if (waitKey(1) >= 0) break;
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
