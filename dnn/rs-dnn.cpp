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
    /// Caffe ���� �д´�.
    Net net = readNetFromCaffe("MobileNetSSD_deploy.prototxt", 
                               "MobileNetSSD_deploy.caffemodel");

    // Start streaming from Intel RealSense Camera
    pipeline pipe;
    auto config = pipe.start();
    auto profile = config.get_stream(RS2_STREAM_COLOR)
                         .as<video_stream_profile>();
    rs2::align align_to(RS2_STREAM_COLOR);

    /// �̹����� ������ ���¿� ���� �� ũ�Ⱑ �޶������� ���μ��� ������ ���� ������.
    /// @todo ���������� ��Ȯ�� ���������� �ϴ��� �𸣰���
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

    ///������ �������� ������� cropSize ��ŭ ���� 2�� ���� ��ġ���� cropSize��ŭ�� Rect�� ���Ѵ�.
    Rect crop(Point((profile.width() - cropSize.width) / 2,
                    (profile.height() - cropSize.height) / 2),
              cropSize);

    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);

    /// windows â�� ������,
    while (getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
        ///���� �������� ��ٸ���
        // Wait for the next set of frames
        auto data = pipe.wait_for_frames();
        /// �������� ���������� aligned�Ǿ������� Ȯ���ϰ�
        // Make sure the frames are spatially aligned
        data = align_to.process(data);

        /// color/depth frame�� ���
        auto color_frame = data.get_color_frame();
        auto depth_frame = data.get_depth_frame();

        /// �� depth �̹����� �޾�����, Į���̹����� ������ �ʾҴٸ�
        /// continue
        // If we only received new depth frame, 
        // but the color did not update, continue
        static int last_frame_number = 0;
        if (color_frame.get_frame_number() == last_frame_number) continue;
        last_frame_number = color_frame.get_frame_number();

        ///RealSense�� frame�� openCV mat �������� ��ȯ�Ѵ�.
        // Convert RealSense frame to OpenCV matrix:
        auto color_mat = frame_to_mat(color_frame);
        auto depth_mat = depth_frame_to_meters(depth_frame);

        /// blob(�̹������� ����� ����)�� ��� ���� color �̹��� ��� dnn�� �̿���
        Mat inputBlob = blobFromImage(color_mat, inScaleFactor,
                                      Size(inWidth, inHeight), meanVal, false); //Convert Mat to batch of images

        /// �о��� cafe ���� ��ǲ�� �����Ѵ�.
        net.setInput(inputBlob, "data"); //set the network input

        /// ���� ���͸� ���ؼ� ����ؼ� ��������.
        Mat detection = net.forward("detection_out"); //compute output

        /// detection�� ������� ���� Mat�� �����.
        /// Mat(int rows, int cols, int type, const Scalar& s);
        Mat detectionMat(detection.size[2], detection.size[3], CV_32F,
                         detection.ptr<float>());

        /// Color�� depth frame���� object�� ��ġ�� �´� crop�� �ڸ���.
        // Crop both color and depth frames
        color_mat = color_mat(crop);
        depth_mat = depth_mat(crop);

        /// confidence Threshold?
        float confidenceThreshold = 0.8f;
        for(int i = 0; i < detectionMat.rows; i++)
        {

            /// ����� �� ���� 
            float confidence = detectionMat.at<float>(i, 2);

            if(confidence > confidenceThreshold)
            {
                /// ����� object�� �� class�� Ȯ�� ��
                size_t objectClass = (size_t)(detectionMat.at<float>(i, 1));

                /// ����� object�� edge�� ��ǥ�� ���
                int xLeftBottom = static_cast<int>(detectionMat.at<float>(i, 3) * color_mat.cols);
                int yLeftBottom = static_cast<int>(detectionMat.at<float>(i, 4) * color_mat.rows);
                int xRightTop = static_cast<int>(detectionMat.at<float>(i, 5) * color_mat.cols);
                int yRightTop = static_cast<int>(detectionMat.at<float>(i, 6) * color_mat.rows);

                /// ����� object�� ��ǥ�� �Ѱ��� Rect�� ��Ÿ����
                Rect object((int)xLeftBottom, (int)yLeftBottom,
                            (int)(xRightTop - xLeftBottom),
                            (int)(yRightTop - yLeftBottom));

                object = object  & Rect(0, 0, depth_mat.cols, depth_mat.rows);

                /// ���� ������ ��� depth�� ����Ͽ�
                /// �̰��� ���� ��ü�� depth�� ���ϱ� ���� �⺻���� ����̴�.
                /// ������ �̰��� ��� ���� �����͸� �Ϲ������� ������ �����ֱ� ���ؼ� 
                /// ����Ͽ���.
                // Calculate mean depth inside the detection region
                // This is a very naive way to estimate objects depth
                // but it is intended to demonstrate how one might 
                // use depth data in general
                Scalar m = mean(depth_mat(object));

                /// �Ÿ��� ��� ����ϰ� �ø��� ���ؼ� stream�� �����
                std::ostringstream ss;
                ss << classNames[objectClass] << " ";
                ss << std::setprecision(2) << m[0] << " meters away";
                String conf(ss.str());

                /// �׸� ���� ǥ�����ֱ�
                rectangle(color_mat, object, Scalar(0, 255, 0));
                int baseLine = 0;
                /// ���� �޾Ƽ� ǥ��
                Size labelSize = getTextSize(ss.str(), FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

                /// object�� ��ǥ�� �� ǥ��
                auto center = (object.br() + object.tl())*0.5;
                center.x = center.x - labelSize.width / 2;

                /// Rectangle �׷��ֱ�
                rectangle(color_mat, Rect(Point(center.x, center.y - labelSize.height),
                    Size(labelSize.width, labelSize.height + baseLine)),
                    Scalar(255, 255, 255), FILLED);
                /// Text �׷��ֱ�
                putText(color_mat, ss.str(), center,
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0));
            }
        }
        /// �̹��� ����
        imshow(window_name, color_mat);

        /// Ű�Է� ������ ����
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
