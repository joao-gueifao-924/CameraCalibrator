#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/ccalib.hpp>
#include <opencv2/calib3d.hpp>

#include "calibration_tools.h"


//static std::string IMAGE_PATH{ R"(C:\Users\joaog\OneDrive\Imagens\Camera Roll\IMG_20200524_134602.jpg)" };

static std::string INPUT_IMAGE_WINDOW_TITLE{ "Input image & Detections" };
static const cv::Size WINDOW_SEARCH_SIZE{ 11, 11 };
static const cv::Size CHESSBOARD_SIZE{ 10,7 };
static constexpr double SQUARE_SIDE_LENGTH{ 172.5 / 11 }; // 172.5 mm per 11 squares
static constexpr int INPUT_VIDEO_FRAME_WIDTH{ 1280 };
static constexpr int INPUT_VIDEO_FRAME_HEIGHT{ 720 };

// Remember that Fast Check erroneously fails with high distortions like fisheye.
static constexpr int chessboard_flags = cv::CALIB_CB_ADAPTIVE_THRESH
                                      | cv::CALIB_CB_NORMALIZE_IMAGE
                                      | cv::CALIB_CB_FAST_CHECK;

static constexpr int ESCAPE_KEY = 27;


static void main_loop()
{
    using namespace std;
    using namespace cv;

    cout << "Hello World!" << endl;
    cout << "OpenCV version: " << CV_VERSION << endl;

    cv::namedWindow(INPUT_IMAGE_WINDOW_TITLE, cv::WINDOW_KEEPRATIO);

    VideoCapture cap{ 0 };
    Mat frame_bgr, frame_bgr_flipped, frame_gray_flipped;


    cap.set(CAP_PROP_FRAME_WIDTH, INPUT_VIDEO_FRAME_WIDTH);
    cap.set(CAP_PROP_FRAME_HEIGHT, INPUT_VIDEO_FRAME_HEIGHT);

    vector<Point2f> corners, flipped_corners;


    while (true)
    {
        cap >> frame_bgr;

        if (frame_bgr.size() != cv::Size(INPUT_VIDEO_FRAME_WIDTH, INPUT_VIDEO_FRAME_HEIGHT))
        {
            throw std::runtime_error("Input video resolution different than requested");
        }

        cv::flip(frame_bgr, frame_bgr_flipped, 1);

        if (frame_bgr.empty())
        {
            throw std::runtime_error("Empty video frame");
        }

        bool found = findChessboardCorners(frame_bgr_flipped, CHESSBOARD_SIZE,
            flipped_corners, chessboard_flags);

        if (found)
        {
            cvtColor(frame_bgr_flipped, frame_gray_flipped, COLOR_BGR2GRAY);

            cornerSubPix(frame_gray_flipped, flipped_corners, WINDOW_SEARCH_SIZE,
                Size(-1, -1),
                TermCriteria(TermCriteria::EPS + TermCriteria::COUNT,
                    30, 0.0001));

            drawChessboardCorners(frame_bgr_flipped, CHESSBOARD_SIZE, flipped_corners, found);
            corners = calibration_tools::flip_horiontally(frame_bgr_flipped.cols, flipped_corners);
        }

        cv::imshow(INPUT_IMAGE_WINDOW_TITLE, frame_bgr_flipped);
        int pressed_key = cv::waitKey(1);

        if (pressed_key == ESCAPE_KEY)
        {
            return;
        }
    }

    return;
}


int main(int argc, char** argv)
{
    main_loop();
    return EXIT_SUCCESS;
}
