#include <iostream>
#include <iomanip>
#include <chrono>
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
static constexpr int MIN_SECONDS_HOLD_TO_CHECK_SIMILARITY{ 1 }; // total seconds the user must hold the pattern in place until a check for similarity of pattern pose store is made and feedback message is shown
static constexpr int MIN_SECONDS_HOLD_TO_ADD{ 3 }; // total seconds the user must hold the pattern in place until a snapshot is taken and detected corners stored
static constexpr double MIN_PATTERN_HOLD_IOU{ 0.96 };

// Remember that Fast Check erroneously fails with high distortions like fisheye.
static constexpr int chessboard_flags = cv::CALIB_CB_ADAPTIVE_THRESH
                                      | cv::CALIB_CB_NORMALIZE_IMAGE
                                      | cv::CALIB_CB_FAST_CHECK;

static constexpr int ESCAPE_KEY = 27;

typedef std::chrono::steady_clock::time_point time_point;

std::chrono::steady_clock::time_point now_time()
{
    return std::chrono::high_resolution_clock::now();
}
long long time_delta_ms(std::chrono::steady_clock::time_point end, std::chrono::steady_clock::time_point start)
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
}
long long time_delta_ms(std::chrono::steady_clock::time_point start)
{
    return time_delta_ms(now_time(), start);
}


static void main_loop()
{
    using namespace pinhole_camera_calibration;
    using namespace std;
    using namespace cv;

    cout << "Hello World!" << endl;
    cout << "OpenCV version: " << CV_VERSION << endl;

    cv::namedWindow(INPUT_IMAGE_WINDOW_TITLE, cv::WINDOW_KEEPRATIO);

    VideoCapture cap{ 0 };
    Mat frame_bgr, frame_gray;

    cap.set(CAP_PROP_FRAME_WIDTH, INPUT_VIDEO_FRAME_WIDTH);
    cap.set(CAP_PROP_FRAME_HEIGHT, INPUT_VIDEO_FRAME_HEIGHT);

    vector<Point2f> corners, previous_corners, flipped_corners;
    double corners_iou = 0;
    time_point frame_new_pose_start_time = now_time();
    double total_seconds_held = 0;

    bool pattern_is_being_held = false;

    camera_calibration this_calibration;

    while (true)
    {
        cap >> frame_bgr;
        corners.clear();
        pattern_is_being_held = false;

        cv::Mat presentation_frame = frame_bgr.clone();
        this_calibration.paint_calibration_footprint(presentation_frame);

        // Mirror the webcam feed horizontally as it makes it easier to move the pattern around.
        cv::flip(presentation_frame, presentation_frame, 1);

        if (frame_bgr.size() != cv::Size(INPUT_VIDEO_FRAME_WIDTH, INPUT_VIDEO_FRAME_HEIGHT))
        {
            throw std::runtime_error("Input video resolution different than requested");
        }

        if (frame_bgr.empty())
        {
            throw std::runtime_error("Empty video frame");
        }

        bool found = findChessboardCorners(frame_bgr, CHESSBOARD_SIZE,
            corners, chessboard_flags);

        if (found)
        {
            cvtColor(frame_bgr, frame_gray, COLOR_BGR2GRAY);

            cornerSubPix(frame_gray, corners, WINDOW_SEARCH_SIZE,
                Size(-1, -1),
                TermCriteria(TermCriteria::EPS + TermCriteria::COUNT,
                    30, 0.0001));

            flipped_corners = calibration_tools::flip_horiontally(frame_bgr.cols, corners);
            drawChessboardCorners(presentation_frame, CHESSBOARD_SIZE, flipped_corners, found);

            corners_iou = calibration_tools::constellation_IoU(corners, previous_corners);
            pattern_is_being_held = (corners_iou > MIN_PATTERN_HOLD_IOU);
        }

        if (!pattern_is_being_held)
        {
            frame_new_pose_start_time = now_time();
        }

        total_seconds_held = double(time_delta_ms(frame_new_pose_start_time) / 1000.0);

        if (total_seconds_held > MIN_SECONDS_HOLD_TO_CHECK_SIMILARITY && !this_calibration.is_different_enough(corners))
        {
            cv::putText(presentation_frame, "Hold pattern somewhere else", {100, 20}, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, {255, 225, 0});
        }

        if (total_seconds_held > MIN_SECONDS_HOLD_TO_ADD)
        {
            this_calibration.add_constellation(corners);
            frame_new_pose_start_time = now_time();
        }
        
        // Create a string stream
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3) << std::setw(6) << std::setfill('0');

        // Set fixed-point format, precision, and width with leading zeros
        //oss << "Corners IoU: "  << corners_iou;
        //cv::putText(frame_bgr_flipped, oss.str(), { 100, 100 }, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, { 255, 225, 0 });
        
        oss.str("");       // Clear the buffer
        oss.clear();       // Reset stream state
        oss << "pattern held for: "<< total_seconds_held << " seconds";
        cv::putText(presentation_frame, oss.str(), { 100, 150 }, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, { 255, 225, 0 });


        cv::imshow(INPUT_IMAGE_WINDOW_TITLE, presentation_frame);
        int pressed_key = cv::waitKey(1);

        if (pressed_key == ESCAPE_KEY)
        {
            return;
        }

        previous_corners = corners;
    }

    return;
}


int main(int argc, char** argv)
{
    main_loop();
    return EXIT_SUCCESS;
}
