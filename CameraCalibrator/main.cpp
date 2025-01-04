#include <filesystem>
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
static const cv::Size CHESSBOARD_SIZE{ 7,10 };
//static constexpr double SQUARE_SIDE_LENGTH_mm{ 172.5 / 11 }; // 172.5 mm per 11 squares
static constexpr double SQUARE_SIDE_LENGTH_mm{ 1.0 }; // 172.5 mm per 11 squares
static constexpr int INPUT_VIDEO_FRAME_WIDTH{ 1280 };
static constexpr int INPUT_VIDEO_FRAME_HEIGHT{ 720 };
static constexpr int MIN_SECONDS_HOLD_TO_CHECK_SIMILARITY{ 1 }; // total seconds the user must hold the pattern in place until a check for similarity of pattern pose store is made and feedback message is shown
static constexpr int MIN_SECONDS_HOLD_TO_ADD{ 3 }; // total seconds the user must hold the pattern in place until a snapshot is taken and detected corners stored
static constexpr double MIN_PATTERN_HOLD_IOU{ 0.90 };
static constexpr int MIN_CALIBRATION_CONSTELLATIONS{ 10 };
static const std::filesystem::path IMAGE_OUTPUT_PATH{ R"(C:\Users\joaog\temp_program_output\CameraCalibrator)" };

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

    cout << "OpenCV version: " << CV_VERSION << endl;

    cv::namedWindow(INPUT_IMAGE_WINDOW_TITLE, cv::WINDOW_KEEPRATIO);

    VideoCapture cap{ 0 };
    Mat frame_bgr, frame_gray;

    cap.set(CAP_PROP_FRAME_WIDTH, INPUT_VIDEO_FRAME_WIDTH);
    cap.set(CAP_PROP_FRAME_HEIGHT, INPUT_VIDEO_FRAME_HEIGHT);

    vector<Point2f> corners, anchor_corners, flipped_corners;
    double corners_iou = 0;
    time_point frame_new_pose_start_time = now_time();
    double total_seconds_held = 0;

    bool pattern_is_being_held = false;
    int total_pictures_saved = 0;

    camera_calibration this_calibration(SQUARE_SIDE_LENGTH_mm, CHESSBOARD_SIZE, { INPUT_VIDEO_FRAME_WIDTH, INPUT_VIDEO_FRAME_HEIGHT });


    std::cout << "Current Directory: " << std::filesystem::current_path() << std::endl;
    std::filesystem::create_directories(IMAGE_OUTPUT_PATH);

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

            corners_iou = calibration_tools::constellation_IoU(corners, anchor_corners);
            pattern_is_being_held = (corners_iou > MIN_PATTERN_HOLD_IOU);
        }

        if (!pattern_is_being_held)
        {
            frame_new_pose_start_time = now_time();
            anchor_corners = corners;
        }

        total_seconds_held = double(time_delta_ms(frame_new_pose_start_time) / 1000.0);

        if (total_seconds_held > MIN_SECONDS_HOLD_TO_CHECK_SIMILARITY && !this_calibration.is_different_enough(corners))
        {
            cv::putText(presentation_frame, "Hold pattern somewhere else", {100, 20}, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, {255, 225, 0});
        }

        if (total_seconds_held > MIN_SECONDS_HOLD_TO_ADD)
        {
            if (this_calibration.add_constellation(corners))
            {
                // Save the frame to file in the format image_0000.png, image_0001.png, etc.
                std::ostringstream filename;
                filename << "image_" << std::setw(4) << std::setfill('0') << total_pictures_saved++ << ".png";
                auto full_path = IMAGE_OUTPUT_PATH / filename.str();
                cv::imwrite(full_path.string(), frame_bgr);
            }
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


        

        if (this_calibration.total_constellations() < MIN_CALIBRATION_CONSTELLATIONS)
        {
            cv::putText(presentation_frame, "Keep registering more patterns", {100, 50}, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, {255, 225, 0});
        }
        else
        {
            this_calibration.fit_model();
        }

        cv::imshow(INPUT_IMAGE_WINDOW_TITLE, presentation_frame);
        int pressed_key = cv::waitKey(1);

        if (pressed_key == ESCAPE_KEY)
        {
            return;
        }
    }

    return;
}

int learn_opencv_calibration_example();


int main(int argc, char** argv)
{
    main_loop();
    //learn_opencv_calibration_example();
    return EXIT_SUCCESS;
}


// Defining the dimensions of checkerboard
int CHECKERBOARD[2]{ 7,10 };

int learn_opencv_calibration_example()
{
    // Creating vector to store vectors of 3D points for each checkerboard image
    std::vector<std::vector<cv::Point3f> > objpoints;

    // Creating vector to store vectors of 2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f> > imgpoints;

    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;
    for (int i{ 0 }; i < CHECKERBOARD[1]; i++)
    {
        for (int j{ 0 }; j < CHECKERBOARD[0]; j++)
            objp.push_back(cv::Point3f(j, i, 0));
    }

    // Extracting path of individual image stored in a given directory
    std::vector<cv::String> images;
    // Path of the folder containing checkerboard images
    //std::string path = "./images/*.jpg";
    std::string path = R"(C:\Users\joaog\temp_program_output\CameraCalibrator\*.png)";

    cv::glob(path, images);

    cv::Mat frame, gray;
    // vector to store the pixel coordinates of detected checker board corners 
    std::vector<cv::Point2f> corner_pts;
    bool success;

    // Looping over all the images in the directory
    for (int i{ 0 }; i < images.size(); i++)
    {
        frame = cv::imread(images[i]);
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Finding checker board corners
        // If desired number of corners are found in the image then success = true  
        success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

        /*
         * If desired number of corner are detected,
         * we refine the pixel coordinates and display
         * them on the images of checker board
        */
        if (success)
        {
            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

            // Displaying the detected corner points on the checker board
            cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

            objpoints.push_back(objp);
            imgpoints.push_back(corner_pts);
        }

        cv::imshow("Image", frame);
        cv::waitKey(1);
    }

    cv::destroyAllWindows();

    cv::Mat cameraMatrix, distCoeffs, R, T;

    /*
     * Performing camera calibration by
     * passing the value of known 3D points (objpoints)
     * and corresponding pixel coordinates of the
     * detected corners (imgpoints)
    */

    const int calibration_flags = cv::CALIB_ZERO_TANGENT_DIST;

    double rms_reproj_error = cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows, gray.cols), cameraMatrix, distCoeffs, R, T, calibration_flags);

    std::cout << "rms_reproj_error : " << rms_reproj_error << std::endl;
    std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
    std::cout << "distCoeffs : " << distCoeffs << std::endl;
    std::cout << "Rotation vector : " << R << std::endl;
    std::cout << "Translation vector : " << T << std::endl;

    return 0;
}
