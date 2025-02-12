#include <filesystem>
#include <iostream>
#include <iomanip>
#include <chrono>
//#include "calibration_tools.h"
#include "camera_calibration.h"
#include <opencv2/highgui.hpp>

//using namespace pinhole_camera_calibration;
using namespace std;
using namespace cv;

static constexpr bool IMAGE_FOLDER_EXECUTION_MODE{ true };
static const cv::Size CHESSBOARD_PATTERN_SIZE{ 6, 10 };
static constexpr double CHESSBOARD_PATTERN_SQUARE_SIDE_LENGTH_mm{ 14.88 };
static const cv::Size INPUT_VIDEO_FRAME_SIZE{ 1280, 720 };
static const std::filesystem::path IMAGE_OUTPUT_PATH{ R"(C:\Users\joaog\temp_program_output\CameraCalibrator)" };

static std::string INPUT_IMAGE_WINDOW_TITLE{ "Input image & Detections" };
static constexpr int ESCAPE_KEY = 27;


bool single_image_iteration(cv::Mat image_bgr, camera_calibration& this_calibration)
{
    if (image_bgr.size() != INPUT_VIDEO_FRAME_SIZE)
    {
        throw std::runtime_error("Input video resolution different than requested");
    }
    if (image_bgr.empty())
    {
        throw std::runtime_error("Empty video frame");
    }

    auto pattern_status = this_calibration.try_register(image_bgr);

    cv::Mat presentation_frame = this_calibration.render_feedback_image(true);
    cv::imshow(INPUT_IMAGE_WINDOW_TITLE, presentation_frame);
    int pressed_key = cv::waitKey(1);
    if (pressed_key == ESCAPE_KEY) return false;

    camera_calibration::fitting_status fitting_status{ camera_calibration::fitting_status::undefined };

    switch (pattern_status)
    {
    case camera_calibration::pattern_status::pattern_not_found:
        std::cout << "Pattern not found" << std::endl;
        break;
    case camera_calibration::pattern_status::pattern_too_similar:
        std::cout << "Pattern too similar" << std::endl;
        break;
    case camera_calibration::pattern_status::pattern_not_held_long_enough:
        std::cout << "Pattern not held long enough" << std::endl;
        break;
    case camera_calibration::pattern_status::pattern_accepted:
        std::cout << "Pattern accepted" << std::endl;
        fitting_status = this_calibration.try_fit();

        if (fitting_status == camera_calibration::fitting_status::newly_fitted_camera_model)
        {
            //auto model = this_calibration.extract_model();
            //std::cout << model;
        }
        break;
    default:
        throw std::logic_error("Unexpected pattern_status value");
        break;
    }

    return true;
}

int image_folder_main()
{
    camera_calibration this_calibration(CHESSBOARD_PATTERN_SQUARE_SIDE_LENGTH_mm, CHESSBOARD_PATTERN_SIZE, INPUT_VIDEO_FRAME_SIZE, true);

    // Extracting path of individual image stored in a given directory
    std::vector<cv::String> images;

    // Path of the folder containing checkerboard images
    std::string path = R"(C:\Users\joaog\temp_program_output\CameraCalibrator\*.png)";
    cv::glob(path, images);

    cv::Mat frame_bgr;

    // Looping over all the images in the directory
    for (int i{ 0 }; i < images.size(); i++)
    {
        frame_bgr = cv::imread(images[i]);
        if (!single_image_iteration(frame_bgr, this_calibration))
        {
            break;
        };
    }

    return EXIT_SUCCESS;
}

int video_main()
{
    VideoCapture cap{ 0 };
    cap.set(CAP_PROP_FRAME_WIDTH, INPUT_VIDEO_FRAME_SIZE.width);
    cap.set(CAP_PROP_FRAME_HEIGHT, INPUT_VIDEO_FRAME_SIZE.height);

    camera_calibration this_calibration(CHESSBOARD_PATTERN_SQUARE_SIDE_LENGTH_mm, CHESSBOARD_PATTERN_SIZE, INPUT_VIDEO_FRAME_SIZE, false);
    
    Mat frame_bgr;

    while (true) 
    {
        cap >> frame_bgr;
        if (!single_image_iteration(frame_bgr, this_calibration))
        {
            break;
        };
    }

    if (!this_calibration.save_registered_images_to_folder(IMAGE_OUTPUT_PATH))
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}


int main(int argc, char** argv)
{
    namedWindow(INPUT_IMAGE_WINDOW_TITLE, WINDOW_KEEPRATIO);
    return (IMAGE_FOLDER_EXECUTION_MODE) ? image_folder_main() : video_main();
}