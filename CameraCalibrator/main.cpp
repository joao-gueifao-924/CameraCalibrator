#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/ccalib.hpp>
#include <opencv2/calib3d.hpp>

//static std::string IMAGE_PATH{ R"(C:\Users\joaog\OneDrive\Imagens\Camera Roll\IMG_20200524_134602.jpg)" };

static std::string INPUT_IMAGE{ "Input image" };

int main(int argc, char** argv)
{
	using namespace std;
	using namespace cv;

	cout << "Hello World!" << endl;
	cout << "OpenCV version: " << CV_VERSION << endl;
	//std::cout << "Trying to load image: " << IMAGE_PATH << std::endl;

	//cv::Mat image = cv::imread(IMAGE_PATH);
	cv::namedWindow(INPUT_IMAGE, cv::WINDOW_KEEPRATIO);
	//cv::imshow(INPUT_IMAGE, image);
	//cv::waitKey(0);

	VideoCapture cap{ 0 };
	Mat frame_bgr, frame_bgr_flipped, frame_gray_flipped;


	cap.set(CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(CAP_PROP_FRAME_HEIGHT, 720);

	// Remember that Fast Check erroneously fails with high distortions like fisheye.
	const int chessboard_flags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK;
	vector<Point2f> corners;
	const cv::Size window_search_size{ 11, 11 };
	const cv::Size chessboard_size{ 9,6 };

	while (true)
	{
		cap >> frame_bgr;

		cv::flip(frame_bgr, frame_bgr_flipped, 1);

		if (frame_bgr.empty())
		{
			throw std::runtime_error("Empty video frame.");
		}

		bool found = findChessboardCorners(frame_bgr_flipped, chessboard_size, corners, chessboard_flags);

		if (found)
		{
			cvtColor(frame_bgr_flipped, frame_gray_flipped, COLOR_BGR2GRAY);
			cornerSubPix(frame_gray_flipped, corners, window_search_size,
				Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.0001));

			drawChessboardCorners(frame_bgr_flipped, chessboard_size, corners, found);
		}

		cv::imshow(INPUT_IMAGE, frame_bgr_flipped);
		cv::waitKey(1);
	}

	return EXIT_SUCCESS;
}
