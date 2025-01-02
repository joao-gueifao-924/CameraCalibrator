#pragma once
#include <vector>
#include <opencv2/core.hpp>

class calibration_tools
{
public: 
	static std::vector<cv::Point2f> flip_horiontally(int image_width, std::vector<cv::Point2f> points);
};

