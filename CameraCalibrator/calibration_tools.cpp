#include "calibration_tools.h"

std::vector<cv::Point2f> calibration_tools::flip_horiontally(int image_width, std::vector<cv::Point2f> points)
{
    std::vector<cv::Point2f> output;
	output.reserve(points.size());

	for (size_t i = 0; i < points.size(); i++)
	{
		cv::Point2f flipped_point;
		flipped_point.x = image_width - points[i].x - 1;
		flipped_point.y = points[i].y;
		output.push_back(flipped_point);
	}

	return output;
}
