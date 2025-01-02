#include "calibration_tools.h"
#include <opencv2/imgproc.hpp>

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

double calibration_tools::constellation_IoU(const std::vector<cv::Point2f>& constellation_a, const std::vector<cv::Point2f>& constellation_b)
{
	if (constellation_a.size() < 4 || constellation_b.size() < 4)
	{
		return 0.0;
	}

	std::vector<cv::Point2f> hull1, hull2;
	cv::convexHull(constellation_a, hull1);
	cv::convexHull(constellation_b, hull2);

	// Compare hull properties (e.g., area)
	double areaHull1 = cv::contourArea(hull1);
	double areaHull2 = cv::contourArea(hull2);
	
	// Compute intersection polygon
	std::vector<cv::Point2f> intersectionPolygon;
	float intersectionArea = cv::intersectConvexConvex(hull1, hull2, intersectionPolygon);

	// Compute Intersection over Union
	double unionArea = areaHull1 + areaHull2 - intersectionArea;
	double iou = (unionArea > 0) ? (intersectionArea / unionArea) : 0.0;
	return iou;
}
