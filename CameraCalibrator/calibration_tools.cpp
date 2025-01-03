#include "calibration_tools.h"
#include <opencv2/imgproc.hpp>
#include <random>
#include <opencv2/calib3d.hpp>

using namespace pinhole_camera_calibration;

double CONSTELLATION_IOU_THRESHOLD{ 0.7 };

PointConstellation2D calibration_tools::flip_horiontally(int image_width, const PointConstellation2D& points)
{
	PointConstellation2D output;
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

double calibration_tools::constellation_IoU(const PointConstellation2D& constellation_a, const PointConstellation2D& constellation_b)
{
	if (constellation_a.size() < 4 || constellation_b.size() < 4)
	{
		return 0.0;
	}

	PointConstellation2D hull_a, hull_b;
	cv::convexHull(constellation_a, hull_a);
	cv::convexHull(constellation_b, hull_b);

	return convex_polygons_IoU(hull_a, hull_b);
}

double pinhole_camera_calibration::calibration_tools::convex_polygons_IoU(const PointConstellation2D& polygon_a, const PointConstellation2D& polygon_b)
{
	if (polygon_a.size() < 4 || polygon_b.size() < 4)
	{
		return 0.0;
	}

	double area_a = cv::contourArea(polygon_a);
	double area_b = cv::contourArea(polygon_b);

	// Compute intersection polygon
	PointConstellation2D intersection_polygon;
	float intersection_area = cv::intersectConvexConvex(polygon_a, polygon_b, intersection_polygon);

	// Compute Intersection over Union
	double union_area = area_a + area_b - intersection_area;
	double iou = (union_area > 0) ? (intersection_area / union_area) : 0.0;
	return iou;
}

PointConstellation3D pinhole_camera_calibration::calibration_tools::generate_chessboard_world_points(float square_side_length_mm, cv::Size pattern_size)
{
	PointConstellation3D output;

	for (int i = 0; i < pattern_size.width; i++)
	{
		for (int j = 0; j < pattern_size.height; j++)
		{
			output.emplace_back(i * square_side_length_mm, j * square_side_length_mm, 0.0f);
		}
	}

	return output;
}

pinhole_camera_calibration::camera_calibration::camera_calibration(float square_side_length_mm, cv::Size pattern_size, cv::Size image_size, int total_colors)
	: image_size_{image_size}
{
	typedef calibration_tools ct;
	generate_colormap_(total_colors);
	calibration_pattern_world_constellations_ = ct::generate_chessboard_world_points(square_side_length_mm, pattern_size);
}

void pinhole_camera_calibration::camera_calibration::add_constellation(const PointConstellation2D& constellation)
{
	if (!is_different_enough(constellation)) return;

	PointConstellation2D hull;
	cv::convexHull(constellation, hull);
	constellations_and_hulls_.push_back({ constellation, hull });
}

bool pinhole_camera_calibration::camera_calibration::is_different_enough(const PointConstellation2D& constellation)
{
	typedef calibration_tools ct;

	PointConstellation2D hull;
	cv::convexHull(constellation, hull);

	for (size_t i = 0; i < constellations_and_hulls_.size(); i++)
	{
		double iou = ct::convex_polygons_IoU(hull, constellations_and_hulls_[i].second);

		if (iou > CONSTELLATION_IOU_THRESHOLD)
		{
			return false;
		}
	}

	return true;
}

void pinhole_camera_calibration::camera_calibration::paint_calibration_footprint(cv::Mat image_bgr)
{
	std::mt19937 gen(42); // Fixed seed for reproducibility
	std::uniform_int_distribution<int> dist(0, colormap_.size()-1); // Range [0, total_colors-1]

	for (size_t i = 0; i < constellations_and_hulls_.size(); i++)
	{
		// pick random color from colormap
		int random_idx = dist(gen); // Generate a random index
		cv::Vec3b random_color = colormap_.at(random_idx);

		const PointConstellation2D& hull = constellations_and_hulls_[i].second;
		std::vector<cv::Point2i> hull_integers;
		hull_integers.reserve(hull.size());

		for (size_t j = 0; j < hull.size(); j++)
		{
			hull_integers.push_back(cv::Point2i{hull[j]});
		}

		//cv::fillConvexPoly(image_bgr, hull_integers, random_color);

		// Create a temporary overlay image
		cv::Mat overlay;
		image_bgr.copyTo(overlay);

		// Draw the polygon on the overlay image
		cv::fillConvexPoly(overlay, hull_integers, random_color);

		double alpha = 0.5;

		// Blend the overlay with the original image using alpha blending
		cv::addWeighted(overlay, alpha, image_bgr, 1.0 - alpha, 0.0, image_bgr);
	}
}

void pinhole_camera_calibration::camera_calibration::fit_model()
{
	// Perform camera calibration
	cv::Mat cameraMatrix, distCoeffs;
	std::vector<cv::Mat> rvecs, tvecs;
	
	std::vector<PointConstellation2D> image_points;
	std::vector<PointConstellation3D> object_world_points_copies;

	for (size_t i = 0; i < constellations_and_hulls_.size(); i++)
	{
		image_points.push_back(constellations_and_hulls_[i].first); // need to discard the constellation hulls for the cv::calibrateCamera call below
		object_world_points_copies.push_back(calibration_pattern_world_constellations_); // push several copies
	}

	double reprojectionError = cv::calibrateCamera(object_world_points_copies, image_points,
		image_size_, cameraMatrix,
		distCoeffs, rvecs, tvecs);

	// Print results
	std::cout << "Camera Matrix:\n" << cameraMatrix << "\n";
	std::cout << "Distortion Coefficients:\n" << distCoeffs << "\n";
	std::cout << "Reprojection Error: " << reprojectionError << "\n";

}

void pinhole_camera_calibration::camera_calibration::generate_colormap_(int total_colors)
{
	cv::Mat grayscale(1, 256, CV_8UC1); // Grayscale image
	for (int i = 0; i < 256; ++i)
		grayscale.at<uchar>(0, i) = i;

	cv::Mat colormap;
	cv::applyColorMap(grayscale, colormap, cv::COLORMAP_JET);

	// Extract distinct colors from the colormap
	for (int i = 0; i < colormap.cols; i += colormap.cols / total_colors)
	{
		cv::Vec3b color = colormap.at<cv::Vec3b>(0, i);
		colormap_.push_back(color);
	}
}
