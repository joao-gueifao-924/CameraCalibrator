#pragma once
#include <vector>
#include <opencv2/core.hpp>


namespace pinhole_camera_calibration
{

	typedef std::vector<cv::Point2f> PointConstellation;

	class calibration_tools // static
	{
	public:
		// Flip a constellation of points inside an image horizontally, given image width.
		static std::vector<cv::Point2f> flip_horiontally(int image_width, const PointConstellation& points);

		// Compute Intersection-Over-Union between the convex hulls of two constellations of points A and B.
		static double constellation_IoU(const PointConstellation& constellation_a, const PointConstellation& constellation_b);

		// Compute Intersection-Over-Union between two convex polygons A and B.
		static double convex_polygons_IoU(const PointConstellation& polygon_a, const PointConstellation& polygon_b);
	};


	class camera_calibration
	{
	public:
		camera_calibration(int total_colors = 32);

		// only adds constellation to store if is_different_enough(constellation) is true
		void add_constellation(const PointConstellation& constellation);

		// checks if constellation is different enough from all other constellations already stored
		bool is_different_enough(const PointConstellation& constellation);

		void paint_calibration_footprint(cv::Mat image_bgr);
	private:
		void generate_colormap_(int total_colors = 32);

		std::vector< std::pair<PointConstellation, PointConstellation> > constellations_and_hulls_;
		std::vector<cv::Vec3b> colormap_;
	};
}
