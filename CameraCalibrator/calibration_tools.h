#pragma once
#include <vector>
#include <opencv2/core.hpp>


namespace pinhole_camera_calibration
{

	typedef std::vector<cv::Point2f> PointConstellation2D;
	typedef std::vector<cv::Point3f> PointConstellation3D;

	class calibration_tools // static
	{
	public:
		// Flip a constellation of points inside an image horizontally, given image width.
		static std::vector<cv::Point2f> flip_horiontally(int image_width, const PointConstellation2D& points);

		// Compute Intersection-Over-Union between the convex hulls of two constellations of points A and B.
		static double constellation_IoU(const PointConstellation2D& constellation_a, const PointConstellation2D& constellation_b);

		// Compute Intersection-Over-Union between two convex polygons A and B.
		static double convex_polygons_IoU(const PointConstellation2D& polygon_a, const PointConstellation2D& polygon_b);

		static PointConstellation3D generate_chessboard_world_points(float square_side_length_mm, cv::Size pattern_size);
	};


	class camera_calibration
	{
	public:
		camera_calibration(float square_side_length_mm, cv::Size pattern_size, cv::Size image_size, int total_colors = 32);

		// only adds constellation to store if is_different_enough(constellation) is true
		void add_constellation(const PointConstellation2D& constellation);

		// checks if constellation is different enough from all other constellations already stored
		bool is_different_enough(const PointConstellation2D& constellation);

		void paint_calibration_footprint(cv::Mat image_bgr);

		int total_constellations() { return constellations_and_hulls_.size(); }

		void fit_model();

	private:
		void generate_colormap_(int total_colors = 32);

		cv::Size image_size_;
		PointConstellation3D calibration_pattern_world_constellations_;
		std::vector< std::pair<PointConstellation2D, PointConstellation2D> > constellations_and_hulls_; // 2D constellations of images of the pattern object and respective polygonal convex hulls
		std::vector<cv::Vec3b> colormap_;
	};
}
