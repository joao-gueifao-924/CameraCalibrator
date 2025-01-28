#pragma once
#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

class camera_calibration_model;

class camera_calibration_model
{
public:

	typedef std::vector<cv::Point2f> PointConstellation2f;
	typedef std::vector<cv::Point3f> PointConstellation3f;

	enum class pattern_status
	{
		undefined,
		pattern_not_found,
		pattern_too_similar,  // pattern was found, but deemed to similar, hence rejected
		pattern_not_held_long_enough, // in case of video, user must hold the pattern in the same position for a min of seconds in order to register it
		pattern_accepted
	};

	enum class fitting_status
	{
		undefined,
		not_enough_registered_images,
		same_previous_model,           // no fitting was retried, has the last model already was fitted to the latest data
		newly_fitted_camera_model,
		fitting_unsuccesful, // e.g., due to bad numerical convergence // if previously fitted model exists, it is kept
	};

	class flat_chessboard_pattern
	{
	public:
		flat_chessboard_pattern(float square_side_length_mm, cv::Size pattern_size);

		friend class camera_calibration_model;
	private:
		void generate_corner_3d_coordinates();
		float square_side_length_mm_{ 0.0f };
		cv::Size size_;
		PointConstellation3f corner_3d_coordinates_;
	};

	struct pattern_registration
	{
		cv::Mat input_image; // image of the pattern object
		PointConstellation2f pattern_corners; // extracted 2D point constellations of that pattern
		PointConstellation2f pattern_corners_convex_hull; //respective polygonal convex hulls
	};

	class camera_model
	{
	public:
		camera_model(cv::Size image_size, cv::Mat camera_matrix_K); // minimal case
		camera_model(double reprojection_error_rms, cv::Size image_size, cv::Mat camera_matrix_K, cv::Mat distortion_coefficients);

		friend std::ostream& operator<<(std::ostream& os, const camera_calibration_model::camera_model& obj);
	private:
		double reprojection_error_rms_{ 0.0 };
		cv::Size image_size_;
		cv::Mat camera_matrix_K_;
		cv::Mat distortion_coefficients_;
		std::vector<cv::Mat> rotation_vectors_;
		std::vector<cv::Mat> translation_vectors_;
	};

	// Remember that Fast Check erroneously fails with high distortions like fisheye.
	static constexpr int chessboard_flags = cv::CALIB_CB_ADAPTIVE_THRESH
		| cv::CALIB_CB_NORMALIZE_IMAGE
		| cv::CALIB_CB_FAST_CHECK;


	camera_calibration_model(float square_side_length_mm, cv::Size pattern_size, cv::Size image_size, bool input_image_folder_mode = false);
	pattern_status try_register(cv::Mat frame_bgr);
	fitting_status try_fit();
	camera_model extract_model();
	cv::Mat render_feedback_image(bool flip_horizontally = false);

	static double constellation_IoU(const camera_calibration_model::PointConstellation2f& constellation_a, const camera_calibration_model::PointConstellation2f& constellation_b);
	static double convex_polygons_IoU(const camera_calibration_model::PointConstellation2f& polygon_a, const camera_calibration_model::PointConstellation2f& polygon_b);
	friend std::ostream& operator<<(std::ostream& os, const camera_calibration_model::camera_model& obj);

	bool save_registered_images_to_folder(std::filesystem::path path);

private:
	bool input_image_folder_mode_{ false }; // mode is either input video or image folder
	bool is_different_enough(const PointConstellation2f& constellation);
	bool add_constellation(cv::Mat input_image_bgr, camera_calibration_model::PointConstellation2f constellation);
	PointConstellation2f flip_horizontally(int image_width, const camera_calibration_model::PointConstellation2f& points);
	void generate_colormap(int total_colors = 32);

	std::unique_ptr<flat_chessboard_pattern> calibration_pattern_;
	cv::Size image_size_;
	std::vector<pattern_registration> pattern_registrations;

	std::unique_ptr<camera_model> last_fitted_model_;

	// These are just for real-time visual feedback to the user
	std::vector<cv::Vec3b> colormap_;
	cv::Mat last_input_videoframe;
	PointConstellation2f anchor_corners, last_detected_pattern_corners_videoframe;
	std::chrono::steady_clock::time_point frame_new_pose_start_time;

};

std::ostream& operator<<(std::ostream& os, const camera_calibration_model::camera_model& obj);
