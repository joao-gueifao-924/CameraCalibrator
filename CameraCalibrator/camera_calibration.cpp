#include "camera_calibration.h"
#include <random>
#include <ctime>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#pragma optimize("", off)

static constexpr int MIN_SECONDS_HOLD_TO_CHECK_SIMILARITY{ 1 }; // total seconds the user must hold the pattern in place until a check for similarity of pattern pose store is made and feedback message is shown
static constexpr int MIN_SECONDS_HOLD_TO_ADD{ 3 }; // total seconds the user must hold the pattern in place until a snapshot is taken and detected corners stored
static constexpr double MIN_PATTERN_HOLD_IOU{ 0.90 };
static constexpr int MIN_TOTAL_PATTERN_REGISTRATIONS{ 10 };
double CONSTELLATION_IOU_THRESHOLD{ 0.7 };
static const cv::Size WINDOW_SEARCH_SIZE{ 11, 11 };


// Utilitary stuff. This should be put into a separate utilitary repo someday...

typedef std::chrono::steady_clock::time_point time_point;

time_point now_time()
{
    return std::chrono::high_resolution_clock::now();
}
long long time_delta_ms(time_point end, time_point start)
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
}
long long time_delta_ms(time_point start)
{
    return time_delta_ms(now_time(), start);
}

double& e(cv::Mat M, size_t row, size_t col)
{
    if (M.depth() != CV_64F || M.channels() != 1)
    {
        throw std::invalid_argument("M has no CV_64F depth");
    }

    if (row > M.rows || col > M.cols)
    {
        throw std::invalid_argument("Trying to access M out of bounds");
    }

    return M.at<double>(row, col);
}

class MatTypeInfo {
public:
    static std::string getDepthInfo(const cv::Mat& mat) {
        switch (mat.depth()) {
        case CV_8U:  return "CV_8U  (8-bit unsigned integer)";
        case CV_8S:  return "CV_8S  (8-bit signed integer)";
        case CV_16U: return "CV_16U (16-bit unsigned integer)";
        case CV_16S: return "CV_16S (16-bit signed integer)";
        case CV_32S: return "CV_32S (32-bit signed integer)";
        case CV_32F: return "CV_32F (32-bit float)";
        case CV_64F: return "CV_64F (64-bit float)";
        default:     return "Unknown depth";
        }
    }

    static std::string getChannelsInfo(const cv::Mat& mat) {
        return "Channels: " + std::to_string(mat.channels());
    }

    static void printMatInfo(const cv::Mat& mat) {
        std::cout << "Matrix Information:\n"
                  << "Size: " << mat.rows << "x" << mat.cols << "\n"
                  << "Depth: " << getDepthInfo(mat) << "\n"
                  << getChannelsInfo(mat) << "\n"
                  << "Total elements: " << mat.total() << "\n"
                  << "Continuous: " << (mat.isContinuous() ? "Yes" : "No") << "\n";
    }
};


camera_calibration::PointConstellation2f camera_calibration::flip_horizontally(int image_width, const camera_calibration::PointConstellation2f& points)
{
    PointConstellation2f output;
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

void camera_calibration::initialize_class()
{
    generate_colormap();
    frame_new_pose_start_time = now_time();
}

camera_calibration::camera_calibration()
{
    initialize_class();
}

camera_calibration::camera_calibration(float square_side_length_mm, cv::Size pattern_size, cv::Size image_size, bool input_image_folder_mode)
    : calibration_pattern_{ std::make_unique<flat_chessboard_pattern>(square_side_length_mm , pattern_size) }
    , image_size_{ image_size }
    , input_image_folder_mode_{ input_image_folder_mode }
{
    initialize_class();
}

float camera_calibration::square_side_length_mm()
{
    if (!calibration_pattern_) return 0.0f;

    return calibration_pattern_->square_side_length_mm_;
}

cv::Size camera_calibration::pattern_size()
{
    if (!calibration_pattern_) return cv::Size();
    return calibration_pattern_->size_;
}

bool camera_calibration::input_image_folder_mode()
{
    return input_image_folder_mode_;
}

void camera_calibration::set_calibration_pattern(float square_side_length_mm, cv::Size pattern_size)
{
    calibration_pattern_ = std::make_unique<flat_chessboard_pattern>(square_side_length_mm , pattern_size);
}

void camera_calibration::set_image_size(cv::Size image_size)
{
    image_size_ = image_size;
}

void camera_calibration::set_input_image_folder_mode(bool value)
{
    input_image_folder_mode_ = value;
}

bool camera_calibration::is_model_available()
{
    return (last_fitted_model_ != nullptr);
}

camera_calibration::pattern_status camera_calibration::try_register(cv::Mat frame_bgr)
{
    using namespace std;
    using namespace cv;
    camera_calibration::PointConstellation2f corners;

    last_input_videoframe = frame_bgr.clone();
    last_detected_pattern_corners_videoframe.clear();

    if (!calibration_pattern_ || !calibration_pattern_->isValid())
    {
        return pattern_status::pattern_not_configured;
    }

    if (!findChessboardCorners(frame_bgr, calibration_pattern_->size_,
                               corners, chessboard_flags))
    {
        return pattern_status::pattern_not_found;
    }

    last_detected_pattern_corners_videoframe = corners;

    if (input_image_folder_mode_)
    {
        if (!is_different_enough(corners))
        {
            return pattern_status::pattern_too_similar;
        }
    }
    else // Input video mode
    {
        double corners_iou = camera_calibration::constellation_IoU(corners, anchor_corners);
        bool pattern_is_being_held = (corners_iou > MIN_PATTERN_HOLD_IOU);

        if (!pattern_is_being_held) {
            frame_new_pose_start_time = now_time();
            anchor_corners = corners;
        }

        double total_seconds_held = double(time_delta_ms(frame_new_pose_start_time) / 1000.0);

        //std::cout << "Pattern held for " << total_seconds_held << " seconds" << std::endl;

        if (total_seconds_held > MIN_SECONDS_HOLD_TO_CHECK_SIMILARITY && !is_different_enough(corners))
        {
            return pattern_status::pattern_too_similar;
        }

        if (total_seconds_held < MIN_SECONDS_HOLD_TO_ADD)
        {
            return pattern_status::pattern_not_held_long_enough;
        }
    }

    Mat frame_gray;
    cvtColor(frame_bgr, frame_gray, COLOR_BGR2GRAY);
    cornerSubPix(frame_gray, corners, WINDOW_SEARCH_SIZE,
        Size(-1, -1),
        TermCriteria(TermCriteria::EPS + TermCriteria::COUNT,
            30, 0.0001));

    // seems repeated, but seize the opportunity to show the refined 
    // corner positions if code execution reached this stage:
    last_detected_pattern_corners_videoframe = corners;

    bool add_status = add_constellation(frame_bgr, corners);

    assert(add_status);
    return pattern_status::pattern_accepted;
}

camera_calibration::fitting_status camera_calibration::try_fit()
{
    if (pattern_registrations_.size() < MIN_TOTAL_PATTERN_REGISTRATIONS)
	    return camera_calibration::fitting_status::not_enough_registered_images;

    // Perform camera calibration
    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    cv::Mat stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors;

    std::vector<PointConstellation2f> image_points;
    std::vector<PointConstellation3f> object_world_points_copies;

    assert(calibration_pattern_);

    for (size_t i = 0; i < pattern_registrations_.size(); i++)
    {
        image_points.push_back(pattern_registrations_[i].pattern_corners);
        object_world_points_copies.push_back(calibration_pattern_->corner_3d_coordinates_); // push several copies
    }

    double rms_reprojectionError = cv::calibrateCamera(object_world_points_copies, image_points,
                                    image_size_, cameraMatrix, distCoeffs, rvecs, tvecs,
                                    stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors);

#if 0 // Print results
    std::cout << "RMS error: " << rms_reprojectionError << std::endl;
    std::cout << "Camera matrix:\n" << cameraMatrix << std::endl;
    std::cout << "Distortion coefficients:\n" << distCoeffs << std::endl;
    std::cout << "Standard deviations of intrinsics:\n" << stdDeviationsIntrinsics << std::endl;
    std::cout << "Standard deviations of extrinsics:\n" << stdDeviationsExtrinsics << std::endl;
    std::cout << "Per-view errors:\n" << perViewErrors << std::endl;
#endif

    camera_calibration::camera_model::projection_parameters params;

    std::cout << MatTypeInfo::getDepthInfo(distCoeffs) << std::endl;

    // Ensure normalization of last element to 1:
    cameraMatrix /= e(cameraMatrix, 2, 2);

    params.fx = e(cameraMatrix, 0, 0);
    params.fy = e(cameraMatrix, 1, 1);
    params.cx = e(cameraMatrix, 0, 2);
    params.cy = e(cameraMatrix, 1, 2);
    params.k1 = e(distCoeffs, 0, 0);
    params.k2 = e(distCoeffs, 0, 1);
    params.p1 = e(distCoeffs, 0, 2);
    params.p2 = e(distCoeffs, 0, 3);
    params.k3 = e(distCoeffs, 0, 4); // k3 comes after p1 and p2

    params.fx_std = e(stdDeviationsIntrinsics, 0, 0);
    params.fy_std = e(stdDeviationsIntrinsics, 1, 0);
    params.cx_std = e(stdDeviationsIntrinsics, 2, 0);
    params.cy_std = e(stdDeviationsIntrinsics, 3, 0);
    params.k1_std = e(stdDeviationsIntrinsics, 4, 0);
    params.k2_std = e(stdDeviationsIntrinsics, 5, 0);
    params.p1_std = e(stdDeviationsIntrinsics, 6, 0);
    params.p2_std = e(stdDeviationsIntrinsics, 7, 0);
    params.k3_std = e(stdDeviationsIntrinsics, 8, 0); // k3 comes after p1 and p2

    // TODO: (re)initialize a full camera model, not just image size and camera matrix!
    last_fitted_model_ = std::make_unique<camera_calibration::camera_model>(rms_reprojectionError, image_size_, params);
    return fitting_status::newly_fitted_camera_model;
}

camera_calibration::camera_model camera_calibration::extract_model()
{
    if (!is_model_available())
    {
        throw std::runtime_error("Model is not available");
    }

    return *last_fitted_model_;
}

cv::Mat camera_calibration::render_feedback_image(bool flip_horizontally)
{
    cv::Mat output = last_input_videoframe.clone();

    std::mt19937 gen(42); // Fixed seed for reproducibility
    std::uniform_int_distribution<int> dist(0, colormap_.size() - 1); // Range [0, total_colors-1]

    for (size_t i = 0; i < pattern_registrations_.size(); i++)
    {
        // pick random color from colormap
        int random_idx = dist(gen); // Generate a random index
        cv::Vec3b random_color = colormap_.at(random_idx);

        const PointConstellation2f& hull = pattern_registrations_[i].pattern_corners_convex_hull;
        const PointConstellation2f& inner_corners = pattern_registrations_[i].pattern_corners;
        PointConstellation2i hull_integers, inner_corners_integers;

        hull_integers.reserve(hull.size());
        inner_corners_integers.reserve(inner_corners.size());

        for (size_t j = 0; j < inner_corners.size(); j++)
        {
            inner_corners_integers.push_back(cv::Point2i{ inner_corners[j] });
        }

        for (size_t j = 0; j < hull.size(); j++)
        {
            hull_integers.push_back(cv::Point2i{ hull[j] });
        }

        cv::polylines(output, hull_integers, true, random_color, 5);

        for (size_t j = 0; j < inner_corners_integers.size(); ++j)
        {
            cv::circle(output, inner_corners_integers[j], 2, random_color, 3);
        }

    }

    PointConstellation2f&  corners = last_detected_pattern_corners_videoframe;

    if (flip_horizontally)
    {
        cv::flip(output, output, 1);
        corners = camera_calibration::flip_horizontally(last_input_videoframe.cols, last_detected_pattern_corners_videoframe);
    }

    if(calibration_pattern_)
        drawChessboardCorners(output, calibration_pattern_->size_, corners, true);

    return output;
}

size_t camera_calibration::get_total_registered_patterns()
{
    return pattern_registrations_.size();
}

camera_calibration::PointConstellation2f camera_calibration::get_currently_detected_pattern_hull()
{
    camera_calibration::PointConstellation2f hull;
    cv::convexHull(last_detected_pattern_corners_videoframe, hull);
    return hull;
}

std::vector<camera_calibration::PointConstellation2f> camera_calibration::get_pattern_registration_hulls()
{
    std::vector<PointConstellation2f> output;
    output.reserve(pattern_registrations_.size());
    
    for (size_t i = 0; i < pattern_registrations_.size(); i++)
    {
        output.push_back(pattern_registrations_[i].pattern_corners_convex_hull);
    }
    return output;
}

void camera_calibration::flat_chessboard_pattern::generate_corner_3d_coordinates()
{
	for (int i = 0; i < size_.height; i++)
	{
		for (int j = 0; j < size_.width; j++)
		{
            corner_3d_coordinates_.emplace_back(j * square_side_length_mm_, i * square_side_length_mm_, 0.0f);
		}
	}
}

std::ostream& operator<<(std::ostream& os, const camera_calibration::camera_model& obj)
{
    std::cout.imbue(std::locale(""));
    os << "Reproj error RMS: " << obj.reprojection_error_rms_ << " [pixels]" << std::endl;
    os << "Image size: " << obj.image_size_.width << " X " << obj.image_size_.height << " [pixels]" << std::endl;
    os << "Focal length (fx, fy): (" << obj.projection_parameters_.fx << ", " << obj.projection_parameters_.fy << ")" << " [pixels]" << std::endl;
    os << "Focal length 3*std.dev(fx, fy): (" << 3.0 * obj.projection_parameters_.fx_std << ", " << 3.0 * obj.projection_parameters_.fy_std << ")" << " [pixels]" << std::endl;
    os << "Optical Center (cx, cy): (" << obj.projection_parameters_.cx << ", " << obj.projection_parameters_.cy << ")" << " [pixels]" << std::endl;
    os << "Optical Center 3*std.dev(cx, cy): (" << 3.0 * obj.projection_parameters_.cx_std << ", " << 3.0 * obj.projection_parameters_.cy_std << ")" << " [pixels]" << std::endl;
    os << "Radial lens distort. (k1, k2, k3): (" << obj.projection_parameters_.k1 << ", " << obj.projection_parameters_.k2 << ", " << obj.projection_parameters_.k3 << ")" << std::endl;
    os << "Radial lens distort. 3*std.dev(k1, k2, k3): (" << obj.projection_parameters_.k1_std << ", " << obj.projection_parameters_.k2_std << ", " << obj.projection_parameters_.k3_std << ")" << std::endl;
    os << "Tangent. lens distort. (p1, p2): (" << obj.projection_parameters_.p1 << ", " << obj.projection_parameters_.p2 << ")" << std::endl;
    os << "Tangent. lens distort. 3*std.dev(p1, p2): (" << 3.0 * obj.projection_parameters_.p1_std << ", " << 3.0 * obj.projection_parameters_.p2_std << ")" << std::endl;

    return os; // Return the stream for chaining
}

camera_calibration::flat_chessboard_pattern::flat_chessboard_pattern(float square_side_length_mm, cv::Size pattern_size)
    : square_side_length_mm_{square_side_length_mm}
    , size_{pattern_size}
{
    generate_corner_3d_coordinates();
}

bool camera_calibration::flat_chessboard_pattern::isValid()
{
    if (square_side_length_mm_ < 0.01
        || size_.width < 2 || size_.height < 2)
        return false;

    return true;
}


double camera_calibration::constellation_IoU(const camera_calibration::PointConstellation2f& constellation_a, const camera_calibration::PointConstellation2f& constellation_b)
{
    if (constellation_a.size() < 4 || constellation_b.size() < 4)
    {
        return 0.0;
    }

    PointConstellation2f hull_a, hull_b;
    cv::convexHull(constellation_a, hull_a);
    cv::convexHull(constellation_b, hull_b);

    return camera_calibration::convex_polygons_IoU(hull_a, hull_b);
}


double camera_calibration::convex_polygons_IoU(const camera_calibration::PointConstellation2f& polygon_a, const camera_calibration::PointConstellation2f& polygon_b)
{
    if (polygon_a.size() < 4 || polygon_b.size() < 4)
    {
        return 0.0;
    }

    double area_a = cv::contourArea(polygon_a);
    double area_b = cv::contourArea(polygon_b);

    // Compute intersection polygon
    PointConstellation2f intersection_polygon;
    float intersection_area = cv::intersectConvexConvex(polygon_a, polygon_b, intersection_polygon);

    // Compute Intersection over Union
    double union_area = area_a + area_b - intersection_area;
    double iou = (union_area > 0) ? (intersection_area / union_area) : 0.0;
    return iou;
}

bool camera_calibration::is_different_enough(const camera_calibration::PointConstellation2f& constellation)
{
    camera_calibration::PointConstellation2f hull;
    cv::convexHull(constellation, hull);

    for (size_t i = 0; i < pattern_registrations_.size(); i++)
    {
        double iou = camera_calibration::convex_polygons_IoU(hull, pattern_registrations_[i].pattern_corners_convex_hull);

        if (iou > CONSTELLATION_IOU_THRESHOLD)
        {
            return false;
        }
    }

    return true;
}

bool camera_calibration::add_constellation(cv::Mat input_image_bgr, camera_calibration::PointConstellation2f constellation)
{
    if (!is_different_enough(constellation)) return false;

    PointConstellation2f hull;
    cv::convexHull(constellation, hull);
    pattern_registration new_registration;
    
    new_registration.input_image = input_image_bgr.clone();
    new_registration.pattern_corners = constellation;
    new_registration.pattern_corners_convex_hull = hull;
    
    pattern_registrations_.push_back(new_registration);
    return true;
}

void camera_calibration::generate_colormap(int total_colors)
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

bool camera_calibration::save_registered_images_to_folder(std::filesystem::path path)
{
    cv::Mat frame_bgr;
    int total_pictures_saved = 0;

    std::time_t now = std::time(nullptr);
    std::tm local_tm;
    
    if (localtime_s(&local_tm, &now) != 0)
    {
        return false; 
    }

    std::ostringstream oss;
    oss << std::put_time(&local_tm, "%Y%m%d_%H%M%S");
    const std::string formatted_timestamp_str = oss.str();

    // Save eachh image to file in the format image_0000.png, image_0001.png, etc.
    for (size_t i = 0; i < pattern_registrations_.size(); i++)
    {
        frame_bgr = pattern_registrations_[i].input_image;
        std::ostringstream filename;
        filename << "image_" << formatted_timestamp_str << "_" << std::setw(4) << std::setfill('0') << total_pictures_saved++ << ".png";
        auto full_path = path / filename.str();

        if (!cv::imwrite(full_path.string(), frame_bgr))
        {
            return false;
        }
    }

    return true;
}


camera_calibration::camera_model::camera_model(double reprojection_error_rms, cv::Size image_size, const projection_parameters &params)
    : reprojection_error_rms_{reprojection_error_rms}
    , image_size_{image_size}
    , projection_parameters_{params}
{
}

std::string camera_calibration::camera_model::to_json() const
{
    std::stringstream ss;
    ss << "{\n"
       << "  \"reprojection_error_rms\": " << reprojection_error_rms_ << ",\n"
       << "  \"image_size\": {\n"
       << "    \"width\": " << image_size_.width << ",\n"
       << "    \"height\": " << image_size_.height << "\n"
       << "  },\n"
       << "  \"projection_parameters\": {\n"
       << "    \"fx\": " << projection_parameters_.fx << ",\n"
       << "    \"fy\": " << projection_parameters_.fy << ",\n"
       << "    \"cx\": " << projection_parameters_.cx << ",\n"
       << "    \"cy\": " << projection_parameters_.cy << ",\n"
       << "    \"k1\": " << projection_parameters_.k1 << ",\n"
       << "    \"k2\": " << projection_parameters_.k2 << ",\n"
       << "    \"k3\": " << projection_parameters_.k3 << ",\n"
       << "    \"p1\": " << projection_parameters_.p1 << ",\n"
       << "    \"p2\": " << projection_parameters_.p2 << ",\n"
       << "    \"fx_std\": " << projection_parameters_.fx_std << ",\n"
       << "    \"fy_std\": " << projection_parameters_.fy_std << ",\n"
       << "    \"cx_std\": " << projection_parameters_.cx_std << ",\n"
       << "    \"cy_std\": " << projection_parameters_.cy_std << ",\n"
       << "    \"k1_std\": " << projection_parameters_.k1_std << ",\n"
       << "    \"k2_std\": " << projection_parameters_.k2_std << ",\n"
       << "    \"k3_std\": " << projection_parameters_.k3_std << ",\n"
       << "    \"p1_std\": " << projection_parameters_.p1_std << ",\n"
       << "    \"p2_std\": " << projection_parameters_.p2_std << "\n"
       << "  }\n"
       << "}";
    return ss.str();
}
