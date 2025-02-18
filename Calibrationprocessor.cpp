// frameprocessor.cpp
#include "Calibrationprocessor.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>

static constexpr size_t FRAME_BUFFER_CAPACITY = 3;

typedef std::chrono::steady_clock sclock;

// internal helper method
std::chrono::microseconds convertToTimePeriod(double fps)
{
    return std::chrono::microseconds(static_cast<long long>(1e6 / fps));
}

// internal helper method

void sleep(long long millisec)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(millisec));
}

void CalibrationProcessor::processFrames() {

    cv::Mat ocv_image;
    cv::Mat presentation_frame;

    auto presentOutputFrame = [this, &presentation_frame]() {
        if (m_outputVideoSink && !presentation_frame.empty())
        {
            QImage outImage(presentation_frame.data,
                            presentation_frame.cols,
                            presentation_frame.rows,
                            presentation_frame.step,
                            QImage::Format_BGR888);
            QVideoFrame outFrame(outImage.copy());
            m_outputVideoSink->setVideoFrame(outFrame);
        }
    };

    while (!m_should_stop_processing_frames)
    {
        if (imageInputFolderMode_)
        {
            if (imageInputFolder_.empty())
            {
                // very ugly, but I am in a rush to make this app feature-complete...
                sleep(30);
                continue;
            }

            // TODO: I need to set a list of image files, then each iteration of the
            // big while-loop targets each image in the list.

            // Extracting path of individual image stored in a given directory
            std::vector<cv::String> images;

            // TODO: accept other kinds of image files, not just PNG.
            // Path of the folder containing checkerboard images
            std::string path = imageInputFolder_.string() + "/*.png";
            //std::string path = R"(C:\Users\joaog\temp_program_output\CameraCalibrator\*.png)";
            cv::glob(path, images);

            // Looping over all the images in the directory
            for (int i{ 0 }; i < images.size(); i++)
            {
                ocv_image = cv::imread(images[i]);
                camera_calibration_->set_image_size(ocv_image.size());
                bool tryFitNewModel = (i == images.size()-1); // only fit model at last image registration
                presentation_frame = singleImageIteration(ocv_image, tryFitNewModel);

                presentOutputFrame();
            }


            // TODO: this is quick and ugly, for obvious reasons...
            // For now, I need to clear the image input folder so that the images are not read
            // and processed over and over indefinitely.
            setImageInputFolder("");
            setReadyToSaveCalibration(camera_calibration_->is_model_available());
        }
        else // Live video input mode
        {
            QImage image;
            if (!m_inputVideoFramesBuffer.pop(image))
            {
                // Avoid CPU spiking in case frames buffer is currently set to not wait to be popped.
                sleep(30);
                continue;
            }
            ocv_image = cv::Mat(image.height(), image.width(), CV_8UC4, image.bits());
            cv::cvtColor(ocv_image, ocv_image, cv::COLOR_RGBA2BGR);
            camera_calibration_->set_image_size(ocv_image.size());
            presentation_frame = singleImageIteration(ocv_image);
            setReadyToSaveCalibration(camera_calibration_->is_model_available());

            presentOutputFrame();
        }
    }
}

void CalibrationProcessor::set_fitting_status(camera_calibration::fitting_status status)
{
    CalibrationProcessor::fitting_status qstatus = toQtEnum(status);
    if (qstatus == fitting_status_) return;
    fitting_status_.store(qstatus);
    emit fitting_statusChanged();
}

void CalibrationProcessor::setTotalRegisteredPatterns(int val)
{
    if (val == totalRegisteredPatterns_) return;
    totalRegisteredPatterns_ = val;
    emit totalRegisteredPatternsChanged();
}

void CalibrationProcessor::setReadyToSaveCalibration(bool val)
{
    if (readyToSaveCalibration_ == val) return;
    readyToSaveCalibration_ = val;
    emit readyToSaveCalibrationChanged();
}


void CalibrationProcessor::set_pattern_status(camera_calibration::pattern_status status)
{
    CalibrationProcessor::pattern_status qstatus = toQtEnum(status);
    if (qstatus == pattern_status_) return;
    pattern_status_.store(qstatus);
    emit pattern_statusChanged();
}

CalibrationProcessor::fitting_status CalibrationProcessor::toQtEnum(camera_calibration::fitting_status val)
{
    switch (val) {
    case camera_calibration::fitting_status::undefined:
        return fitting_status::Undefined;
    case camera_calibration::fitting_status::not_enough_registered_images:
        return fitting_status::NotEnoughRegisteredImages;
    case camera_calibration::fitting_status::same_previous_model:
        return fitting_status::SamePreviousModel;
    case camera_calibration::fitting_status::newly_fitted_camera_model:
        return fitting_status::NewlyFittedCameraModel;
    case camera_calibration::fitting_status::fitting_unsuccessful:
        return fitting_status::FittingUnsuccessful;
    default:
        throw std::invalid_argument("Unrecongnised value. Keep your enums in sync!");
    }
}

CalibrationProcessor::pattern_status CalibrationProcessor::toQtEnum(camera_calibration::pattern_status val)
{
    switch (val) {
    case camera_calibration::pattern_status::undefined:
        return pattern_status::Undefined;
    case camera_calibration::pattern_status::pattern_not_configured:
        return pattern_status::PatternNotConfigured;
    case camera_calibration::pattern_status::pattern_not_found:
        return pattern_status::PatternNotFound;
    case camera_calibration::pattern_status::pattern_too_similar:
        return pattern_status::PatternTooSimilar;
    case camera_calibration::pattern_status::pattern_not_held_long_enough:
        return pattern_status::PatternNotHeldLongEnough;
    case camera_calibration::pattern_status::pattern_accepted:
        return pattern_status::PatternAccepted;
    default:
        throw std::invalid_argument("Unrecongnised value. Keep your enums in sync!");
    }
}

CalibrationProcessor::CalibrationProcessor(QObject *parent)
    : QObject(parent)
    , camera_calibration_{ std::make_unique<camera_calibration>() }
    , m_inputVideoFramesBuffer{ circular_buffer_thread_safe<QImage>(FRAME_BUFFER_CAPACITY) }
    //, m_output_videoframes_buffer{ circular_buffer_thread_safe<QVideoFrame>(FRAME_BUFFER_CAPACITY) }
    , m_imageProcessingThread{ std::thread(&CalibrationProcessor::processFrames, this) }
    , m_lastFrameTime{ sclock::now() }
    , m_targetFrameTime_microsec { convertToTimePeriod(DEFAULT_MAX_FRAME_RATE) }
{
}

CalibrationProcessor::~CalibrationProcessor()
{
    m_should_stop_processing_frames = true;
    m_inputVideoFramesBuffer.should_wait_to_pop(false); // in case actively held on wait_pop() call

    if (m_imageProcessingThread.joinable())
    {
        m_imageProcessingThread.join();
    }
}

double CalibrationProcessor::maxFrameRate() const
{
    return 1e6 / m_targetFrameTime_microsec.count();
}

void CalibrationProcessor::setMaxFrameRate(double val)
{
    if (val < 0)
    {
        qWarning() << __FUNCTION__ << " does not accept negative values. Ignoring input value of " << val;
        return;
    }

    const auto input_val_microsec = convertToTimePeriod(val);
    const auto distance = std::chrono::abs(input_val_microsec - m_targetFrameTime_microsec);
    if (distance.count() < 1LL) return;

    m_targetFrameTime_microsec = input_val_microsec;
    emit maxFrameRateChanged();
}


QVideoSink* CalibrationProcessor::inputVideoSink() const { return m_inputVideoSink.get(); }


void CalibrationProcessor::setInputVideoSink(QVideoSink* newSink)
{
    if (m_inputVideoSink.get() == newSink) return;

    if (m_inputVideoSink)
        disconnect(m_inputVideoSink.get(), &QVideoSink::videoFrameChanged,
                   this, &CalibrationProcessor::receiveFrame);

    m_inputVideoSink.reset(newSink);

    if (m_inputVideoSink)
        connect(m_inputVideoSink.get(), &QVideoSink::videoFrameChanged,
                this, &CalibrationProcessor::receiveFrame);

    emit inputVideoSinkChanged();
}

QVideoSink *CalibrationProcessor::outputVideoSink() const { return m_outputVideoSink.get(); }

void CalibrationProcessor::setOutputVideoSink(QVideoSink *newSink)
{
    if (m_outputVideoSink.get() == newSink) return;
    m_outputVideoSink.reset(newSink);
    emit outputVideoSinkChanged();
}

double CalibrationProcessor::square_side_length_mm()
{
    if (!camera_calibration_) return 0.0;

    return static_cast<double>(camera_calibration_->square_side_length_mm());
}

void CalibrationProcessor::setSquare_side_length_mm(double val)
{
    if (!camera_calibration_) return;

    const double length = square_side_length_mm();
    if (std::abs(length - val) < 1e-3) return;

    const cv::Size size = camera_calibration_->pattern_size();
    camera_calibration_->set_calibration_pattern(val, size);
    emit square_side_length_mmChanged();
}

QSize CalibrationProcessor::pattern_size()
{
    if (!camera_calibration_) return QSize();
    const cv::Size size = camera_calibration_->pattern_size();
    return QSize{size.width, size.height};
}

void CalibrationProcessor::setPattern_size(QSize val)
{
    if (!camera_calibration_) return;

    const QSize size = pattern_size();
    if (size == val) return;

    const double length = square_side_length_mm();
    camera_calibration_->set_calibration_pattern(length, cv::Size{val.width(), val.height()});
    emit pattern_sizeChanged();
}

bool CalibrationProcessor::getImageInputFolderMode()
{
    return imageInputFolderMode_;
}

void CalibrationProcessor::setImageInputFolderMode(bool val)
{
    if (imageInputFolderMode_ == val) return;
    imageInputFolderMode_ = val;

    camera_calibration_->set_input_image_folder_mode(val);

    // TODO: improve this.
    // Right now, we need to have processFrames() stop waiting
    // to pop next video frame from the buffer whenever we
    // activate imageInputFolderMode_ and do the contrary otherwise.
    m_inputVideoFramesBuffer.should_wait_to_pop(!imageInputFolderMode_); // in case actively held on wait_pop() call

    emit imageInputFolderModeChanged();
}

QString CalibrationProcessor::getImageInputFolder()
{
    return QString::fromStdString(imageInputFolder_.string());
}

void CalibrationProcessor::setImageInputFolder(QString val)
{
    const std::filesystem::path valpath = std::filesystem::path(val.toStdString());
    if (valpath == imageInputFolder_) return;
    imageInputFolder_ = valpath;
    emit imageInputFolderChanged();
}

int CalibrationProcessor::totalRegisteredPatterns()
{
    return totalRegisteredPatterns_;
}

bool CalibrationProcessor::readyToSaveCalibration()
{
    return readyToSaveCalibration_;
}

bool CalibrationProcessor::saveCameraModelJson(QString filePath)
{
    if (!readyToSaveCalibration_) return false;

    std::ofstream file(filePath.toStdString());
    if (!file.is_open()) {
        return false;
    }

    std::string jsonString = camera_calibration_->extract_model().to_json();
    file << jsonString;
    file.close();
    return true;
}

CalibrationProcessor::pattern_status CalibrationProcessor::get_pattern_status()
{
    return pattern_status_.load();
}

CalibrationProcessor::fitting_status CalibrationProcessor::get_fitting_status()
{
    return fitting_status_.load();
}

void CalibrationProcessor::receiveFrame(const QVideoFrame &frame)
{
    auto workTime = sclock::now() - m_lastFrameTime;

    if (workTime >= m_targetFrameTime_microsec)
    {
        m_inputVideoFramesBuffer.push(frame.toImage());
        m_lastFrameTime = sclock::now();
    }
}

cv::Mat CalibrationProcessor::singleImageIteration(cv::Mat image_bgr, bool try_fit_new_model)
{
    if (image_bgr.empty())
    {
        throw std::runtime_error("Empty video frame");
    }

    if (!camera_calibration_)
    {
        throw std::runtime_error("Camera Calibration not instanciated");
    }

    auto pattern_status = camera_calibration_->try_register(image_bgr);
    set_pattern_status(pattern_status);

    setTotalRegisteredPatterns( static_cast<int>(camera_calibration_->get_total_registered_patterns() ) );

    cv::Mat presentation_frame = camera_calibration_->render_feedback_image(true);
    //cv::imshow("INPUT_IMAGE_WINDOW_TITLE", presentation_frame);
    //cv::waitKey(1);

    camera_calibration::fitting_status fitting_status{ camera_calibration::fitting_status::undefined };

    if (pattern_status == camera_calibration::pattern_status::pattern_accepted && try_fit_new_model)
    {
        fitting_status = camera_calibration_->try_fit();

        if (fitting_status == camera_calibration::fitting_status::newly_fitted_camera_model)
        {
            camera_calibration::camera_model new_model = camera_calibration_->extract_model();
            std::cout << "New camera model: " << std::endl;
            std::cout << new_model << std::endl;
        }
    }

    set_fitting_status(fitting_status);
    return presentation_frame;
}
