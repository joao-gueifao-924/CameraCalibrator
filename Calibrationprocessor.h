#pragma once

#include <QObject>
#include <QVideoSink>
#include <QVideoFrame>
#include <QQmlEngine>
#include "circularbuffer_threadsafe.h"
#include "CameraCalibrator/camera_calibration.h"
#include <filesystem>

class CalibrationProcessor : public QObject
{
    Q_OBJECT
    QML_ELEMENT
    Q_CLASSINFO("RegisterEnumClassesUnscoped", "false")

public:

    // Remember to keep in sync with corresponding enums inside camera_calibration backend class!
    enum class pattern_status
    {
        Undefined,
        PatternNotConfigured,
        PatternNotFound,				// pattern couldn't be found in the provided image or videoframe
        PatternTooSimilar,			// pattern was found, but its pose deemed similar to one of the poses already registered, hence rejected
        PatternNotHeldLongEnough,	// in case of video, user must hold the pattern in the same position for a min of seconds in order to register it
        PatternAccepted
    };
    Q_ENUM(pattern_status)
    std::string getPatternStatusString(pattern_status status) {
        switch (status) {
        case pattern_status::Undefined:
            return "Pattern status undefined";
        case pattern_status::PatternNotConfigured:
            return "Pattern not configured";
        case pattern_status::PatternNotFound:
            return "Pattern not found";
        case pattern_status::PatternTooSimilar:
            return "Pattern too similar";
        case pattern_status::PatternNotHeldLongEnough:
            return "Pattern not held long enough";
        case pattern_status::PatternAccepted:
            return "Pattern accepted";
        default:
            return "Unknown Status";
        }
    }
    Q_INVOKABLE QString getPatternStatusQString(pattern_status status)
    {
        return QString::fromStdString(getPatternStatusString(status));
    }

    // Remember to keep in sync with corresponding enums inside camera_calibration backend class!
    enum class fitting_status
    {
        Undefined,
        NotEnoughRegisteredImages,
        SamePreviousModel,			// fitting was not retried, because current model already was fitted to the latest data
        NewlyFittedCameraModel,
        FittingUnsuccessful,			// e.g., due to bad numerical convergence // if previously fitted model exists, it is kept
    };
    Q_ENUM(fitting_status)
    std::string getFittingStatusString(fitting_status status) {
        switch (status) {
        case fitting_status::Undefined:
            return "Fitting status undefined";
        case fitting_status::NotEnoughRegisteredImages:
            return "Insufficient images for calibration";
        case fitting_status::SamePreviousModel:
            return "Using previous calibration model";
        case fitting_status::NewlyFittedCameraModel:
            return "Newly fitted calibration model";
        case fitting_status::FittingUnsuccessful:
            return "Fitting unsuccessful";
        default:
            return "Unknown Status";
        }
    }
    Q_INVOKABLE QString getFittingStatusQString(fitting_status status)
    {
        return QString::fromStdString(getFittingStatusString(status));
    }

    Q_PROPERTY(double maxFrameRate READ maxFrameRate WRITE setMaxFrameRate NOTIFY maxFrameRateChanged)
    Q_PROPERTY(QVideoSink* inputVideoSink READ inputVideoSink WRITE setInputVideoSink NOTIFY inputVideoSinkChanged)
    Q_PROPERTY(QVideoSink* outputVideoSink READ outputVideoSink WRITE setOutputVideoSink NOTIFY outputVideoSinkChanged)
    Q_PROPERTY(double square_side_length_mm READ square_side_length_mm WRITE setSquare_side_length_mm NOTIFY square_side_length_mmChanged);
    Q_PROPERTY(QSize pattern_size READ pattern_size WRITE setPattern_size NOTIFY pattern_sizeChanged);
    Q_PROPERTY(fitting_status fittingStatus READ get_fitting_status NOTIFY fitting_statusChanged);
    Q_PROPERTY(pattern_status patternStatus READ get_pattern_status NOTIFY pattern_statusChanged);
    Q_PROPERTY(bool imageInputFolderMode READ getImageInputFolderMode WRITE setImageInputFolderMode NOTIFY imageInputFolderModeChanged);
    Q_PROPERTY(QString imageInputFolder READ getImageInputFolder WRITE setImageInputFolder NOTIFY imageInputFolderChanged);
    Q_PROPERTY(int totalRegisteredPatterns READ totalRegisteredPatterns NOTIFY totalRegisteredPatternsChanged);
    Q_PROPERTY(bool readyToSaveCalibration READ readyToSaveCalibration NOTIFY readyToSaveCalibrationChanged);

public:
    static constexpr double DEFAULT_MAX_FRAME_RATE = 30.0;
    explicit CalibrationProcessor(QObject *parent = nullptr);
    ~CalibrationProcessor();

    double maxFrameRate() const;
    void setMaxFrameRate(double val);

    QVideoSink* inputVideoSink() const;
    void setInputVideoSink(QVideoSink* newSink);
    QVideoSink* outputVideoSink() const;
    void setOutputVideoSink(QVideoSink* newSink);
    double square_side_length_mm();
    void setSquare_side_length_mm(double val);
    QSize pattern_size();
    void setPattern_size(QSize val);

    fitting_status get_fitting_status();
    pattern_status get_pattern_status();
    bool getImageInputFolderMode();
    void setImageInputFolderMode(bool val);
    QString getImageInputFolder();
    void setImageInputFolder(QString val);
    int totalRegisteredPatterns();
    bool readyToSaveCalibration();

    Q_INVOKABLE bool saveCameraModelJson(QString filePath);

signals:
    void maxFrameRateChanged();
    void inputVideoSinkChanged();
    void outputVideoSinkChanged();
    void square_side_length_mmChanged();
    void pattern_sizeChanged();
    void fitting_statusChanged();
    void pattern_statusChanged();
    void imageInputFolderModeChanged();
    void imageInputFolderChanged();
    void totalRegisteredPatternsChanged();
    void readyToSaveCalibrationChanged();

private slots:
    void receiveFrame(const QVideoFrame &frame);

private:
    cv::Mat singleImageIteration(cv::Mat image_bgr, bool try_fit_new_model = true);
    void processFrames();
    pattern_status toQtEnum(camera_calibration::pattern_status val);
    fitting_status toQtEnum(camera_calibration::fitting_status val);

    void set_fitting_status(camera_calibration::fitting_status status);
    void set_pattern_status(camera_calibration::pattern_status status);
    void setTotalRegisteredPatterns(int val);
    void setReadyToSaveCalibration(bool val);

    std::unique_ptr<camera_calibration> camera_calibration_;
    circular_buffer_thread_safe<QImage> m_inputVideoFramesBuffer;
    std::shared_ptr<QVideoSink> m_inputVideoSink, m_outputVideoSink;
    std::thread m_imageProcessingThread;
    std::atomic<bool> m_should_stop_processing_frames{ false };
    std::chrono::steady_clock::time_point m_lastFrameTime;
    std::chrono::microseconds m_targetFrameTime_microsec;
    std::atomic<pattern_status> pattern_status_{ pattern_status::Undefined };
    std::atomic<fitting_status> fitting_status_{ fitting_status::Undefined };
    std::atomic<bool> imageInputFolderMode_{ false };
    std::filesystem::path imageInputFolder_;
    std::atomic<int> totalRegisteredPatterns_{ 0 };
    std::atomic<bool> readyToSaveCalibration_{false};
};
