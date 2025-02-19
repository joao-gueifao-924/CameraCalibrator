#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <Calibrationprocessor.h>

int main(int argc, char *argv[])
{
    int exitCode;
    do {
        QGuiApplication app(argc, argv);
        QQmlApplicationEngine engine;
        QObject::connect(
            &engine,
            &QQmlApplicationEngine::objectCreationFailed,
            &app,
            []() { QCoreApplication::exit(-1); },
            Qt::QueuedConnection);
        engine.loadFromModule("CameraCalibrator", "Main");

        exitCode = app.exec();
    } while (exitCode == CalibrationProcessor::EXIT_CODE_RESTART);

    return exitCode;
}
