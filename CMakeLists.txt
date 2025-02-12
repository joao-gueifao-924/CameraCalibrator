cmake_minimum_required(VERSION 3.16)

project(CameraCalibrator VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 REQUIRED COMPONENTS Quick Multimedia)
find_package(OpenCV REQUIRED)
find_package(Boost 1.85.0 REQUIRED COMPONENTS filesystem system)

qt_standard_project_setup(REQUIRES 6.5)

qt_add_executable(appCameraCalibrator
    main.cpp
    CameraCalibrator/camera_calibration.h CameraCalibrator/camera_calibration.cpp
    circularbuffer_threadsafe.h
)

qt_add_qml_module(appCameraCalibrator
    URI CameraCalibrator
    VERSION 1.0
    QML_FILES
        Main.qml
        SOURCES Calibrationprocessor.h Calibrationprocessor.cpp
        QML_FILES NumberField.qml

)

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
set_target_properties(appCameraCalibrator PROPERTIES
#    MACOSX_BUNDLE_GUI_IDENTIFIER com.example.appCameraCalibrator
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)


target_include_directories(appCameraCalibrator PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(appCameraCalibrator PRIVATE ${Boost_INCLUDE_DIRS})

target_link_libraries(appCameraCalibrator
    PRIVATE Qt6::Quick
    Qt6::Multimedia
    ${OpenCV_LIBS}
)

include(GNUInstallDirs)
install(TARGETS appCameraCalibrator
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
