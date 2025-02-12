import QtQuick
import QtQuick.Controls.Universal
import QtQuick.Layouts
import QtMultimedia
import CameraCalibrator 1.0

ApplicationWindow {
    id: mainWindow
    Universal.theme: Universal.System

    // width: 640
    // height: 480
    visible: true
    minimumWidth: 853
    minimumHeight: 480
    title: qsTr("Hello World")
    //visibility: Window.Maximized


    // Background that will clear focus when clicked on
    Item {
        id: backgroundStealFocus
        anchors.fill: parent

        MouseArea {
            anchors.fill: parent
            onClicked: {
                backgroundStealFocus.forceActiveFocus()
            }
        }
    }

    RowLayout {
        anchors.fill: parent
        spacing: 0

        //Sidebar:
        Rectangle {
            id: sidebar
            Layout.fillHeight: true
            Layout.preferredWidth: 200
            color: palette.window

            // Sidebar content goes here
            ColumnLayout {
                //anchors.top: mainWindow
                spacing: 10
                // Add your menu items here

                ComboBox {
                    Layout.minimumWidth: 200
                    id: cbCamera
                    model: mediaDevices.videoInputs
                    textRole: "description"
                    onActivated: camera.active = true
                }

                Button {
                    id: button1
                    text: "Load Calibration"
                    Layout.fillWidth: true
                }
                Button {
                    id: button2
                    text: "Save Calibration"
                    Layout.fillWidth: true
                }

                Label {
                    text: "Square side length (mm)"
                    //font.pixelSize: 14
                }

                NumberField
                {
                    id: squareSideLengthNumberField
                    decimals: 2
                    value: 14.88
                    Layout.preferredWidth: 150
                }

                Label {
                    text: "Pattern corners wide"
                    //font.pixelSize: 14
                }

                NumberField
                {
                    id: patternCornersWideNumberField
                    decimals: 0
                    value: 10
                    width: 300
                }

                Label {
                    text: "Pattern corners tall"
                    //font.pixelSize: 14
                }

                NumberField
                {
                    id: patternCornersTallNumberField
                    decimals: 0
                    value: 6
                }
            }
        }

        // Separator
        Rectangle {
            Layout.fillHeight: true
            Layout.preferredWidth: 3
            color: Qt.darker(palette.window) // Uses system theme's dark color for separation
        }

        // Main content area
        ColumnLayout {
            id: mainContentLayout
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 0

            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: "black"

                VideoOutput {
                    id: inputCameraFeed_videoOutput
                    visible: false
                    Component.onCompleted:
                       calibrationProcessor.inputVideoSink = inputCameraFeed_videoOutput.videoSink
                }

                VideoOutput {
                    id: processedVideo_videoOutput
                    anchors.centerIn: parent
                    width: parent.width
                    height: parent.height
                    fillMode: VideoOutput.PreserveAspectFit
                    Component.onCompleted:
                        calibrationProcessor.outputVideoSink = processedVideo_videoOutput.videoSink
                }
            }



            Rectangle {
                id: panelBeneathVideoOutput
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.minimumHeight: 25
                Layout.maximumHeight: 25
                color: palette.window

                RowLayout
                {
                    anchors.fill: parent

                    Text {
                        id: patternResultText
                        text: calibrationProcessor.patternResultString
                        color: palette.text
                        Layout.preferredWidth: parent.width/2
                        horizontalAlignment: Text.AlignLeft
                    }

                    Text {
                        id: fittingResultText
                        text: calibrationProcessor.fittingResultString
                        color: palette.text
                        Layout.preferredWidth: parent.width/2
                        horizontalAlignment: Text.AlignLeft
                    }
                }
            }
        }
    }


    MediaDevices {
        id: mediaDevices
        Component.onCompleted: {
            // Lists the attached video inputs:
           for (let i = 0; i < mediaDevices.videoInputs.length; i++) {
               console.log("Camera", i,
                           "ID:", mediaDevices.videoInputs[i].id,
                           "Description:", mediaDevices.videoInputs[i].description)
           }
        }
    }

    CaptureSession {
          camera: Camera {
            id: camera
            active: true
            cameraDevice: mediaDevices.videoInputs[cbCamera.currentIndex]
          }

          videoOutput: inputCameraFeed_videoOutput
    }

    CalibrationProcessor {
        id: calibrationProcessor
        maxFrameRate: 30.0
        square_side_length_mm: squareSideLengthNumberField.value
        pattern_size: Qt.size(patternCornersWideNumberField.value, patternCornersTallNumberField.value)

        readonly property string fittingResultString: getFittingStatusQString(fittingStatus)
        readonly property string patternResultString: getPatternStatusQString(patternStatus)
    }
}
