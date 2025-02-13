import QtQuick
import QtQuick.Controls.Universal
import QtQuick.Layouts
import QtQuick.Dialogs
import QtCore
import QtMultimedia
import CameraCalibrator 1.0

ApplicationWindow {
    id: mainWindow
    title: "Camera Calibrator"
    Universal.theme: Universal.System

    // width: 640
    // height: 480
    visible: true
    minimumWidth: 853
    minimumHeight: 480
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

            ColumnLayout {
                spacing: 10
                // Add your menu items here

                Column {
                    spacing: 10

                    ButtonGroup {
                        id: modeGroup
                        exclusive: true
                    }

                    RadioButton {
                        id: liveVideoModeRadioButton
                        text: "Live Video Mode"
                        checked: false
                        ButtonGroup.group: modeGroup
                    }

                    RadioButton {
                        id: imageFolderModeRadioButton
                        text: "Image Folder Mode"
                        checked: false
                        ButtonGroup.group: modeGroup
                    }
                }

                ComboBox {
                    id: cbCamera
                    Layout.minimumWidth: 200
                    visible: liveVideoModeRadioButton.checked
                    model: mediaDevices.videoInputs
                    textRole: "description"
                }

                Button {
                    id: selectFolderButton
                    text: "Open Folder..."
                    visible: imageFolderModeRadioButton.checked
                    Layout.fillWidth: true
                    onClicked: inputImageFolderDialog.open()
                }

                Label {
                    text: "Square side length (mm)"
                }

                NumberField
                {
                    id: squareSideLengthNumberField
                    decimals: 2
                    value: 14.88
                    Layout.preferredWidth: 150
                }

                Label {
                    text: "Total corners wide x tall"
                }

                RowLayout
                {
                    NumberField
                    {
                        id: patternCornersWideNumberField
                        decimals: 0
                        value: 10
                        width: 300
                    }

                    Label {
                        text: "X"
                    }

                    NumberField
                    {
                        id: patternCornersTallNumberField
                        decimals: 0
                        value: 6
                    }
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
                    visible: false // this is a hack, I need to learn how to avoid a intermediary video output
                    Component.onCompleted:
                       calibrationProcessor.inputVideoSink = inputCameraFeed_videoOutput.videoSink
                }

                VideoOutput {
                    id: processedVideo_videoOutput
                    visible: liveVideoModeRadioButton.checked
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

    FolderDialog {
        id: inputImageFolderDialog
        currentFolder: StandardPaths.writableLocation(StandardPaths.DocumentsLocation)
        title: "Please choose a folder with your images"

        onAccepted: {
            console.log("Selected folder:", selectedFolder)
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
            active: liveVideoModeRadioButton.checked
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
