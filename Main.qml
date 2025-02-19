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

    minimumWidth: sidebarOuterColumnLayout.implicitWidth + 2 * sidebarOuterColumnLayout.anchors.margins + 320
    minimumHeight: sidebarOuterColumnLayout.implicitHeight + 2 * sidebarOuterColumnLayout.anchors.margins
    visible: true


    Component.onCompleted: {
        width = 853 + sidebar.width + separatorVerticalLine.width
        height = 480 + topPanelBeneathVideoOutput.height + bottomPanelBeneathVideoOutput.height
    }

    property url calibrationSaveFolder: "";

    // Remove "file:///" prefix of paths:
    function removeFileProtocolPrefix(fileUrl)
    {
        const pathname = new URL(fileUrl).pathname
        const pathnameNoPrefix = pathname.replace(/^\/([A-Za-z]:\/)/, '$1')
        return pathnameNoPrefix
    }

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
            Layout.preferredWidth: 225
            color: palette.window

            ColumnLayout {
                id: sidebarOuterColumnLayout
                spacing: 10
                anchors.top: parent.top
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.margins: 10

                ColumnLayout {
                    id: calibrationInputParamsGroup
                    enabled: calibrationProcessor.allowChangingInputParameters
                    Layout.fillWidth: true

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
                            onCheckedChanged: calibrationProcessor.imageInputFolderMode = imageFolderModeRadioButton.checked
                        }
                    }

                    ComboBox {
                        id: cbCamera
                        Layout.fillWidth: true
                        visible: liveVideoModeRadioButton.checked
                        model: mediaDevices.videoInputs
                        textRole: "description"
                        //popup.width: 400
                        implicitContentWidthPolicy: ComboBox.WidestText
                    }

                    Button {
                        id: selectFolderButton
                        text: "Select Folder..."
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
                        Layout.preferredWidth: 75
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
                            Layout.preferredWidth: 50
                        }

                        Label {
                            text: "X"
                        }

                        NumberField
                        {
                            id: patternCornersTallNumberField
                            decimals: 0
                            value: 6
                            Layout.preferredWidth: 50
                        }
                    }
                }

                Button {
                    id: saveCalibrationButton
                    text: "Save Calibration..."
                    enabled: calibrationProcessor.readyToSaveCalibration
                    Layout.fillWidth: true
                    onClicked: saveCalibrationFileDialog.open()
                }

                Button {
                    id: restartButton
                    text: "Restart session"
                    enabled: !calibrationProcessor.allowChangingInputParameters
                    Layout.fillWidth: true
                    onClicked: calibrationProcessor.restartApp()
                }
            }
        }

        // Separator
        Rectangle {
            id: separatorVerticalLine
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
                    visible: false // this is a hack, I need to learn how to avoid this intermediary video output
                    Component.onCompleted:
                       calibrationProcessor.inputVideoSink = inputCameraFeed_videoOutput.videoSink
                }

                VideoOutput {
                    id: processedVideo_videoOutput
                    // TODO: Use a proper image viewer when doing camera calibration with images instead of live video
                    // For now we're using VideoOuptut for both.
                    visible: calibrationProcessor.imageInputFolderMode
                                ? (calibrationProcessor.imageInputFolder.length > 0 || calibrationProcessor.totalRegisteredPatterns > 0)
                                : true
                    anchors.centerIn: parent
                    width: parent.width
                    height: parent.height
                    fillMode: VideoOutput.PreserveAspectFit
                    Component.onCompleted:
                        calibrationProcessor.outputVideoSink = processedVideo_videoOutput.videoSink
                }
            }


            Rectangle {
                id: topPanelBeneathVideoOutput
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.minimumHeight: 25
                Layout.maximumHeight: 25
                color: palette.window
                //visible: calibrationProcessor.imageInputFolderMode // imageInputFolderPathText.text.length > 0

                Text {
                    id: imageInputFolderPathText
                    color: palette.text
                    text: {
                        let output = ""
                        if (!liveVideoModeRadioButton.checked && !imageFolderModeRadioButton.checked)
                        {
                            return "First, configure your calibration session"
                        }

                        if (liveVideoModeRadioButton.checked)
                        {
                            if (calibrationProcessor.readyToSaveCalibration)
                                return "Save calibration model now or keep registering more pattern poses for potential improvement"
                            if (calibrationProcessor.totalRegisteredPatterns === 0)
                                return "Present calibration pattern and hold it completely still until it gets registered"
                            return "Now keep registering more pattern poses by presenting and holding still"
                        }
                        if (liveVideoModeRadioButton.checked && calibrationProcessor.readyToSaveCalibration)
                        {
                            output = "Present and hold calibration pattern until it gets registered"
                        }
                        if (imageFolderModeRadioButton.checked)
                        {
                            if (calibrationProcessor.readyToSaveCalibration)
                            {
                                output = "Ready to save calibration model"
                            }
                            else
                            {
                                output = calibrationProcessor.imageInputFolder.length > 0
                                  ? calibrationProcessor.imageInputFolder
                                  : "Select the folder with images to be processed..."
                            }
                        }
                        return output
                    }
                }
            }

            Rectangle {
                id: bottomPanelBeneathVideoOutput
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
            calibrationSaveFolder = selectedFolder
            const folderPathNoPrefix = removeFileProtocolPrefix(selectedFolder)
            calibrationProcessor.imageInputFolder = folderPathNoPrefix
        }
    }

    FileDialog {
        id: saveCalibrationFileDialog
        title: "Save Calibration File"
        fileMode: FileDialog.SaveFile
        defaultSuffix: "json"
        nameFilters: ["JSON files (*.json)"]
        currentFolder: calibrationSaveFolder

        onAccepted: {
            const filePathNoPrefix = removeFileProtocolPrefix(selectedFile)
            calibrationProcessor.saveCameraModelJson(filePathNoPrefix)
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
        readonly property bool allowChangingInputParameters: totalRegisteredPatterns == 0
    }
}
