import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Control {
    id: root

    readonly property bool isDecimal: decimals > 0
    property int decimals: 2
    property double value: 0.0
    property alias placeholderText: inputField.placeholderText

    // A private flag to avoid recursive updates
    property bool _updating: false

    contentItem: TextField {
        id: inputField
        anchors.fill: parent
        validator: isDecimal ? doubleValidator : intValidator

        property string previousValue: text

        IntValidator {
            id: intValidator
        }

        DoubleValidator {
            id: doubleValidator
            notation: DoubleValidator.StandardNotation
            decimals: root.decimals
        }

        inputMethodHints: Qt.ImhFormattedNumbersOnly

        onFocusChanged: {
            if (focus) {
                previousValue = text
            }
        }

        // When text changes, update the external numeric property
        onTextChanged: {
            if (_updating) return

            _updating = true
            var num = Number(text)
            // Update value only if the text is a valid number
            if (!isNaN(num)) {
                root.value = num
            }
            _updating = false
        }

        Component.onCompleted: {
            // Ensure the displayed text reflects the initial numeric value
            text = (root.isDecimal) ? root.value.toFixed(root.decimals) : Math.round(root.value).toString()
        }

        Keys.onPressed: function(event) {
            if (event.key === Qt.Key_Return || event.key === Qt.Key_Enter) {
                event.accepted = true
                focus = false
                if (isDecimal) {
                    text = Number(text).toFixed(decimals)
                }
            } else if (event.key === Qt.Key_Escape) {
                event.accepted = true
                text = previousValue
                focus = false
            }
        }
    }

    // Listen for external changes to 'value' and update the text accordingly.
    onValueChanged: {
        if (!_updating) {
            _updating = true
            var formatted = (root.isDecimal) ? value.toFixed(root.decimals) : Math.round(value).toString()
            // Only update if the text is out of sync to avoid unnecessary assignments.
            if (inputField.text !== formatted)
                inputField.text = formatted
            _updating = false
        }
    }
}
