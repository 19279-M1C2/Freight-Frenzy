package org.firstinspires.ftc.teamcode.components


import com.acmerobotics.dashboard.config.Config
import com.amarcolini.joos.command.AbstractComponent
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector
import org.firstinspires.ftc.teamcode.util.telemetry.RobotTelemetry


class Vision(hMap: HardwareMap) : AbstractComponent() {
    @Config(value = "Vision")
    companion object {
        var VUFORIA_KEY =
            "AQfsT7f/////AAABmUob5YNsRUaTsw8OCAwh8U0CkBoG6Niyn6ObvMlbI5xX/mYd1kc2pIXrQ+m6UEf/rZAbLSs4vY3PJe5VfIqPXMBZxEuBI8uVrRcPjWs60w8zBRmwf1z3akCWiwvMu3QNTei9oWkZ4hAWILWYLq2QEBlPjmGmEwnTM6VmJAPVSXRD4ZtBdZ7fKcrxlSwdMz1bZiEZZJEdQHTqqXY31zH3cvu9TOZxvYzFm94/JryfQSnhqaS2qOYen02JvqmDsWN8JpTeUCiDLOyX02femfeiZffIvu2zntQyoNUevcDYPptjYYlBGSDP6oTT9iOrvqf4tWyJeeO+kEMqt2Kd4KnEDMukHH5bruHlXO2wwOrOifrL"

        const val TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite"
        val LABELS = listOf(
            "Ball",
            "Cube",
            "Duck",
            "Marker"
        )
    }

    enum class ObjectPosition {
        LEFT,
        MIDDLE,
        RIGHT
    }

    private val vuforia: VuforiaLocalizer
    private val tfod: TFObjectDetector

    init {
        val parameters = VuforiaLocalizer.Parameters()

        parameters.vuforiaLicenseKey = VUFORIA_KEY
        parameters.cameraName = hMap.get(WebcamName::class.java, "Webcam 1")

        //  Instantiate the Vuforia engine

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters)
        RobotTelemetry.addLine("vuforia inited")


        val tfodMonitorViewId: Int = hMap.appContext.resources.getIdentifier(
            "tfodMonitorViewId", "id", hMap.appContext.packageName
        )
        val tfodParameters = TFObjectDetector.Parameters(tfodMonitorViewId)
        tfodParameters.minResultConfidence = 0.8f
        tfodParameters.isModelTensorFlow2 = true
        tfodParameters.inputSize = 320
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia)
        tfod.loadModelFromAsset(
            TFOD_MODEL_ASSET,
            "Ball", "Cube", "Duck", "Marker"
        )

        tfod.activate()
        tfod.setZoom(1.5, 16.0 / 9)
    }

    // getUpdatedRecognitions() will return null if no new information is available since
    // the last time that call was made.
    fun getPosition(): Pair<ObjectPosition, Double> =
        tfod.updatedRecognitions?.filter { it.label == "Marker" }?.let { recognitions ->
            if (recognitions.size >= 2) return ObjectPosition.LEFT to -1.0
            recognitions.first { it.label == "Marker" }?.let {
                val markerPosition = it.left.toDouble()
                if (markerPosition < 200) {
                    ObjectPosition.RIGHT to markerPosition
                } else {
                    ObjectPosition.MIDDLE to markerPosition
                }
            }
        } ?: (ObjectPosition.MIDDLE to -1.0)
}