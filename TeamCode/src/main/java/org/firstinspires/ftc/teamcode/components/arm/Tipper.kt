package org.firstinspires.ftc.teamcode.components.arm

import com.acmerobotics.dashboard.config.Config
import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.hardware.Servo
import org.firstinspires.ftc.teamcode.Constants.SERVO_RPM
import org.firstinspires.ftc.teamcode.util.telemetry.RobotTelemetry

class Tipper(private val tipper: Servo) : AbstractComponent() {
    @Config(value = "Tipper")
    companion object {
        var TIPPER_NAME = "tipper"

        var UNTTIPED_TICKS = 0.0
        var TIPPED_TICKS = 1.0
        var FIRST_TILT_TICKS = 0.2
        var SECOND_TILT_TICKS = 0.3
    }

    enum class TipperPosition(val ticks: Double) {
        FIRST_TILT(FIRST_TILT_TICKS), SECOND_TILT(SECOND_TILT_TICKS), UNTIPPED(UNTTIPED_TICKS), TIPPED(TIPPED_TICKS)
    }

    init {
        subcomponents.add(tipper)

        RobotTelemetry.addTelemetry("tipper pos") { tipper.position }
    }

    fun setPosition(position: Double) = tipper.goToPosition(position, SERVO_RPM)

    fun setPosition(position: TipperPosition) = setPosition(position.ticks)

    val tipped
        get() = tipper.position > TipperPosition.SECOND_TILT.ticks

    fun toggleTip() = if (tipped) setPosition(TipperPosition.SECOND_TILT) else setPosition(TipperPosition.TIPPED)
}