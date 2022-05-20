package org.firstinspires.ftc.teamcode.components.arm

import com.acmerobotics.dashboard.config.Config
import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.hardware.Servo
import org.firstinspires.ftc.teamcode.components.arm.Tipper.Tipper.TILT_TICKS
import org.firstinspires.ftc.teamcode.components.arm.Tipper.Tipper.TIPPED_TICKS
import org.firstinspires.ftc.teamcode.components.arm.Tipper.Tipper.UNTTIPED_TICKS
import org.firstinspires.ftc.teamcode.util.telemetry.RobotTelemetry

class Tipper(val tipper: Servo) : AbstractComponent() {
    @Config(value = "Tipper")
    object Tipper {
        var TIPPER_NAME = "tipper"

        @JvmField
        var UNTTIPED_TICKS = 0.95

        @JvmField
        var TIPPED_TICKS = 0.4

        @JvmField
        var TILT_TICKS = 0.8
    }

    enum class TipperPosition {
        TILT, UNTIPPED, TIPPED;

        val ticks: Double
            get() {
                return when (this) {
                    TILT -> TILT_TICKS
                    UNTIPPED -> UNTTIPED_TICKS
                    TIPPED -> TIPPED_TICKS
                }
            }
    }

    init {
        subcomponents.add(tipper)

        RobotTelemetry.addTelemetry("tipper pos") { tipper.position }
    }

    fun setPosition(position: Double) = Command.of {
        tipper.position = position
    }

    fun setPosition(position: TipperPosition) = setPosition(position.ticks)

    val tipped
        get() = tipper.position > TILT_TICKS

    fun toggleTip() = if (tipped) setPosition(TipperPosition.TILT) else setPosition(TipperPosition.TIPPED)
}