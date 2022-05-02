package org.firstinspires.ftc.teamcode.components.arm

import com.acmerobotics.dashboard.config.Config
import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.hardware.Servo
import org.firstinspires.ftc.teamcode.Constants.SERVO_RPM

class Tipper(private val tipper: Servo) : AbstractComponent() {
    @Config
    object Tipper {
        @JvmField
        var INTAKE_TILT_POSITION = 0.2

        @JvmField
        var TIPPER_NAME = "tipper"
    }

    fun setTipper(position: Double) = tipper.goToPosition(position, SERVO_RPM)
    fun tilt() = setTipper(Tipper.INTAKE_TILT_POSITION)

    fun tip() = setTipper(1.0)
    fun untip() = setTipper(0.0)

    val tipped
        get() = tipper.position > Tipper.INTAKE_TILT_POSITION

    fun toggleTip(): Command = if (tipped) untip() else tip()
}