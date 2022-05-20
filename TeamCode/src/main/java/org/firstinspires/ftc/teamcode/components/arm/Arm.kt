package org.firstinspires.ftc.teamcode.components.arm

import com.acmerobotics.dashboard.config.Config
import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.hardware.Motor
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.teamcode.util.telemetry.RobotTelemetry

class Arm(
    val spool: Motor,
    val tipper: Tipper,
    val limitSwitch: TouchSensor?,
    val intake: Intake
) :
    AbstractComponent() {
    @Config(value = "Arm")
    companion object {
        @JvmField
        var LIMIT_SWITCH_NAME = "limit-switch"

        @JvmField
        var SPOOL_NAME = "spool"

        @JvmField
        var REVERSED = true

        @JvmField
        var BRAKE = true

        @JvmField
        var FLOOR_TICKS = 0

        @JvmField
        var INTAKE_TICKS = 0

        @JvmField
        var SHARED_TICKS = 0

        @JvmField
        var LOW_TICKS = 0

        @JvmField
        var MEDIUM_TICKS = 0

        @JvmField
        var HIGH_TICKS = 1900


    }

    enum class Position {
        FLOOR,
        SHARED,
        LOW,
        MEDIUM,
        HIGH,
        INTAKE;

        val ticks: Int
            get() = when (this) {
                FLOOR -> FLOOR_TICKS
                SHARED -> SHARED_TICKS
                LOW -> LOW_TICKS
                MEDIUM -> MEDIUM_TICKS
                HIGH -> HIGH_TICKS
                INTAKE -> INTAKE_TICKS
            }
    }

    init {
        subcomponents.add(tipper)
        subcomponents.add(spool)
        subcomponents.add(intake)

        if (REVERSED) spool.reversed()
        if (!BRAKE) spool.zeroPowerBehavior = Motor.ZeroPowerBehavior.FLOAT

        RobotTelemetry.addTelemetry("arm pos") { spool.currentPosition }
    }

    fun limitTipped() = limitSwitch?.isPressed

    fun setSpool(ticks: Int) = spool.goToPosition(ticks).runUntil { !spool.isBusy() || limitTipped() == true }
    fun setSpool(position: Position) = setSpool(position.ticks)

    fun goToPosition(position: Position): Command {
        // first check if we have to cross the intake ticks to get to our target
        val crossing =
            if (spool.currentPosition < position.ticks)
                Position.INTAKE.ticks in spool.currentPosition..position.ticks else
                Position.INTAKE.ticks in position.ticks..spool.currentPosition

        // if we arent crossing, we move on. If we are going downards do one thing, otherwise do another
        return if (!crossing) {
            setSpool(position)
        } else if (position.ticks < spool.currentPosition) {
            tipper.setPosition(Tipper.TipperPosition.TILT) then
                    setSpool(Position.INTAKE) then
                    tipper.setPosition(Tipper.TipperPosition.UNTIPPED) then
                    setSpool(position)
        } else {
            tipper.setPosition(Tipper.TipperPosition.UNTIPPED) then
                    setSpool(Position.INTAKE) then
                    tipper.setPosition(Tipper.TipperPosition.TILT) then
                    setSpool(position)
        }
    }

    // mannual drive, ability to tilt tipper and move arm
    fun drive(power: Double) {
        if (limitTipped() == true && power > 0) spool.power = 0.0
        else spool.power = power
    }
}