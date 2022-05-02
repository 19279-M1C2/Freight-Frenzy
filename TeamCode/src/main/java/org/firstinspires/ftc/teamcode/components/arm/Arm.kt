package org.firstinspires.ftc.teamcode.components.arm

import com.acmerobotics.dashboard.config.Config
import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.SequentialCommand
import com.amarcolini.joos.hardware.Motor
import com.qualcomm.robotcore.hardware.TouchSensor

class Arm(
    private val spool: Motor,
    private val tipper: Tipper,
    private val limitSwitch: TouchSensor,
    private val intake: Intake
) :
    AbstractComponent() {
    @Config
    object Arm {
        @JvmField
        var LIMIT_SWITCH_NAME = "limit-switch"

        @JvmField
        var SPOOL_NAME = "spool"

        @JvmField
        var INTAKE_TICKS = 1
    }

    enum class Position(val floorTicks: Int) {
        FLOOR(1),
        SHARED(2),
        LOW(3),
        MEDIUM(4),
        HIGH(5),
    }

    init {
        subcomponents.add(tipper)
        subcomponents.add(spool)
        subcomponents.add(intake)
    }

    fun limitTipped() = limitSwitch.isPressed

    fun setSpool(ticks: Int) = spool.goToPosition(ticks).runUntil { !spool.isBusy() || limitTipped() }

    fun goToPosition(position: Position): Command {
        // first check if we have to cross the intake ticks to get to our target
        val crossing = Arm.INTAKE_TICKS in spool.currentPosition..position.floorTicks

        // if we are crossing, we need to go to the intake position first
        return if (crossing) {
            SequentialCommand(
                true,
                intake.getDefaultCommand(),
                setSpool(Arm.INTAKE_TICKS),
                tipper.tilt(),
                setSpool(position.floorTicks)
            )
        } else {
            setSpool(position.floorTicks)
        }
    }

    // mannual drive, ability to tilt tipper and move arm
    fun drive(power: Double) {
        if (limitTipped()) spool.power = 0.0
        else spool.power = power
    }

    fun tilt() = tipper.toggleTip()
}