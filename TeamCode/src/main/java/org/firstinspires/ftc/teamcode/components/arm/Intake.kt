package org.firstinspires.ftc.teamcode.components.arm

import com.acmerobotics.dashboard.config.Config
import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.command.BasicCommand
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.FunctionalCommand
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.util.NanoClock
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.Motors.CORE_HEX_TPR
import org.firstinspires.ftc.teamcode.Motors.CORE_HEX_RPM
import org.firstinspires.ftc.teamcode.util.telemetry.RobotTelemetry

class Intake(private val motorEx: DcMotorEx, rpm: Double = CORE_HEX_RPM, tpr: Double = CORE_HEX_TPR) :
    AbstractComponent() {
    @Config(value = "Intake")
    companion object {
        var NAME = "intake"
        var REVERSED = false

        var CURRENT_THRESHOLD = 100
        var POWER = 0.5
    }

    private val motor: Motor = Motor(motorEx, rpm, tpr)

    init {
        subcomponents.add(motor)
        motor.zeroPowerBehavior = Motor.ZeroPowerBehavior.FLOAT
        if (REVERSED) motor.reversed()

        RobotTelemetry.addTelemetry("current") {
            motorEx.getCurrent(CurrentUnit.MILLIAMPS)
        }
    }


    override fun getDefaultCommand(): Command {
        return BasicCommand { motor.power = 0.0 }
    }

    fun setPower(power: Double) {
        motor.power = power
    }

    val goForwards = Command.of { setPower(0.8) }
    val goBackwards = Command.of { setPower(-0.8) }

    fun cargoInside() = motorEx.getCurrent(CurrentUnit.MILLIAMPS) > CURRENT_THRESHOLD

    fun intake(): Command {
        var startTime: Double? = null

        return FunctionalCommand(
            init = { setPower(POWER) },
            execute = {
                if (cargoInside()) {
                    startTime = NanoClock.system().seconds()
                }
            },
            isFinished = { startTime?.let { it + 1 < NanoClock.system().seconds() } ?: false },
            end = { setPower(0.0) },
            isInterruptable = true,
            requirements = setOf(this)
        )
    }
}