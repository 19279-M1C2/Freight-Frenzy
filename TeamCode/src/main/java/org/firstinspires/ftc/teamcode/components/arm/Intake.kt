package org.firstinspires.ftc.teamcode.components.arm

import com.acmerobotics.dashboard.config.Config
import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.command.BasicCommand
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.FunctionalCommand
import com.amarcolini.joos.hardware.Motor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.Constants.CORE_HEX_RPM
import org.firstinspires.ftc.teamcode.Constants.CORE_HEX_TPR

class Intake(private val motorEx: DcMotorEx, rpm: Double = CORE_HEX_RPM, tpr: Double = CORE_HEX_TPR) :
    AbstractComponent() {
    @Config
    object Intake {
        @JvmField
        var NAME = "intake"

        @JvmField
        var CURRENT_THRESHOLD = 100

        @JvmField
        var POWER = 0.5
    }

    private val motor: Motor = Motor(motorEx, rpm, tpr)

    init {
        subcomponents.add(motor)
        motor.zeroPowerBehavior = Motor.ZeroPowerBehavior.FLOAT
    }


    override fun getDefaultCommand(): Command {
        return BasicCommand { motor.power = 0.0 }
    }

    fun setPower(power: Double) {
        motor.power = power
    }

    fun cargoInside() = motorEx.getCurrent(CurrentUnit.MILLIAMPS) > Intake.CURRENT_THRESHOLD

    fun intake(): Command = FunctionalCommand(
        init = { setPower(Intake.POWER) },
        isFinished = { cargoInside() },
        end = { setPower(0.0) }, isInterruptable = true
    )
}