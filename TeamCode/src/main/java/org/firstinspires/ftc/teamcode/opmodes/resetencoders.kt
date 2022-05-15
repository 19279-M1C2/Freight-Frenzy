package org.firstinspires.ftc.teamcode.opmodes

import com.amarcolini.joos.command.RobotOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.MainRobot
import org.firstinspires.ftc.teamcode.util.Degree


@TeleOp(name = "ResetEncoder", group = "Tuning")
class ResetEncoders : RobotOpMode<MainRobot>() {

    companion object {
        const val ANGLE: Degree = 90.0
    }

    override fun preStart() {

    }

    override fun preInit() {
        initialize<MainRobot>()

        robot.drive.motors.motors.forEach {
            it.resetEncoder()
        }
    }
}