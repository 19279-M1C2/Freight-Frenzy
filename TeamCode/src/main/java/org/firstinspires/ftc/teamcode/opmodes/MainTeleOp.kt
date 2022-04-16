package org.firstinspires.ftc.teamcode.opmodes

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.RobotOpMode
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.util.rad
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.MainRobot

@TeleOp(name = "Main", group = "Main")
class MainTeleOp : RobotOpMode<MainRobot>() {
    override fun preInit() {
        initialize<MainRobot>()
    }

    override fun preStart() {
        val drive = robot.drive

        val driver =
            Command.of {
                val (leftStickX, leftStickY) = robot.gamepad.p1.getLeftStick()
                val rightStickX = robot.gamepad.p1.getRightStick().x

                drive.setDrivePower(Pose2d(-leftStickX, leftStickY, rightStickX.rad) * 0.45)
            }.requires(drive).onEnd { drive.setDrivePower(Pose2d(0.0, 0.0, 0.0)) }
                .runUntil { robot.gamepad.p1.right_bumper.isActive }

        robot.schedule(driver)
    }
}