package org.firstinspires.ftc.teamcode.opmodes

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "Main", group = "Main")
class MainTeleOp : EnhancedOpMode() {

    override fun startCommands() {
        val drive = robot.drive

        val driver =
            Command.of {

                val (leftStickX, leftStickY) = gamepad.p1.getLeftStick()
                val rightStickX = gamepad.p1.getRightStick().x

                drive.setDrivePower(Pose2d(-leftStickX, leftStickY, rightStickX) * 0.45)
            }.requires(drive).onEnd { drive.setDrivePower(Pose2d(0.0, 0.0, 0.0)) }.runUntil { gamepad.p1.right_bumper.isActive }

        schedule(driver)
    }

    override fun initCommands() {

    }


}