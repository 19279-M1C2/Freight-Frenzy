package org.firstinspires.ftc.teamcode.opmodes

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.MainRobot

@TeleOp(name = "MainTeleOp", group = "Main")
class MainTeleOp : EnhancedOpMode<MainRobot>() {
    // this is all you need to do for an op mode.
    override fun init() {
        // make sure to set it as the right TeleOp
        initialize(MainRobot(this))
    }

    override fun startSchedule() {
        val drive = robot.drive

        val driver =
            Command.of {
                val leftStickX = gamepad1.left_stick_x.toDouble()
                val leftStickY = gamepad1.left_stick_y.toDouble()
                val rightStickX = gamepad1.right_stick_x.toDouble()

                telemetry.addData("leftStickX", leftStickX)
                telemetry.addData("leftStickY", leftStickY)
                telemetry.addData("rightStickX", rightStickX)

                telemetry.addData("drive.left", drive.poseEstimate)
                telemetry.update()
                drive.setDrivePower(Pose2d(leftStickX, leftStickY, rightStickX))
                drive.update()
            }.requires(drive)
                .onEnd { drive.setDrivePower(Pose2d(0.0, 0.0, 0.0)) }
                .runUntil(false)

        schedule(driver)
    }

    override fun initSchedule() {

    }


}