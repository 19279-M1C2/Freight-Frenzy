package org.firstinspires.ftc.teamcode.opmodes

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.RobotOpMode
import com.amarcolini.joos.command.SuperTelemetry
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.kinematics.DiffSwerveKinematics
import com.amarcolini.joos.util.rad
import com.amarcolini.joos.util.wrap
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Constants.Module.TRACK_WIDTH
import org.firstinspires.ftc.teamcode.MainRobot
import kotlin.math.PI

@TeleOp(name = "Main", group = "Main")
class MainTeleOp : RobotOpMode<MainRobot>() {
    private lateinit var robotTargetTele: Pair<SuperTelemetry.Item, SuperTelemetry.Item>

    override fun preInit() {
        initialize<MainRobot>()
        robotTargetTele =
            robot.tele.addData("Left Target", 0.0) to
                    robot.tele.addData("Right Target", 0.0)

        robotTargetTele.first.isRetained = true
        robotTargetTele.second.isRetained = true
    }

    override fun preStart() {
        val drive = robot.drive

        val driver =
            Command.of {
                val (leftStickX, leftStickY) = robot.gamepad.p1.getLeftStick()
                val rightStickX = robot.gamepad.p1.getRightStick().x
                val drivePose = Pose2d(leftStickX, leftStickY, rightStickX.rad) * 0.45

                // update some telemetry
                val (leftTarget, rightTarget) = DiffSwerveKinematics.robotToModuleOrientations(drivePose, TRACK_WIDTH)
                robotTargetTele.first.setValue(leftTarget.radians.wrap(-PI / 2, PI / 2))
                robotTargetTele.second.setValue(rightTarget.radians.wrap(-PI / 2, PI / 2))

                drive.setDrivePower(drivePose)
            }.requires(drive).onEnd { drive.setDrivePower(Pose2d(0.0, 0.0, 0.0)) }
                .runUntil { false }

        robot.schedule(driver)
    }
}