package org.firstinspires.ftc.teamcode.opmodes

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.RobotOpMode
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.geometry.Vector2d
import com.amarcolini.joos.kinematics.DiffSwerveKinematics
import com.amarcolini.joos.util.rad
import com.amarcolini.joos.util.wrap
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Drive.FAST_SPEED
import org.firstinspires.ftc.teamcode.Drive.SLOW_SPEED
import org.firstinspires.ftc.teamcode.MainRobot
import org.firstinspires.ftc.teamcode.components.arm.Tipper
import org.firstinspires.ftc.teamcode.util.telemetry.RobotTelemetry
import kotlin.math.PI

@TeleOp(name = "Main", group = "Main")
class MainTeleOp : RobotOpMode<MainRobot>() {
    override fun preInit() {
        initialize<MainRobot>()

        RobotTelemetry.addTelemetry("left target" to {
            val (leftTarget) = DiffSwerveKinematics.robotToModuleOrientations(drivePose, 1.0)
            leftTarget.radians.wrap(-PI / 2, PI / 2)
        }, "right target" to {
            val (_, rightTarget) = DiffSwerveKinematics.robotToModuleOrientations(drivePose, 1.0)
            rightTarget.radians.wrap(-PI / 2, PI / 2)
        })
    }

    private val drivePose: Pose2d
        get() {
            val (leftStickX, leftStickY) = robot.gamepad.p1.getLeftStick()
            val rightStickX = robot.gamepad.p1.getRightStick().x
            var driveVector = Vector2d(leftStickX, leftStickY)

            val slowMode = robot.gamepad.p1.left_trigger.isActive

            robot.imu?.let {
                driveVector = driveVector.rotated(it.heading)
            }

            val drivePose = Pose2d(driveVector, rightStickX.rad * 2.0)

            return drivePose * if (slowMode) SLOW_SPEED else FAST_SPEED
        }

    override fun preStart() {
        val drive = robot.drive
        val arm = robot.arm

        val driver =
            Command.of {
                drive.setDrivePower(drivePose)
            }.onEnd { drive.setDrivePower(Pose2d(0.0, 0.0, 0.0)) }
                .runUntil { false }

        // tipper
        robot.map({ robot.gamepad.p2.square.justActivated }, arm.tipper.setPosition(Tipper.TipperPosition.UNTIPPED))
        robot.map({ robot.gamepad.p2.triangle.justActivated }, arm.tipper.setPosition(Tipper.TipperPosition.TILT))
        robot.map({ robot.gamepad.p2.cross.justActivated }, arm.tipper.setPosition(Tipper.TipperPosition.TIPPED))

        // intake
        robot.map({ robot.gamepad.p2.right_bumper.isActive }, arm.intake.goForwards)
        robot.map({ robot.gamepad.p2.left_bumper.isActive }, arm.intake.goBackwards)

        // if they are both off then lets set speed to 0
        robot.map(
            { !robot.gamepad.p2.right_bumper.isActive && !robot.gamepad.p2.left_bumper.isActive },
            arm.intake.getDefaultCommand()
        )

        val armer = Command.of {
            val armPower = robot.gamepad.p2.getLeftStick().y
            arm.drive(armPower * 0.3)
        }.runUntil { false }

        robot.schedule(driver, armer)
    }
}