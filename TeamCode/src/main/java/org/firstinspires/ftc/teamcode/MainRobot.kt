package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.Robot
import com.amarcolini.joos.command.RobotOpMode
import com.amarcolini.joos.control.FeedforwardCoefficients
import com.amarcolini.joos.control.PIDCoefficients
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.geometry.Vector2d
import com.amarcolini.joos.hardware.Imu
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.trajectory.config.GenericConstraints
import org.firstinspires.ftc.teamcode.Constants.DRIVE_LEFT_A_NAME
import org.firstinspires.ftc.teamcode.Constants.DRIVE_LEFT_B_NAME
import org.firstinspires.ftc.teamcode.Constants.DRIVE_RIGHT_A_NAME
import org.firstinspires.ftc.teamcode.Constants.DRIVE_RIGHT_B_NAME
import org.firstinspires.ftc.teamcode.Constants.ULTRAPLANETARY_MAX_RPM
import org.firstinspires.ftc.teamcode.Constants.ULTRAPLANETARY_TICKS
import org.firstinspires.ftc.teamcode.components.drive.DifferentialSwerveDrive


class MainRobot(opMode: RobotOpMode<MainRobot>, private val mode: Mode) : Robot(opMode) {

    // declare your motors and sensors here. CRS, Servos, DC, Drivetrain
    private lateinit var drive: DifferentialSwerveDrive
    private lateinit var imu: Imu

    enum class Mode {
        TeleOpMain, Auto
    }

    /**
     * [init] runs when the robot is in init.
     * use it to reset servo positions, make sure swerves are aligned, calibrate sensors
     * make sure to add telemetry too
     */
    override fun init() {
        imu = Imu(hMap, "imu");

        imu.autoDetectUpAxis()

        drive = DifferentialSwerveDrive(
            Motor(hMap, DRIVE_LEFT_A_NAME, ULTRAPLANETARY_MAX_RPM, ULTRAPLANETARY_TICKS),
            Motor(hMap, DRIVE_LEFT_B_NAME, ULTRAPLANETARY_MAX_RPM, ULTRAPLANETARY_TICKS),
            Motor(hMap, DRIVE_RIGHT_A_NAME, ULTRAPLANETARY_MAX_RPM, ULTRAPLANETARY_TICKS),
            Motor(hMap, DRIVE_RIGHT_B_NAME, ULTRAPLANETARY_MAX_RPM, ULTRAPLANETARY_TICKS),
            imu,
            modulePID = PIDCoefficients(4.0),
            feedforwardCoefficients = FeedforwardCoefficients(),
            constraints = GenericConstraints(),
            translationalPID = PIDCoefficients(1.0),
            headingPID = PIDCoefficients(1.0)
        )

        // reset arms whatever
    }

    /**
     * This runs whenever the robot starts. It is automatically called by the teleOp
     */
    override fun start() {
        when (mode) {
            Mode.TeleOpMain -> {
                schedule(
                    Command.of {
                        val leftStick = gamepad.p1.getLeftStick()
                        val rightStick = gamepad.p1.getRightStick()

                        telemetry.addData(leftStick.toString(), "Left Stick")
                        telemetry.addData(rightStick.toString(), "Right Stick")

                        drive.setDrivePower(Pose2d(leftStick.x, leftStick.y, rightStick.x))
                    }.requires(drive).runUntil(false)
                )
            }
            Mode.Auto -> {

                // When building trajectories. You want to ensure continuous (think rational functions)
                // So you want to do one of two things.
                // 1. Split into separate commands
                // 2. just use splines and it'll automatically make cool curves

                val firstPath = drive.followTrajectory(
                    drive.trajectoryBuilder(Pose2d(0.0, 0.0, 5.0))
                        .splineTo(Vector2d(0.0, 20.0), 90.0)
                        .lineTo(Vector2d(20.0, 20.0))
                        .splineTo(Vector2d(40.0, 2.0), 20.0)
                        .build()
                )

                schedule(firstPath)
            }
        }
    }
}