package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.Robot
import com.amarcolini.joos.command.RobotOpMode
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.geometry.Vector2d
import com.amarcolini.joos.hardware.Imu
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.hardware.Servo
import org.firstinspires.ftc.teamcode.components.arm.*
import org.firstinspires.ftc.teamcode.components.drive.DifferentialSwerveDrive


class MainRobot(private val opMode: RobotOpMode<MainRobot>, val mode: Mode) : Robot(opMode) {

    // declare your motors and sensors here. CRS, Servos, DC, Drivetrain
    lateinit var folds: Folds
    lateinit var mount: Mount
    lateinit var arm: Arm
    lateinit var drive: DifferentialSwerveDrive

    enum class Mode {
        TeleOpMain, Auto
    }

    /**
     * [init] runs when the robot is in init.
     * use it to reset servo positions, make sure swerves are aligned, calibrate sensors
     * make sure to add telemetry too
     */
    override fun init() {
        folds = Folds(Servo(hMap, Constants.FIRST_FOLD_NAME), Servo(hMap, Constants.SECOND_FOLD_NAME))
        mount =
            Mount(
                MountSpin(
                    Motor(
                        hMap,
                        Constants.MOUNT_SPIN_NAME,
                        6000.0,
                        Constants.ULTRAPLANATRY_TICKS * Constants.MOUNT_SPIN_RATIO
                    )
                ),
                MountHeight(
                    Motor(
                        hMap,
                        Constants.MOUNT_HEIGHT_NAME,
                        6000.0,
                        Constants.ULTRAPLANATRY_TICKS * Constants.MOUNT_HEIGHT_RATIO
                    )
                )
            )

        arm = Arm(mount, folds)

        drive = DifferentialSwerveDrive(
            Motor(hMap, Constants.DRIVE_LEFT_A_NAME, 6000.0),
            Motor(hMap, Constants.DRIVE_LEFT_B_NAME, 6000.0),
            Motor(hMap, Constants.DRIVE_RIGHT_A_NAME, 6000.0),
            Motor(hMap, Constants.DRIVE_RIGHT_B_NAME, 6000.0),
            Constants.TRACK_WIDTH,
            Imu(hMap, "imu")
        )

        schedule(Command.of { mount.reset() }.requires(mount))
        schedule((folds.setSecondAngle(0.0) and folds.setFirstAngle(0.0)).requires(folds))
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
                schedule(
                    drive.followTrajectory(
                        drive.trajectoryBuilder(Pose2d(0.0, 0.0, 0.0))
                            .splineTo(Vector2d(0.0, 20.0), 90.0)
                            .lineTo(Vector2d(20.0, 20.0))
                            .splineTo(Vector2d(40.0, 2.0), 20.0)
                            .build()
                    )
                )
            }
        }
    }
}