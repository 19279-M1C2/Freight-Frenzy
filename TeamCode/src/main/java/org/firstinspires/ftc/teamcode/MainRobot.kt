package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.FunctionalCommand
import com.amarcolini.joos.command.Robot
import com.amarcolini.joos.hardware.Imu
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.util.wrap
import org.firstinspires.ftc.teamcode.Constants.DRIVE_LEFT_A_NAME
import org.firstinspires.ftc.teamcode.Constants.DRIVE_LEFT_B_NAME
import org.firstinspires.ftc.teamcode.Constants.ULTRAPLANETARY_MAX_RPM
import org.firstinspires.ftc.teamcode.Constants.ULTRAPLANETARY_TICKS
import org.firstinspires.ftc.teamcode.components.DummyMotor
import org.firstinspires.ftc.teamcode.components.drive.DifferentialSwerveDrive
import org.firstinspires.ftc.teamcode.opmodes.EnhancedOpMode
import org.firstinspires.ftc.teamcode.util.TelemetryUpdater
import kotlin.math.PI


class MainRobot(val opMode: EnhancedOpMode) : Robot(opMode) {

    private val teleValues = mutableListOf<TelemetryUpdater>()

    // declare your motors and sensors here. CRS, Servos, DC, Drivetrain
    lateinit var drive: DifferentialSwerveDrive
    private var imu: Imu? = null

    private fun initDrive() {
        initImu()
        val driveLeftA = Motor(hMap, DRIVE_LEFT_A_NAME, ULTRAPLANETARY_MAX_RPM, ULTRAPLANETARY_TICKS)
        val driveLeftB = Motor(hMap, DRIVE_LEFT_B_NAME, ULTRAPLANETARY_MAX_RPM, ULTRAPLANETARY_TICKS)

        driveLeftB.reversed()

        teleValues.addAll(
            listOf(
                TelemetryUpdater("Drive Left A", telemetry) { driveLeftA.currentPosition },
                TelemetryUpdater("Drive Left B", telemetry) { driveLeftB.currentPosition }
            )
        )
//        val driveRightA = Motor(hMap, DRIVE_RIGHT_A_NAME, ULTRAPLANETARY_MAX_RPM, ULTRAPLANETARY_TICKS)
//        val driveRightB = Motor(hMap, DRIVE_RIGHT_B_NAME, ULTRAPLANETARY_MAX_RPM, ULTRAPLANETARY_TICKS)
        val driveRightA = Motor(
            DummyMotor(ULTRAPLANETARY_MAX_RPM, ULTRAPLANETARY_TICKS),
            ULTRAPLANETARY_MAX_RPM,
            ULTRAPLANETARY_TICKS
        )
        val driveRightB = Motor(
            DummyMotor(ULTRAPLANETARY_MAX_RPM, ULTRAPLANETARY_TICKS),
            ULTRAPLANETARY_MAX_RPM,
            ULTRAPLANETARY_TICKS
        )

        driveRightB.reversed()

        drive = DifferentialSwerveDrive(
            driveLeftA,
            driveLeftB,
            driveRightA,
            driveRightB,
            null, // TODO make this a real IMU when mounted
        )
        teleValues.addAll(
            listOf(TelemetryUpdater(
                "left",
                telemetry
            ) { ((drive.getModuleOrientations().first).wrap(-0.5 * PI, 0.5 * PI)) },
                TelemetryUpdater(
                    "right",
                    telemetry
                ) { ((drive.getModuleOrientations().second).wrap(-0.5 * PI, 0.5 * PI)) },
                TelemetryUpdater(
                    "target left",
                    telemetry
                ) { (drive.getTargetModuleOrientations().first).wrap(-0.5 * PI, 0.5 * PI) },
                TelemetryUpdater(
                    "target right",
                    telemetry
                ) { (drive.getTargetModuleOrientations().second).wrap(-0.5 * PI, 0.5 * PI) }
            ))

        register(drive)
    }

    private fun initImu() {
        imu = Imu(hMap, "imu")

        // If we have an IMU, let's set up the correct axis to be front incase we have it mounted vertically
        // TODO: Ensure that the IMU is mounted on a flat surface, sideways is fine just not diagonally
        val res = imu?.autoDetectUpAxis()

        if (res != null) {
            telemetry.addLine("imu: up axis: $res")
        } else {
            telemetry.addLine("imu: up axis not detected")

            // void the imu so that we aren't using the wrong axis
            imu = null
        }
    }

    /**
     * [init] runs when the robot is in init.
     * use it to reset servo positions, make sure swerves are aligned, calibrate sensors
     * make sure to add telemetry too
     */
    override fun init() {
        // we have a few options here. We can init the subsystems in the opmodes to seperate out responsiblities
        // we can also just init them here based on what is installed on the robot
        // for now im going with the first option
        initDrive()

        // reset arms whatever
        opMode.initCommands()
        schedule(telemetryUpdate)
    }

    private val telemetryUpdate = FunctionalCommand(
        execute = {
            teleValues.forEach { it.update() }
        },
        isFinished = { false }
    )

    /**
     * This runs whenever the robot starts. It is automatically called by the teleOp
     */
    override fun start() {
        opMode.startCommands()
    }


//        when (mode) {
//            Mode.TeleOpMain -> {
//                val driver =
//                    Command.of {
//                        val leftStick = gamepad.p1.getLeftStick()
//                        val rightStick = gamepad.p1.getRightStick()
//
//                        drive.setDrivePower(Pose2d(leftStick.x, leftStick.y, rightStick.x))
//                    }.requires(drive)
//                        .onEnd { drive.setDrivePower(Pose2d(0.0, 0.0, 0.0)) }
//                        .runUntil(false)
//
//                schedule(driver)
//            }
//            Mode.Auto -> {
//
//                // When building trajectories. You want to ensure continuity (think rational functions)
//                // So you want to do one of two things.
//                // 1. Split into separate commands
//                // 2. just use splines and it'll automatically make cool curves
//
//                val firstPath = drive.followTrajectory(
//                    drive.trajectoryBuilder(Pose2d(0.0, 0.0, 5.0))
//                        .splineTo(Vector2d(0.0, 20.0), 90.0)
//                        .lineTo(Vector2d(20.0, 20.0))
//                        .splineTo(Vector2d(40.0, 2.0), 20.0)
//                        .build()
//                )
//
//                schedule(firstPath)
//            }
//        }
}
