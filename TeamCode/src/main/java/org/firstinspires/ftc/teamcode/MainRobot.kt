package org.firstinspires.ftc.teamcode

//import org.firstinspires.ftc.teamcode.components.drive.DifferentialSwerveDrive
import com.amarcolini.joos.command.FunctionalCommand
import com.amarcolini.joos.command.Robot
import com.amarcolini.joos.command.RobotOpMode
import com.amarcolini.joos.hardware.Imu
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.hardware.drive.DiffSwerveDrive
import com.amarcolini.joos.kinematics.DiffSwerveKinematics
import com.amarcolini.joos.trajectory.config.DiffSwerveConstraints
import com.amarcolini.joos.util.wrap
import org.firstinspires.ftc.teamcode.Constants.Coefficients.HEADING_PID
import org.firstinspires.ftc.teamcode.Constants.Coefficients.MODULE_PID
import org.firstinspires.ftc.teamcode.Constants.Coefficients.TRANSLATIONAL_PID
import org.firstinspires.ftc.teamcode.Constants.DRIVE_LEFT_A_NAME
import org.firstinspires.ftc.teamcode.Constants.DRIVE_LEFT_B_NAME
import org.firstinspires.ftc.teamcode.Constants.DRIVE_RIGHT_A_NAME
import org.firstinspires.ftc.teamcode.Constants.DRIVE_RIGHT_B_NAME
import org.firstinspires.ftc.teamcode.Constants.Module.GEAR_RATIO
import org.firstinspires.ftc.teamcode.Constants.Module.TICKS_PER_REV
import org.firstinspires.ftc.teamcode.Constants.Module.TRACK_WIDTH
import org.firstinspires.ftc.teamcode.Constants.Module.WHEEL_RADIUS
import org.firstinspires.ftc.teamcode.Constants.SPOOL_MOTOR_NAME
import org.firstinspires.ftc.teamcode.Constants.SPOOL_MOTOR_RPM
import org.firstinspires.ftc.teamcode.Constants.ULTRAPLANETARY_MAX_RPM
import org.firstinspires.ftc.teamcode.Constants.ULTRAPLANETARY_TICKS
import org.firstinspires.ftc.teamcode.components.DummyMotor
import org.firstinspires.ftc.teamcode.components.arm.Spool
import org.firstinspires.ftc.teamcode.util.TelemetryUpdater
import kotlin.math.PI

class MainRobot(val opMode: RobotOpMode<MainRobot>) : Robot(opMode) {

    val teleValues = mutableListOf<TelemetryUpdater>()
    val tele = telemetry

    // declare your motors and sensors here. CRS, Servos, DC, Drivetrain
    lateinit var drive: DiffSwerveDrive
    private var imu: Imu? = null
    lateinit var spool: Spool

    private fun driveMotorFactory(name: String): Motor =
        Motor(hMap, name, ULTRAPLANETARY_MAX_RPM, TICKS_PER_REV, WHEEL_RADIUS, GEAR_RATIO)


    private fun motorFactory(name: String): Motor = Motor(hMap, name, ULTRAPLANETARY_MAX_RPM, ULTRAPLANETARY_TICKS)

    private fun dummyDriveMotorFactory() = Motor(
        DummyMotor(ULTRAPLANETARY_MAX_RPM, ULTRAPLANETARY_TICKS),
        ULTRAPLANETARY_MAX_RPM,
        ULTRAPLANETARY_TICKS, WHEEL_RADIUS, GEAR_RATIO
    )

    private fun initDrive() {
        initImu()
        val driveLeftA = driveMotorFactory(DRIVE_LEFT_A_NAME)
        val driveLeftB = driveMotorFactory(DRIVE_LEFT_B_NAME)
        val driveRightA = driveMotorFactory(DRIVE_RIGHT_A_NAME)
        val driveRightB = driveMotorFactory(DRIVE_RIGHT_B_NAME)

        listOf(driveLeftA, driveLeftB, driveRightA, driveRightB).forEach {
            it.resetEncoder()
        }

        drive = DiffSwerveDrive(
            driveLeftA to driveLeftB,
            driveRightA to driveRightB,
            imu,
            DiffSwerveConstraints(trackWidth = TRACK_WIDTH),
            MODULE_PID,
            TRANSLATIONAL_PID,
            HEADING_PID
        )

        teleValues.addAll(
            listOf(
                TelemetryUpdater(
                    "left",
                    telemetry
                ) {
                    val (first, second) = drive.motors.motors.map { it.rotation }
                    DiffSwerveKinematics.gearToModuleOrientation(first, second).radians.wrap(-PI / 2, PI / 2)
                },
                TelemetryUpdater(
                    "right",
                    telemetry
                ) {
                    val (_, _, first, second) = drive.motors.motors.map { it.rotation }
                    DiffSwerveKinematics.gearToModuleOrientation(first, second).radians.wrap(-PI / 2, PI / 2)
                },
            )
        )
//
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

    private fun initArm() {
        val spoolMotor = Motor(hMap, SPOOL_MOTOR_NAME, SPOOL_MOTOR_RPM, 40 * ULTRAPLANETARY_TICKS)
        spool = Spool(spoolMotor)
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
//        initArm()

        // reset arms whatever

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
