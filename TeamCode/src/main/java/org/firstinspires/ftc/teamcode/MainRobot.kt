package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.Robot
import com.amarcolini.joos.command.RobotOpMode
import com.amarcolini.joos.hardware.Imu
import com.amarcolini.joos.hardware.Servo
import com.amarcolini.joos.hardware.drive.DiffSwerveDrive
import com.amarcolini.joos.kinematics.DiffSwerveKinematics
import com.amarcolini.joos.util.wrap
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.Coefficients.HEADING_PID
import org.firstinspires.ftc.teamcode.Coefficients.MODULE_PID
import org.firstinspires.ftc.teamcode.Coefficients.TRAJECTORY_CONSTRAINTS
import org.firstinspires.ftc.teamcode.Coefficients.TRANSLATIONAL_PID
import org.firstinspires.ftc.teamcode.Drive.DRIVE_LEFT_A_NAME
import org.firstinspires.ftc.teamcode.Drive.DRIVE_LEFT_B_NAME
import org.firstinspires.ftc.teamcode.Drive.DRIVE_RIGHT_A_NAME
import org.firstinspires.ftc.teamcode.Drive.DRIVE_RIGHT_B_NAME
import org.firstinspires.ftc.teamcode.Motors.CORE_HEX_RPM
import org.firstinspires.ftc.teamcode.Motors.CORE_HEX_TPR
import org.firstinspires.ftc.teamcode.Motors.motorFactory
import org.firstinspires.ftc.teamcode.components.arm.Arm
import org.firstinspires.ftc.teamcode.components.arm.Intake
import org.firstinspires.ftc.teamcode.components.arm.Tipper

import org.firstinspires.ftc.teamcode.components.arm.Tipper.Tipper.TIPPER_NAME
import org.firstinspires.ftc.teamcode.util.telemetry.RobotTelemetry
import kotlin.math.PI

class MainRobot(val opMode: RobotOpMode<MainRobot>) : Robot(opMode) {
    // declare your motors and sensors here. CRS, Servos, DC, Drivetrain
    lateinit var drive: DiffSwerveDrive
    var imu: Imu? = null
    lateinit var arm: Arm

    private fun initDrive() {
        initImu()
        val driveLeftA = motorFactory(Motors.MotorType.DRIVE, DRIVE_LEFT_A_NAME, hMap)
        val driveLeftB = motorFactory(Motors.MotorType.DRIVE, DRIVE_LEFT_B_NAME, hMap)
        val driveRightA = motorFactory(Motors.MotorType.DRIVE, DRIVE_RIGHT_A_NAME, hMap)
        val driveRightB = motorFactory(Motors.MotorType.DRIVE, DRIVE_RIGHT_B_NAME, hMap)

        drive = DiffSwerveDrive(
            driveLeftA to driveLeftB,
            driveRightA to driveRightB,
            imu,
            TRAJECTORY_CONSTRAINTS,
            MODULE_PID,
            TRANSLATIONAL_PID,
            HEADING_PID
        )

        RobotTelemetry.addTelemetry(
            "left" to {
                val (first, second) = drive.motors.motors.map { it.rotation }
                DiffSwerveKinematics.gearToModuleOrientation(first, second).radians.wrap(-PI / 2, PI / 2)
            }, "right" to {
                val (_, _, first, second) = drive.motors.motors.map { it.rotation }
                DiffSwerveKinematics.gearToModuleOrientation(first, second).radians.wrap(-PI / 2, PI / 2)
            })

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
        val spool = motorFactory(Motors.MotorType.HD_HEX_40, Arm.SPOOL_NAME, hMap)
        val intake = Intake(hMap.get(DcMotorEx::class.java, Intake.NAME), CORE_HEX_RPM, CORE_HEX_TPR)

        spool.resetEncoder()

        arm = Arm(
            spool,
            Tipper(Servo(hMap, TIPPER_NAME)),
            null,
//            hMap.get(TouchSensor::class.java, Arm.Arm.LIMIT_SWITCH_NAME),
            intake
        )

        register(arm)
    }

    /**
     * [init] runs when the robot is in init.
     * use it to reset servo positions, make sure swerves are aligned, calibrate sensors
     * make sure to add telemetry too
     */
    override fun init() {
        RobotTelemetry.telemetry = telemetry
        register(RobotTelemetry)
        // we have a few options here. We can init the subsystems in the opmodes to seperate out responsiblities
        // we can also just init them here based on what is installed on the robot
        // for now im going with the first option
        initDrive()
        initArm()
    }

    /**
     * This runs whenever the robot starts. It is automatically called by the teleOp
     */
    override fun start() {

    }
}
