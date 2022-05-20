package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.amarcolini.joos.control.FeedforwardCoefficients
import com.amarcolini.joos.control.PIDCoefficients
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.trajectory.config.DiffSwerveConstraints
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.components.DummyMotor
import org.firstinspires.ftc.teamcode.util.Inch
import kotlin.math.PI


/**
 * stick any constants that aren't associated with a specific subsystem here
 * [Drive] is an exception as we don't actually have a drive class. same with [Coefficients]
 */
@Config
object Motors {
    var ULTRAPLANETARY_TICKS = 28.0
    var ULTRAPLANETARY_MAX_RPM = 6000.0

    var HD_HEX40_TICKS = ULTRAPLANETARY_TICKS * 40.0
    var HD_HEX40_MAX_RPM = 150.0

    var CORE_HEX_RPM = 150.0
    var CORE_HEX_TPR = 1120.0

    var SERVO_RPM = 150.0

    private fun driveMotorFactory(name: String, hMap: HardwareMap) =
        Motor(hMap, name, ULTRAPLANETARY_MAX_RPM, Drive.TICKS_PER_REV, Drive.WHEEL_RADIUS, Drive.GEAR_RATIO)

    private fun ultraMotorFactory(name: String, hMap: HardwareMap) =
        Motor(hMap, name, ULTRAPLANETARY_MAX_RPM, ULTRAPLANETARY_TICKS)

    fun dummyDriveMotorFactory() = Motor(
        DummyMotor(ULTRAPLANETARY_MAX_RPM, ULTRAPLANETARY_TICKS),
        ULTRAPLANETARY_MAX_RPM,
        ULTRAPLANETARY_TICKS, Drive.WHEEL_RADIUS, Drive.GEAR_RATIO
    )

    private fun coreHexMotorFactory(name: String, hMap: HardwareMap) =
        Motor(hMap, name, CORE_HEX_RPM, CORE_HEX_TPR)

    private fun hdHex40MotorFactory(name: String, hMap: HardwareMap) =
        Motor(hMap, name, HD_HEX40_MAX_RPM, HD_HEX40_TICKS)

    enum class MotorType {
        ULTRAPLANETARY,
        CORE_HEX,
        HD_HEX_40,
        DRIVE,
    }

    fun motorFactory(type: MotorType, name: String, hMap: HardwareMap) =
        when (type) {
            MotorType.ULTRAPLANETARY -> ultraMotorFactory(name, hMap)
            MotorType.CORE_HEX -> coreHexMotorFactory(name, hMap)
            MotorType.HD_HEX_40 -> hdHex40MotorFactory(name, hMap)
            MotorType.DRIVE -> driveMotorFactory(name, hMap)
        }
}

@Config
object Drive {
    // a drives in direction of robot
    var DRIVE_LEFT_A_NAME = "drive-left-a"
    var DRIVE_LEFT_B_NAME = "drive-left-b"
    var DRIVE_RIGHT_A_NAME = "drive-right-a"
    var DRIVE_RIGHT_B_NAME = "drive-right-b"

    var GEAR_RATIO = 4.0 // Crown to wheel
    var TICKS_PER_REV = 835.0 // Motor to crown

    var TRACK_WIDTH: Inch = 16.0
    var WHEEL_RADIUS: Inch = 2.5 / 2

    @JvmField
    var SLOW_SPEED = 0.5

    @JvmField
    var FAST_SPEED = 1.0
}

@Config
object Coefficients {
    @JvmField
    var MODULE_PID = PIDCoefficients(70.0, 0.0, 0.9)

    @JvmField
    var TRAJECTORY_CONSTRAINTS = DiffSwerveConstraints(
        trackWidth = 1.0,
        maxAccel = 5.0,
        maxVel = 5.0,
        maxGearVel = 100.0 * 0.3
    )

    @JvmField
    var TRANSLATIONAL_PID = PIDCoefficients(7.0, 0.0, 0.5)

    @JvmField
    var HEADING_PID = PIDCoefficients(10.0, 0.0, 0.5)

    @JvmField
    var FEED_FORWARD_COEFFICIENTS = FeedforwardCoefficients(
        1 / (Motors.ULTRAPLANETARY_MAX_RPM / 60 * Drive.GEAR_RATIO * Drive.WHEEL_RADIUS * PI),
        0.0,
        0.0
    )
}
