package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.amarcolini.joos.control.PIDCoefficients
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.trajectory.config.DiffSwerveConstraints
import org.firstinspires.ftc.teamcode.util.Inch

@JoosConfig
object Constants {
    // stick any constants that aren't associated with a specific subsystem here

    // all distances in M

    @JvmField
    var ULTRAPLANETARY_TICKS = 28.0

    @JvmField
    var ULTRAPLANETARY_MAX_RPM = 6000.0

    // A spins in the direction of the wheel, B in the opposite direction
    @JvmField
    var DRIVE_LEFT_A_NAME = "drive-left-a"

    @JvmField
    var DRIVE_LEFT_B_NAME = "drive-left-b"

    @JvmField
    var DRIVE_RIGHT_A_NAME = "drive-right-a"

    @JvmField
    var DRIVE_RIGHT_B_NAME = "drive-right-b"


    @JvmField
    var CORE_HEX_RPM = 150.0

    @JvmField
    var CORE_HEX_TPR = 1120.0

    @JvmField
    var SERVO_RPM = 150.0

    @Config
    object Module {
        // From Crown Rev to drive wheel
        @JvmField
        var GEAR_RATIO = 4.0

        // From motor to module rev
        @JvmField
        var TICKS_PER_REV = 921.0

        @JvmField
        var TRACK_WIDTH: Inch = 16.0

        @JvmField
        var WHEEL_RADIUS: Inch = 2.5 / 2
    }

    @Config
    object Coefficients {
        @JvmField
        var MODULE_PID = PIDCoefficients(50.0, 1.0, 2.0)

        @JvmField
        var TRAJECTORY_CONSTRAINTS = DiffSwerveConstraints(trackWidth = Module.TRACK_WIDTH)

        @JvmField
        var TRANSLATIONAL_PID = PIDCoefficients(1.0, 0.0, 0.5)

        @JvmField
        var HEADING_PID = PIDCoefficients(1.0, 0.0, 0.5)
    }
}