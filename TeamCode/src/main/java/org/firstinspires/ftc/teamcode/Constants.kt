package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.control.FeedforwardCoefficients
import com.amarcolini.joos.control.PIDCoefficients
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.trajectory.config.DiffSwerveConstraints
import org.firstinspires.ftc.teamcode.util.Inch
import kotlin.math.PI

@JoosConfig
object Constants {
    // stick any constants that aren't associated with a specific subsystem here

    // all distances in M

    var ULTRAPLANETARY_TICKS = 28.0
    var ULTRAPLANETARY_MAX_RPM = 6000.0

    // A spins in the direction of the wheel, B in the opposite direction
    var DRIVE_LEFT_A_NAME = "drive-left-a"
    var DRIVE_LEFT_B_NAME = "drive-left-b"
    var DRIVE_RIGHT_A_NAME = "drive-right-a"
    var DRIVE_RIGHT_B_NAME = "drive-right-b"

    var CORE_HEX_RPM = 150.0
    var CORE_HEX_TPR = 1120.0

    var SERVO_RPM = 150.0

    @JoosConfig
    object Module {
        // From Crown Rev to drive wheel
        var GEAR_RATIO = 4.0

        // From motor to module rev
        var TICKS_PER_REV = 835.0

        var TRACK_WIDTH: Inch = 16.0
        var WHEEL_RADIUS: Inch = 2.5 / 2

        var SLOW_SPEED = 0.2
        var FAST_SPEED = 0.45
    }

    @JoosConfig
    object Coefficients {
        var MODULE_PID = PIDCoefficients(125.0, 0.0, 0.0)

        var TRAJECTORY_CONSTRAINTS = DiffSwerveConstraints(trackWidth = Module.TRACK_WIDTH)
        var TRANSLATIONAL_PID = PIDCoefficients(1.0, 0.0, 0.5)
        var HEADING_PID = PIDCoefficients(1.0, 0.0, 0.5)

        var FEED_FORWARD_COEFFICIENTS = FeedforwardCoefficients(
            1 / (ULTRAPLANETARY_MAX_RPM / 60 * Module.GEAR_RATIO * Module.WHEEL_RADIUS * PI),
            0.0,
            0.0
        )
    }
}