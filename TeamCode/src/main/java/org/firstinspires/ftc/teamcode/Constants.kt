package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.amarcolini.joos.control.FeedforwardCoefficients
import com.amarcolini.joos.control.PIDCoefficients
import com.amarcolini.joos.trajectory.config.GenericConstraints
import org.firstinspires.ftc.teamcode.util.Inch

@Config
object Constants {
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

    @Config
    object Module {
        // From Motor to drive wheel
        @JvmField
        var GEAR_RATIO = (7.0 / 60.0)
        //* (5.0 / 6.0)

        // From motor to module rev
        @JvmField
        var TICKS_PER_REV = 880.0

        @JvmField
        var TRACK_WIDTH: Inch = 16.0

        @JvmField
        var WHEEL_RADIUS: Inch = 2.0
    }

    @Config
    object Coefficients {
        @JvmField
        var MODULE_PID = PIDCoefficients(800.0, 0.0, 50.0)

        @JvmField
        var FEEDFORWARD_COEFFICIENTS = FeedforwardCoefficients(0.00668450761)

        @JvmField
        var TRAJECTORY_CONSTRAINTS = GenericConstraints()

        @JvmField
        var TRANSLATIONAL_PID = PIDCoefficients(1.0, 0.0, 0.5)

        @JvmField
        var HEADING_PID = PIDCoefficients(1.0, 0.0, 0.5)
    }
}