package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.control.FeedforwardCoefficients
import com.amarcolini.joos.control.PIDCoefficients
import com.amarcolini.joos.trajectory.config.GenericConstraints
import org.firstinspires.ftc.teamcode.util.Inch

// all distances in M
const val ULTRAPLANETARY_TICKS = 28.0
const val ULTRAPLANETARY_MAX_RPM = 6000.0

// A spins in the direction of the wheel, B in the opposite direction
const val DRIVE_LEFT_A_NAME = "drive-left-a"
const val DRIVE_LEFT_B_NAME = "drive-left-b"
const val DRIVE_RIGHT_A_NAME = "drive-right-a"
const val DRIVE_RIGHT_B_NAME = "drive-right-b"

object Module {
    const val GEAR_RATIO = 1.0 / 30.0
    const val TICKS_PER_REV = 100.0
    const val TRACK_WIDTH: Inch = 16.0
    const val WHEEL_RADIUS: Inch = 2.0
}

object Coefficients {
    val MODULE_PID = PIDCoefficients(4.0, 0.0, 0.1)
    val FEEDFORWARD_COEFFICIENTS = FeedforwardCoefficients()
    val TRAJECTORY_CONSTRAINTS = GenericConstraints()
    val TRANSLATIONAL_PID = PIDCoefficients(1.0, 0.0, 0.5)
    val HEADING_PID = PIDCoefficients(1.0, 0.0, 0.5)
}