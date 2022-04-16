package org.firstinspires.ftc.teamcode.components.drive

import com.amarcolini.joos.control.PIDCoefficients
import com.amarcolini.joos.hardware.Imu
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.hardware.drive.DiffSwerveDrive
import com.amarcolini.joos.trajectory.config.DiffSwerveConstraints

class DiffSwerveDriveTele(
    private val leftModule: Pair<Motor, Motor>,
    private val rightModule: Pair<Motor, Motor>,
    imu: Imu? = null,
    constraints: DiffSwerveConstraints = DiffSwerveConstraints(
        listOf(
            leftModule.first,
            leftModule.second,
            rightModule.first,
            rightModule.second
        ).minOf { it.maxDistanceVelocity }
    ),
    moduleOrientationPID: PIDCoefficients,
    translationalPID: PIDCoefficients = PIDCoefficients(4.0, 0.0, 0.5),
    headingPID: PIDCoefficients = PIDCoefficients(4.0, 0.0, 0.5)
) : DiffSwerveDrive(leftModule, rightModule, imu, constraints, moduleOrientationPID, translationalPID, headingPID) {

    init {
    }

}