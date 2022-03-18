package org.firstinspires.ftc.teamcode.components.drive

import com.amarcolini.joos.drive.Drive
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.kinematics.Kinematics
import com.amarcolini.joos.localization.Localizer
import kotlin.math.cos
import kotlin.math.sin

class DifferentialSwerveLocalizer(
    private val wheelPositions: () -> Pair<Double, Double>,
    private val wheelVelocities: () -> Pair<Double, Double>? = { null },
    private val moduleOrientations: () -> Pair<Double, Double>,
    private val trackWidth: Double,
    private val drive: Drive,
    private val useExternalHeading: Boolean = true
) : Localizer {
    private var _poseEstimate: Pose2d = Pose2d()
    override var poseEstimate: Pose2d
        get() = _poseEstimate
        set(value) {
            lastWheelPositions = null
            lastExtHeading = null
            if (useExternalHeading) drive.externalHeading = value.heading
            _poseEstimate = value
        }

    private var lastWheelPositions: Pair<Double, Double>? = null
    private var lastExtHeading: Double? = null

    override var poseVelocity: Pose2d? = null
        private set

    override fun update() {
        val wheelPositions = wheelPositions()
        val moduleOrientations = moduleOrientations()
        val externalHeading = if (useExternalHeading) drive.externalHeading else null
        lastWheelPositions?.let { lastWheelPositions ->
            val leftDelta = wheelPositions.first - lastWheelPositions.first
            val rightDelta = wheelPositions.second - lastWheelPositions.second
            val robotPoseDelta = Pose2d(
                (leftDelta * cos(moduleOrientations.first) + rightDelta * cos(moduleOrientations.second)) * 0.5,
                (leftDelta * sin(moduleOrientations.first) + rightDelta * sin(moduleOrientations.second)) * 0.5,
                lastExtHeading?.let { externalHeading?.minus(it) }
                    ?: ((rightDelta * sin(moduleOrientations.second) - leftDelta * sin(moduleOrientations.first)) / trackWidth)
            )
            _poseEstimate = Kinematics.relativeOdometryUpdate(
                _poseEstimate, robotPoseDelta
            )
        }

        val wheelVelocities = wheelVelocities()
        val extHeadingVel = if (useExternalHeading) drive.getExternalHeadingVelocity() else null
        poseVelocity =
            if (wheelVelocities != null)
                Pose2d(
                    (wheelVelocities.first * cos(moduleOrientations.first) + wheelVelocities.second * cos(
                        moduleOrientations.second
                    )) * 0.5,
                    (wheelVelocities.first * sin(moduleOrientations.first) + wheelVelocities.second * sin(
                        moduleOrientations.second
                    )) * 0.5,
                    extHeadingVel
                        ?: ((wheelVelocities.second * sin(moduleOrientations.second) - wheelVelocities.first * sin(
                            moduleOrientations.first
                        )) / trackWidth)
                )
            else null

        lastWheelPositions = wheelPositions
        lastExtHeading = externalHeading
    }
}