package org.firstinspires.ftc.teamcode.components.drive

import com.amarcolini.joos.control.FeedforwardCoefficients
import com.amarcolini.joos.control.PIDCoefficients
import com.amarcolini.joos.control.PIDFController
import com.amarcolini.joos.drive.DriveSignal
import com.amarcolini.joos.followers.HolonomicPIDVAFollower
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.geometry.Vector2d
import com.amarcolini.joos.hardware.Imu
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.hardware.drive.DriveComponent
import com.amarcolini.joos.localization.Localizer
import com.amarcolini.joos.trajectory.config.TrajectoryConstraints
import org.firstinspires.ftc.teamcode.Coefficients
import org.firstinspires.ftc.teamcode.Module
import kotlin.math.PI

/**
 * Differential swerve controller
 * @param trackWidth the distance between the two modules
 * @param translationalPID translation controller for the trajectory follower
 * @param headingPID heading controller for trajectory follower
 * @param modulePID PID for the swerve modules themselves
 * @param feedforwardCoefficients the feedforward coeffs for the motors
 * @param constraints The constraints for the drive
 * @param gearRatio The gear ratio from one of the motors to the wheel shaft
 * @param ticksPerRev The tickets per rev of each module
 * @param wheelRadius The radius of each wheel
 */
class DifferentialSwerveDrive(
    private val leftMotorA: Motor, private val leftMotorB: Motor,
    private val rightMotorA: Motor, private val rightMotorB: Motor,
    override val imu: Imu? = null,

    // All the drive base constants
    private val trackWidth: Double = Module.TRACK_WIDTH,
    private val gearRatio: Double = Module.GEAR_RATIO,
    private val ticksPerRev: Double = Module.TICKS_PER_REV,
    private val wheelRadius: Double = Module.WHEEL_RADIUS,

    // Motor Tuning
    modulePID: PIDCoefficients = Coefficients.MODULE_PID,
    feedforwardCoefficients: FeedforwardCoefficients = Coefficients.FEEDFORWARD_COEFFICIENTS,

    // All the things to do with trajectory following
    override val constraints: TrajectoryConstraints = Coefficients.TRAJECTORY_CONSTRAINTS,
    translationalPID: PIDCoefficients = Coefficients.TRANSLATIONAL_PID,
    headingPID: PIDCoefficients = Coefficients.HEADING_PID,
) : DriveComponent() {
    /*
     * Note that a motor A corresponds to the motor that spins the same direction as the wheel,
     * and motor B spins the opposite direction.
    */

    override var localizer: Localizer = DifferentialSwerveLocalizer(
        ::getWheelPositions, ::getWheelVelocities, ::getModuleOrientations,
        trackWidth, this, imu != null
    )

    override val trajectoryFollower = HolonomicPIDVAFollower(
        translationalPID, translationalPID,
        headingPID,
        Pose2d(0.5, 0.5, Math.toRadians(5.0)),
        0.5
    )

    override val rawExternalHeading: Double = imu?.heading ?: 0.0

    private val leftModuleController = PIDFController(modulePID)
    private val rightModuleController = PIDFController(modulePID)

    private val motors = listOf(leftMotorA, leftMotorB, rightMotorA, rightMotorB)

    init {
        leftModuleController.setInputBounds(-PI, PI)
        rightModuleController.setInputBounds(-PI, PI)

        motors.forEach {
            it.distancePerRev = 2 * PI * wheelRadius * gearRatio
            it.feedforwardCoefficients = feedforwardCoefficients
        }
    }

    private var leftVel = 0.0
    private var leftAccel = 0.0
    private var rightVel = 0.0
    private var rightAccel = 0.0

    private fun getWheelPositions(): Pair<Double, Double> =
        (leftMotorA.distance - leftMotorB.distance) * 0.5 to (rightMotorA.distance - rightMotorB.distance) * 0.5

    private fun getWheelVelocities(): Pair<Double, Double> =
        (leftMotorA.distanceVelocity - leftMotorB.distanceVelocity) * 0.5 to (rightMotorA.distanceVelocity - rightMotorB.distanceVelocity) * 0.5

    private fun getModuleOrientations(): Pair<Double, Double> =
        (leftMotorA.currentPosition + leftMotorB.currentPosition) / ticksPerRev * PI to (rightMotorA.currentPosition + rightMotorB.currentPosition) / ticksPerRev * PI

    override fun setDrivePower(drivePower: Pose2d) {
        val leftVector = Vector2d(drivePower.x, drivePower.y + drivePower.heading)
        val rightVector = Vector2d(drivePower.x, drivePower.y - drivePower.heading)

        leftModuleController.setTarget(leftVector.angle())
        rightModuleController.setTarget(rightVector.angle())

        leftVel = leftVector.norm() * leftMotorA.maxRPM
        leftAccel = 0.0
        rightVel = rightVector.norm() * rightMotorA.maxRPM
        rightAccel = 0.0
    }

    override fun update() {
        super.update()
        val moduleOrientations = getModuleOrientations()
        val leftControl = leftModuleController.update(
            moduleOrientations.first
        )
        leftMotorA.setSpeed(leftVel + leftControl, leftAccel)
        leftMotorB.setSpeed(-leftVel + leftControl, -leftAccel)

        val rightControl = rightModuleController.update(
            moduleOrientations.second
        )
        rightMotorA.setSpeed(rightVel + rightControl, rightAccel)
        rightMotorB.setSpeed(-rightVel + rightControl, -rightAccel)

        motors.forEach { it.update() }
    }

    override fun setDriveSignal(driveSignal: DriveSignal) {
        val leftVelVector = Vector2d(driveSignal.vel.x, driveSignal.vel.y + driveSignal.vel.heading)
        val rightVelVector = Vector2d(driveSignal.vel.x, driveSignal.vel.y - driveSignal.vel.heading)

        val leftAccelVector = Vector2d(driveSignal.accel.x, driveSignal.accel.y + driveSignal.accel.heading)
        val rightAccelVector = Vector2d(driveSignal.accel.x, driveSignal.accel.y - driveSignal.accel.heading)

        leftModuleController.setTarget(leftVelVector.angle())
        rightModuleController.setTarget(rightVelVector.angle())

        val divisor = (2 * PI * wheelRadius * gearRatio)
        leftVel = leftVelVector.norm() / divisor
        leftAccel = leftAccelVector.norm() / divisor
        rightVel = rightVelVector.norm() / divisor
        rightAccel = rightAccelVector.norm() / divisor
    }

    override fun setRunMode(runMode: Motor.RunMode) {
        motors.forEach {
            it.runMode = runMode
        }
    }

    override fun setZeroPowerBehavior(zeroPowerBehavior: Motor.ZeroPowerBehavior) {
        motors.forEach {
            it.zeroPowerBehavior = zeroPowerBehavior
        }
    }
}


