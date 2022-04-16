package org.firstinspires.ftc.teamcode.components.drive
//
//import com.amarcolini.joos.control.FeedforwardCoefficients
//import com.amarcolini.joos.control.PIDCoefficients
//import com.amarcolini.joos.control.PIDFController
//import com.amarcolini.joos.drive.DriveSignal
//import com.amarcolini.joos.followers.HolonomicPIDVAFollower
//import com.amarcolini.joos.geometry.Pose2d
//import com.amarcolini.joos.geometry.Vector2d
//import com.amarcolini.joos.hardware.Imu
//import com.amarcolini.joos.hardware.Motor
//import com.amarcolini.joos.hardware.drive.DriveComponent
//import com.amarcolini.joos.localization.Localizer
//import com.amarcolini.joos.trajectory.config.TrajectoryConstraints
//import com.amarcolini.joos.util.wrap
//import org.firstinspires.ftc.teamcode.Constants.Coefficients.FEEDFORWARD_COEFFICIENTS
//import org.firstinspires.ftc.teamcode.Constants.Coefficients.HEADING_PID
//import org.firstinspires.ftc.teamcode.Constants.Coefficients.MODULE_PID
//import org.firstinspires.ftc.teamcode.Constants.Coefficients.TRAJECTORY_CONSTRAINTS
//import org.firstinspires.ftc.teamcode.Constants.Coefficients.TRANSLATIONAL_PID
//import org.firstinspires.ftc.teamcode.Constants.Module.GEAR_RATIO
//import org.firstinspires.ftc.teamcode.Constants.Module.TICKS_PER_REV
//import org.firstinspires.ftc.teamcode.Constants.Module.TRACK_WIDTH
//import org.firstinspires.ftc.teamcode.Constants.Module.WHEEL_RADIUS
//import kotlin.math.PI
//import kotlin.math.abs
//
///**
// * Differential swerve controller
// * @param trackWidth the distance between the two modules
// * @param translationalPID translation controller for the trajectory follower
// * @param headingPID heading controller for trajectory follower
// * @param modulePID PID for the swerve modules themselves
// * @param feedforwardCoefficients the feedforward coeffs for the motors
// * @param constraints The constraints for the drive
// * @param gearRatio The gear ratio from one of the motors to the wheel shaft
// * @param ticksPerRev The ticks per rev of each module
// * @param wheelRadius The radius of each wheel
// */
//class DifferentialSwerveDrive(
//    private val leftMotorA: Motor, private val leftMotorB: Motor,
//    private val rightMotorA: Motor, private val rightMotorB: Motor,
//    override val imu: Imu? = null,
//
//    // All the drive base constants
//    private val trackWidth: Double = TRACK_WIDTH,
//    private val gearRatio: Double = GEAR_RATIO,
//    private val ticksPerRev: Double = TICKS_PER_REV,
//    private val wheelRadius: Double = WHEEL_RADIUS,
//
//    // Motor Tuning
//    modulePID: PIDCoefficients = MODULE_PID,
//    feedforwardCoefficients: FeedforwardCoefficients = FEEDFORWARD_COEFFICIENTS,
//
//    // All the things to do with trajectory following
//    override val constraints: TrajectoryConstraints = TRAJECTORY_CONSTRAINTS,
//    translationalPID: PIDCoefficients = TRANSLATIONAL_PID,
//    headingPID: PIDCoefficients = HEADING_PID,
//) : DriveComponent() {
//    /*
//     * Note that a motor A corresponds to the motor that spins the same direction as the wheel,
//     * and motor B spins the opposite direction.
//    */
//
//    override var localizer: Localizer = DifferentialSwerveLocalizer(
//        ::getWheelPositions, ::getWheelVelocities, ::getModuleOrientations,
//        trackWidth, this, imu != null
//    )
//
//    override val trajectoryFollower = HolonomicPIDVAFollower(
//        translationalPID, translationalPID,
//        headingPID,
//        Pose2d(0.5, 0.5, Math.toRadians(5.0)),
//        0.5
//    )
//
//    override val rawExternalHeading: Double = imu?.heading ?: 0.0
//
//    private val leftModuleController = PIDFController(modulePID)
//    private val rightModuleController = PIDFController(modulePID)
//
//    private val motors = listOf(leftMotorA, leftMotorB, rightMotorA, rightMotorB)
//
//    init {
//        leftModuleController.setInputBounds(-0.5 * PI, 0.5 * PI)
//        rightModuleController.setInputBounds(-0.5 * PI, 0.5 * PI)
////        leftModuleController.setInputBounds(-PI, PI)
////        rightModuleController.setInputBounds(-PI, PI)
//
//        motors.forEach {
//            it.runMode = Motor.RunMode.RUN_WITHOUT_ENCODER
//            it.distancePerRev = 2 * PI * wheelRadius * gearRatio
//            it.feedforwardCoefficients = feedforwardCoefficients
//            it.resetEncoder()
//        }
//    }
//
//    private var leftVel = 0.0
//    private var leftAccel = 0.0
//    private var rightVel = 0.0
//    private var rightAccel = 0.0
//
//    private fun getWheelPositions(): Pair<Double, Double> =
//        (leftMotorA.distance - leftMotorB.distance) * 0.5 to (rightMotorA.distance - rightMotorB.distance) * 0.5
//
//    private fun getWheelVelocities(): Pair<Double, Double> =
//        (leftMotorA.distanceVelocity - leftMotorB.distanceVelocity) * 0.5 to (rightMotorA.distanceVelocity - rightMotorB.distanceVelocity) * 0.5
//
//    fun getModuleOrientations(): Pair<Double, Double> =
//        (leftMotorA.currentPosition + leftMotorB.currentPosition) / ticksPerRev * PI to (rightMotorA.currentPosition + rightMotorB.currentPosition) / ticksPerRev * PI
//
//    fun getTargetModuleOrientations(): Pair<Double, Double> =
//        leftModuleController.targetPosition to rightModuleController.targetPosition
//
////    private fun getDirectionalModuleOrientations(): Pair<Double, Double> {
////        val (left, right) = getModuleOrientations()
////        val (sameLeft, sameRight) = isSameHalf
////
////        val directionalLeft = if (sameLeft) left else left + PI
////        val directionalRight = if (sameRight) right else right + PI
////
////        return directionalLeft to directionalRight
////    }
//
//
//    override fun setDrivePower(drivePower: Pose2d) {
//        val leftVector = Vector2d(drivePower.x, drivePower.y + drivePower.heading)
//        val rightVector = Vector2d(drivePower.x, drivePower.y - drivePower.heading)
//
//        leftModuleController.setTarget(leftVector.angle())
//        rightModuleController.setTarget(rightVector.angle())
//
//        leftVel = leftVector.norm() * leftMotorA.maxRPM
//        leftAccel = 0.0
//        rightVel = rightVector.norm() * rightMotorA.maxRPM
//        rightAccel = 0.0
//    }
//
//    override fun update() {
//        super.update()
//
//        val moduleOrientations = getModuleOrientations()
//
//        setSpeed(leftMotorA, leftMotorB, leftModuleController, moduleOrientations.first, leftVel, leftAccel)
//        setSpeed(rightMotorA, rightMotorB, rightModuleController, moduleOrientations.second, rightVel, rightAccel)
//
//        motors.forEach { it.update() }
//    }
//
//    private fun setSpeed(
//        motorA: Motor,
//        motorB: Motor,
//        controller: PIDFController,
//        moduleOrientation: Double,
//        vel: Double,
//        accel: Double
//    ) {
//        val sameHalf = isSameHalf(moduleOrientation, controller.targetPosition)
//        val (v, a) = calculateDirection(sameHalf, vel, accel)
//        motorA.setSpeed(v + controller.update(moduleOrientation), a)
//        motorB.setSpeed(-v + controller.update(moduleOrientation), -a)
//    }
//
//    /**
//     * This functions inverts the direction of a speed based on [sameHalf]
//     */
//    private fun calculateDirection(
//        sameHalf: Boolean,
//        vel: Double,
//        accel: Double
//    ): Pair<Double, Double> {
//
//        return if (sameHalf) {
//            Pair(vel, accel)
//        } else {
//            Pair(-vel, -accel)
//        }
//    }
//
//    /**
//     * Checks if two angles are within half pi radians of each other
//     */
//    private fun isSameHalf(first: Double, second: Double) =
//        (abs(first.wrap(-PI, PI) - second.wrap(-PI, PI)) <= 0.5 * PI)
//
//
//    override fun setDriveSignal(driveSignal: DriveSignal) {
//        val leftVelVector = Vector2d(driveSignal.vel.x, driveSignal.vel.y + driveSignal.vel.heading)
//        val rightVelVector = Vector2d(driveSignal.vel.x, driveSignal.vel.y - driveSignal.vel.heading)
//
//        val leftAccelVector = Vector2d(driveSignal.accel.x, driveSignal.accel.y + driveSignal.accel.heading)
//        val rightAccelVector = Vector2d(driveSignal.accel.x, driveSignal.accel.y - driveSignal.accel.heading)
//
//        leftModuleController.setTarget(leftVelVector.angle())
//        rightModuleController.setTarget(rightVelVector.angle())
//
//        val divisor = (2 * PI * wheelRadius * gearRatio)
//        leftVel = leftVelVector.norm() / divisor
//        leftAccel = leftAccelVector.norm() / divisor
//        rightVel = rightVelVector.norm() / divisor
//        rightAccel = rightAccelVector.norm() / divisor
//    }
//
//    override fun setRunMode(runMode: Motor.RunMode) {
//        motors.forEach {
//            it.runMode = runMode
//        }
//    }
//
//    override fun setZeroPowerBehavior(zeroPowerBehavior: Motor.ZeroPowerBehavior) {
//        motors.forEach {
//            it.zeroPowerBehavior = zeroPowerBehavior
//        }
//    }
//}
//
//
