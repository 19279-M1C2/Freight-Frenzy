package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.control.PIDCoefficients
import com.amarcolini.joos.control.PIDFController
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.hardware.Imu
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.trajectory.config.SwerveConstraints

class DiffSwerve(
    private val leftFrontMotor: Motor,
    private val leftBackMotor: Motor,
    private val rightFrontMotor: Motor,
    private val rightBackMotor: Motor,
    val imu: Imu? = null,
    val constraints: SwerveConstraints = SwerveConstraints(
        listOf(
            leftBackMotor, leftFrontMotor, rightBackMotor, rightFrontMotor
        ).minOf { it.maxRPM }),
    translationalPIDEff: PIDCoefficients = PIDCoefficients(1.0, 0.0, 0.5),
    headingPIDEff: PIDCoefficients = PIDCoefficients(1.0, 0.0, 0.5)
) {
    //) {
    private val motors = listOf(leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor)
    private val leftModule = Pair(leftFrontMotor, leftBackMotor)
    private val rightModule = Pair(rightFrontMotor, rightBackMotor)
    private val modules = listOf(leftModule, rightModule)

    private val translationalPID = PIDFController(translationalPIDEff)
    private val headingPID = Pair(PIDFController(headingPIDEff), PIDFController(headingPIDEff))

    fun setDrivePower(power: Pose2d) {
        headingPID.toList().forEach {
            it.targetPosition = power.heading
        }

        val headingAdjust = getModuleOrientations().zip(headingPID.toList()).map { (curr, control) ->
            control.update(curr)
        }

        translationalPID.targetPosition = 0.0 // or whatever direction we want to turn

        val translationAdjust = translationalPID.update(getEncoderDrift())

        // figure out what these PIDs output, if they are negatives between 0.5 and -0.5 then normalize them or whatever.
        // looks like we can bind the output?

        modules.zip(headingAdjust.toList()).forEach { (module, heading) ->
            val (f, s) = module
            // get propper powers here
            // we need it so that translation adjust is different for each module
            f.power = power.x * translationAdjust * (0.5 + heading)
            s.power = power.x * translationAdjust * (-0.5 + heading)
        }
    }


    fun setRunMode(runMode: Motor.RunMode) {
        motors.forEach { it.runMode = runMode }
    }

    fun setZeroPowerBehavior(zeroPowerBehavior: Motor.ZeroPowerBehavior) {
        motors.forEach { it.zeroPowerBehavior = zeroPowerBehavior }
    }

    private fun getWheelPositions(): List<Double> = motors.map { it.distance }

    private fun getWheelVelocities(): List<Double> = motors.map { it.distanceVelocity }

    private fun getModuleOrientations(): List<Double> = modules.map { (f, s) -> (f.distance - s.distance) / 2 }
    private fun getModulePositions(): List<Double> = modules.map { (it.first.distance + it.second.distance) / 2 }

    private fun getEncoderDrift() = getModulePositions()[0] - getModulePositions()[1]

    fun update() {
        motors.forEach { it.update() }

    }
}