package org.firstinspires.ftc.teamcode.opmodes

import com.amarcolini.joos.command.*
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.util.NanoClock
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.internal.system.Misc
import org.firstinspires.ftc.teamcode.Drive.GEAR_RATIO
import org.firstinspires.ftc.teamcode.Drive.WHEEL_RADIUS
import org.firstinspires.ftc.teamcode.MainRobot
import org.firstinspires.ftc.teamcode.util.Inch
import org.firstinspires.ftc.teamcode.util.LoggingUtil.getLogFile
import org.firstinspires.ftc.teamcode.util.RegressionUtil
import org.firstinspires.ftc.teamcode.util.RegressionUtil.fitAccelData
import org.firstinspires.ftc.teamcode.util.RegressionUtil.fitRampData
import org.firstinspires.ftc.teamcode.util.telemetry.RobotTelemetry
import kotlin.math.sqrt


@TeleOp(name = "Automatic Feedforward Tunner", group = "Tuning")
class AutomaticFeedforwardTunner : RobotOpMode<MainRobot>() {
    companion object {
        const val MAX_POWER = 0.7
        const val DISTANCE: Inch = 100.0
    }


    override fun preStart() {
        var fitIntercept = false

        val maxVel: Double = WHEEL_RADIUS * GEAR_RATIO
        val finalVel = MAX_POWER * maxVel
        val accel = finalVel * finalVel / (2.0 * DISTANCE)
        val rampTime = sqrt(2.0 * DISTANCE / accel)
        val timeSamples: MutableList<Double> = mutableListOf()
        val positionSamples: MutableList<Double> = mutableListOf()
        val powerSamples: MutableList<Double> = mutableListOf()
        val drive = robot.drive
        val clock = NanoClock.system()
        var startTime = clock.seconds()
        var rampResult: RegressionUtil.RampResult = RegressionUtil.RampResult(0.0, 0.0, 0.0)

        val kVTune = SequentialCommand(
            false,
            FunctionalCommand(
                init = {
                    RobotTelemetry.clearAll()
                    RobotTelemetry.addLine("Would you like to fit kStatic?")
                    RobotTelemetry.addLine("Press (Y/Δ) for yes, (B/O) for no")
                },
                isFinished = { gamepad1.y || gamepad1.b },
                end = { if (gamepad1.y) fitIntercept = true }),
            FunctionalCommand(
                init = {
                    RobotTelemetry.clearAll()
                    RobotTelemetry.addLine(
                        Misc.formatInvariant(
                            "Place your robot on the field with at least %.2f in of room in front", DISTANCE
                        )
                    )
                    RobotTelemetry.addLine("Press (Y/Δ) to begin")
                },
                isFinished = { gamepad1.y },
                end = {
                    RobotTelemetry.clearAll()
                    RobotTelemetry.addLine("Running...")
                }),
            FunctionalCommand(
                init = { startTime = clock.seconds() },
                execute = {
                    val elapsedTime: Double = clock.seconds() - time
                    val vel = accel * elapsedTime
                    val power = vel / maxVel

                    timeSamples.add(elapsedTime)
                    positionSamples.add(drive.poseEstimate.x)
                    powerSamples.add(power)

                    drive.setDrivePower(Pose2d(power, 0.0, 0.0))
                    drive.updatePoseEstimate()
                },
                isFinished = {
                    val elapsedTime: Double = clock.seconds() - time
                    elapsedTime > rampTime
                }, end = {
                    drive.setDrivePower(Pose2d(0.0, 0.0, 0.0))
                    rampResult = fitRampData(
                        timeSamples, positionSamples, powerSamples, fitIntercept,
                        getLogFile(
                            Misc.formatInvariant(
                                "DriveRampRegression-%d.csv", System.currentTimeMillis()
                            )
                        )
                    )
//
//                    RobotTelemetry.clearAll()
//                    RobotTelemetry.addLine("Quasi-static ramp up test complete")
//
//                    val (kV, kStatic, rSquare) = rampResult
//
//                    if (fitIntercept) {
//                        RobotTelemetry.addLine(
//                            Misc.formatInvariant(
//                                "kV = %.5f, kStatic = %.5f (R^2 = %.2f)",
//                                kV, kStatic, rSquare
//                            )
//                        )
//                    } else {
//                        RobotTelemetry.addLine(
//                            Misc.formatInvariant(
//                                "kV = %.5f (R^2 = %.2f)",
//                                kStatic, rSquare
//                            )
//                        )
//                    }
                }
            )
        )

        var fitAccelFF = false

        val kATune = SequentialCommand(
            false,
            FunctionalCommand(
                init = {
                    RobotTelemetry.addLine("Would you like to fit kA?")
                    RobotTelemetry.addLine("Press (Y/Δ) for yes, (B/O) for no")

                },
                isFinished = { gamepad1.y || gamepad1.b },
                end = { if (gamepad1.y) fitAccelFF = true }),

            Command.select {
                val maxPowerTime = DISTANCE / maxVel

                return@select if (!fitAccelFF) BasicCommand()
                else SequentialCommand(
                    false,
                    FunctionalCommand(
                        init = {
                            RobotTelemetry.clearAll()
                            RobotTelemetry.addLine("Place the robot back in its starting position")
                            RobotTelemetry.addLine("Press (Y/Δ) to continue")
                        }, isFinished = { gamepad1.y },
                        end = {
                            RobotTelemetry.clearAll()
                            RobotTelemetry.addLine("Running...")
                        }),

                    FunctionalCommand(
                        init = {
                            timeSamples.clear()
                            positionSamples.clear()
                            powerSamples.clear()

                            drive.poseEstimate = Pose2d()
                            drive.setDrivePower(Pose2d(MAX_POWER, 0.0, 0.0))

                            startTime = clock.seconds()
                        },
                        execute = {
                            val elapsedTime = clock.seconds() - startTime

                            timeSamples.add(elapsedTime)
                            positionSamples.add(drive.poseEstimate.x)
                            powerSamples.add(MAX_POWER)

                            drive.updatePoseEstimate()
                        },
                        isFinished = {
                            val elapsedTime = clock.seconds() - startTime
                            elapsedTime > maxPowerTime
                        }, end = {
                            drive.setDrivePower(Pose2d(0.0, 0.0, 0.0))

                            val (kA, rSquare) = fitAccelData(
                                timeSamples, positionSamples, powerSamples, rampResult,
                                getLogFile(
                                    Misc.formatInvariant(
                                        "DriveAccelRegression-%d.csv", System.currentTimeMillis()
                                    )
                                )
                            )

                            RobotTelemetry.clearAll()
                            RobotTelemetry.addLine("Constant power test complete")
                            RobotTelemetry.addLine(
                                Misc.formatInvariant(
                                    "kA = %.5f (R^2 = %.2f)",
                                    kA, rSquare
                                )
                            )
                        }
                    )
                )
            }
        )

        robot.schedule(SequentialCommand(false, kVTune, kATune))
    }

    override fun preInit() {
        initialize<MainRobot>()
    }


}