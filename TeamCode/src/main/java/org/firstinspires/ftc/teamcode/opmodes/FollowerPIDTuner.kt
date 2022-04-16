package org.firstinspires.ftc.teamcode.opmodes

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.RobotOpMode
import com.amarcolini.joos.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.MainRobot
import org.firstinspires.ftc.teamcode.util.Inch


/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives in a DISTANCE-by-DISTANCE square indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin driving in a square.
 * You should observe the target position (green) and your pose estimate (blue) and adjust your
 * follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 */
@TeleOp(name = "FollowerPIDTuner", group = "Tuning")
class FollowerPIDTuner : RobotOpMode<MainRobot>() {

    companion object {
        const val DISTANCE: Inch = 48.0
    }

    override fun preStart() {
        val drive = robot.drive

        val startPose = Pose2d(-DISTANCE / 2, -DISTANCE / 2, 0.0)

        drive.poseEstimate = startPose

        robot.schedule(Command.of {
            robot.schedule(
                Command.select {
                    val trajSeq = drive.trajectoryBuilder(startPose)
                        .forward(DISTANCE)
                        .turn(Math.toRadians(90.0))
                        .forward(DISTANCE)
                        .turn(Math.toRadians(90.0))
                        .forward(DISTANCE)
                        .turn(Math.toRadians(90.0))
                        .forward(DISTANCE)
                        .turn(Math.toRadians(90.0))
                        .build()

                    drive.followTrajectory(trajSeq)
                }
            )
        }.runUntil { false }.requires(drive))
    }

    override fun preInit() {
        initialize<MainRobot>()
    }


}