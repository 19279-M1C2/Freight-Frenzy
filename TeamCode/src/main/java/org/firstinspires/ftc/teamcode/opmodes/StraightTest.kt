package org.firstinspires.ftc.teamcode.opmodes

import com.amarcolini.joos.command.BasicCommand
import com.amarcolini.joos.command.SequentialCommand
import com.amarcolini.joos.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.util.Inch


@TeleOp(name = "StraightTest", group = "Tuning")
class StraightTest : EnhancedOpMode() {

    companion object {
        const val DISTANCE: Inch = 60.0
    }

    override fun startCommands() {
        val drive = robot.drive
        val trajectory = drive.trajectoryBuilder(Pose2d()).forward(DISTANCE).build()

        val follow = drive.followTrajectory(trajectory)

        schedule(SequentialCommand(false, follow, BasicCommand {
            val poseEstimate: Pose2d = drive.poseEstimate
            telemetry.addData("finalX", poseEstimate.x)
            telemetry.addData("finalY", poseEstimate.y)
            telemetry.addData("finalHeading", poseEstimate.heading)
        }))
    }

    override fun initCommands() {

    }


}