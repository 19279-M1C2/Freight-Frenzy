package org.firstinspires.ftc.teamcode.opmodes

import com.amarcolini.joos.command.BasicCommand
import com.amarcolini.joos.command.RobotOpMode
import com.amarcolini.joos.command.SequentialCommand
import com.amarcolini.joos.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.MainRobot
import org.firstinspires.ftc.teamcode.util.Inch


@TeleOp(name = "StraightTest", group = "Tuning")
class StraightTest : RobotOpMode<MainRobot>() {

    companion object {
        const val DISTANCE: Inch = 60.0
    }

    override fun preStart() {
        val drive = robot.drive
        val trajectory = drive.trajectoryBuilder(Pose2d()).forward(DISTANCE).build()

        val follow = drive.followTrajectory(trajectory)

        robot.schedule(SequentialCommand(false, follow, BasicCommand {
            val poseEstimate: Pose2d = drive.poseEstimate
            telemetry.addData("finalX", poseEstimate.x)
            telemetry.addData("finalY", poseEstimate.y)
            telemetry.addData("finalHeading", poseEstimate.heading)
        }))
    }

    override fun preInit() {
        initialize<MainRobot>()
    }
}