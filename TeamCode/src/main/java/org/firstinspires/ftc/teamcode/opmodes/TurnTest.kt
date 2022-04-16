package org.firstinspires.ftc.teamcode.opmodes

import com.amarcolini.joos.command.RobotOpMode
import com.amarcolini.joos.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.MainRobot
import org.firstinspires.ftc.teamcode.util.Degree


@TeleOp(name = "TurnTest", group = "Tuning")
class TurnTest : RobotOpMode<MainRobot>() {

    companion object {
        const val ANGLE: Degree = 90.0
    }

    override fun preStart() {
        val drive = robot.drive
        val trajectory = drive.trajectoryBuilder(Pose2d()).turn(ANGLE).build()

        val follow = drive.followTrajectory(trajectory)

        robot.schedule(follow)
    }

    override fun preInit() {
        initialize<MainRobot>()
    }
}