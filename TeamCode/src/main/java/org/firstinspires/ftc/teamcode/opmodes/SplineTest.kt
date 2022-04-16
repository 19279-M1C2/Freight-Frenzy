package org.firstinspires.ftc.teamcode.opmodes

import com.amarcolini.joos.command.RobotOpMode
import com.amarcolini.joos.command.SequentialCommand
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.MainRobot
import org.firstinspires.ftc.teamcode.util.Inch


@TeleOp(name = "SplineTest", group = "Tuning")
class SplineTest : RobotOpMode<MainRobot>() {

    companion object {
        const val DISTANCE: Inch = 50.0
    }

    override fun preStart() {
        val drive = robot.drive
        val forth = drive.trajectoryBuilder(Pose2d()).splineTo(Vector2d(30.0, 30.0), 0.0).build()
        val back = drive.trajectoryBuilder(forth.end()).splineTo(Vector2d(0.0, 0.0), 180.0).build()


        robot.schedule(
            SequentialCommand(
                false,
                drive.followTrajectory(forth),
                drive.followTrajectory(back)
            )
        )

    }

    override fun preInit() {
        initialize<MainRobot>()
    }
}