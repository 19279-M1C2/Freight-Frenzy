package org.firstinspires.ftc.teamcode.opmodes

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.RobotOpMode
import com.amarcolini.joos.command.SequentialCommand
import com.amarcolini.joos.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.MainRobot
import org.firstinspires.ftc.teamcode.util.Inch


@TeleOp(name = "BackAndForthTest", group = "Tuning")
class BackAndForthTest : RobotOpMode<MainRobot>() {

    companion object {
        const val DISTANCE: Inch = 50.0
    }

    override fun preStart() {
        val drive = robot.drive
        val forth = drive.trajectoryBuilder(Pose2d()).forward(DISTANCE).build()
        val back = drive.trajectoryBuilder(forth.end()).back(DISTANCE).build()
        robot.schedule(Command.of {
            robot.schedule(
                SequentialCommand(
                    false,
                    drive.followTrajectory(forth),
                    drive.followTrajectory(back)
                )

            )
        }.runUntil { false }.requires(drive))
    }

    override fun preInit() {
        initialize<MainRobot>()
    }


}