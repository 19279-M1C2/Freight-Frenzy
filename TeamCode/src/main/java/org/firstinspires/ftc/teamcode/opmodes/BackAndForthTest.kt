package org.firstinspires.ftc.teamcode.opmodes

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.SequentialCommand
import com.amarcolini.joos.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.util.Inch


@TeleOp(name = "BackAndForthTest", group = "Tuning")
class BackAndForthTest : EnhancedOpMode() {

    companion object {
        const val DISTANCE: Inch = 50.0
    }

    override fun startCommands() {
        val drive = robot.drive
        val forth = drive.trajectoryBuilder(Pose2d()).forward(DISTANCE).build()
        val back = drive.trajectoryBuilder(forth.end()).back(DISTANCE).build()
        schedule(Command.of {
            schedule(
                SequentialCommand(
                    false,
                    drive.followTrajectory(forth),
                    drive.followTrajectory(back)
                )

            )
        }.runUntil { false }.requires(drive))
    }

    override fun initCommands() {

    }


}