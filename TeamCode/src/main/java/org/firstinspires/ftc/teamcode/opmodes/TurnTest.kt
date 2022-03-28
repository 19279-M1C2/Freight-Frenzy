package org.firstinspires.ftc.teamcode.opmodes

import com.amarcolini.joos.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.MainRobot
import org.firstinspires.ftc.teamcode.util.Degree


@TeleOp(name = "TurnTest", group = "Tuning")
class TurnTest : EnhancedOpMode<MainRobot>() {

    companion object {
        const val ANGLE: Degree = 90.0
    }

    override fun init() {
        initialize(MainRobot(this))
    }

    override fun startSchedule() {
        val drive = robot.drive
        val trajectory = drive.trajectoryBuilder(Pose2d()).turn(ANGLE).build()

        val follow = drive.followTrajectory(trajectory)

        schedule(follow)
    }

    override fun initSchedule() {

    }


}