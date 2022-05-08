package org.firstinspires.ftc.teamcode.opmodes

import com.amarcolini.joos.command.RobotOpMode
import com.amarcolini.joos.command.SequentialCommand
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.util.deg
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.MainRobot
import org.firstinspires.ftc.teamcode.components.Vision
import org.firstinspires.ftc.teamcode.components.arm.Arm
import org.firstinspires.ftc.teamcode.util.toLevel

@Autonomous(name = "Freight Side", group = "Auto")
class FreightSideAuto : RobotOpMode<MainRobot>() {
    private lateinit var vision: Vision


    override fun preInit() {
        initialize<MainRobot>()

        robot.schedule(robot.arm.tipper.setPosition(0.0))
        vision = Vision(hardwareMap)
    }

    override fun preStart() {
        // TODO figure out timing here. Do we do with math? Do we hardcode cycles?

        val intakeCommand = robot.arm.intake.intake()
        val (loc, coord) = vision.getPosition()

        val level = loc.toLevel()
        robot.tele.addLine("Height: $level")
        robot.tele.addLine("Coord: $coord")

        val dropCommand =
            SequentialCommand(
                true,
                robot.arm.goToPosition(level),
                robot.arm.tipper.tilt(),
                robot.arm.tipper.untip(),
                robot.arm.goToPosition(Arm.Position.FLOOR)
            )

        val hubPoint = Pose2d(-10.90, 40.50, (-90).deg)
        val startPoint = Pose2d(13.0, 65.0, (-90).deg)
        val depotPoint = Pose2d(47.0, 63.87, (-180).deg)

        robot.drive.localizer.poseEstimate = startPoint

        val initToHub = robot.drive.trajectoryBuilder(startPoint, (-137).deg)
            .splineToLinearHeading(hubPoint, (-122).deg)
            .build()

        val toDepot = robot.drive.trajectoryBuilder(hubPoint, 93.deg)
            .splineToLinearHeading(Pose2d(-6.11, 63.62, (-180).deg), 14.deg)
            .splineToLinearHeading(depotPoint, 356.deg)
            .build()

        val toHub = robot.drive.trajectoryBuilder(depotPoint, 356.deg)
            .splineToLinearHeading(Pose2d(-6.11, 63.62, (-180).deg), 14.deg)
            .splineToLinearHeading(hubPoint, (-122).deg)
            .build()


    }
}