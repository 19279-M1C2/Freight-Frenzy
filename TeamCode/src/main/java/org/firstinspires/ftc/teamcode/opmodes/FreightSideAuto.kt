package org.firstinspires.ftc.teamcode.opmodes

import com.amarcolini.joos.command.RobotOpMode
import com.amarcolini.joos.command.SequentialCommand
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


    }
}