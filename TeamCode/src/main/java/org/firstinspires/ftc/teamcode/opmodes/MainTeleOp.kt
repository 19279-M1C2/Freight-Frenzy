package org.firstinspires.ftc.teamcode.opmodes

import com.amarcolini.joos.command.RobotOpMode
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.MainRobot

@TeleOp(name = "MainTeleOp", group = "Main")
@Disabled
class MainTeleOp : RobotOpMode<MainRobot>() {
    // this is all you need to do for an op mode.
    override fun init() {
        // make sure to set it as the right TeleOp
        initialize(MainRobot(this, MainRobot.Mode.TeleOpMain))
    }
}