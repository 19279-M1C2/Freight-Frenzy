package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.RobotOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "MainTeleOp", group = "Main")
class MainTeleOp : RobotOpMode<MainRobot>() {
    // this is all you need to do for an op mode.
    override fun init() {
        // make sure to set it as the right TeleOp
        initialize(MainRobot(this))
    }
}