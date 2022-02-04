package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.Robot
import com.amarcolini.joos.command.RobotOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "TeleOp", group = "Main")
class TeleOp : RobotOpMode() {

    private lateinit var robot: Robot;

    // this is all you need to do for an op mode.
    override fun init() {
        // make sure to set it as the right TeleOp
        initialize(MainRobot(this, MainRobot.OpMode.TeleOp))
    }
}

