package org.firstinspires.ftc.teamcode

//import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.amarcolini.joos.command.Robot
import com.amarcolini.joos.command.RobotOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "HaveFun", group = "Two")
class TestOpMode : RobotOpMode() {

    private lateinit var robot: Robot;

    override fun init() {
        initialize(MainRobot(this))
    }





}

