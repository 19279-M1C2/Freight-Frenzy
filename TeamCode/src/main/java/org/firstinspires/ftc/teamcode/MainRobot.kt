package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.Robot
import com.amarcolini.joos.command.RobotOpMode


class MainRobot(opMode: RobotOpMode, private val mode: OpMode) : Robot(opMode) {

    // we can create different op modes using this
    enum class OpMode {
        TeleOp,
        Auto,
    }

    // declare your motors and sensors here. CRS, Servos, DC, Drivetrain

    /**
     * [init] runs when the robot is in init.
     * use it to reset servo positions, make sure swerves are aligned, calibrate sensors
     * make sure to add telemetry too
     */
    override fun init() {

    }

    /**
     * This runs whenever the robot starts. It is automatically called by the teleOp
     */
    override fun start() {
        // this is used to tell if we are running auto or teleop or other Op modes.
        if (mode == OpMode.TeleOp) {
            // we could also just call other functions here and split code
            TODO("TeleOp")
        } else if (mode == OpMode.Auto) {
            TODO("Auto")
        }

    }
}