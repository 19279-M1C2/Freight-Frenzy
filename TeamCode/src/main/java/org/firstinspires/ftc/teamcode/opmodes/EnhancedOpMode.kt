package org.firstinspires.ftc.teamcode.opmodes

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.RobotOpMode
import com.amarcolini.joos.gamepad.MultipleGamepad
import org.firstinspires.ftc.teamcode.MainRobot

abstract class EnhancedOpMode : RobotOpMode<MainRobot>() {
    fun schedule(vararg command: Command) = robot.schedule(*command)

    lateinit var gamepad: MultipleGamepad

    final override fun init() {
        initialize(MainRobot(this))
        gamepad = robot.gamepad
    }

    abstract fun startCommands()
    abstract fun initCommands()
}