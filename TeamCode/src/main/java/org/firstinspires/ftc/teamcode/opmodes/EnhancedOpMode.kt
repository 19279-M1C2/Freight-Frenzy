package org.firstinspires.ftc.teamcode.opmodes

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.Robot
import com.amarcolini.joos.command.RobotOpMode

abstract class EnhancedOpMode<T : Robot> : RobotOpMode<T>() {
    fun schedule(vararg command: Command) = robot.schedule(*command)

    abstract fun startSchedule()
    abstract fun initSchedule()
}