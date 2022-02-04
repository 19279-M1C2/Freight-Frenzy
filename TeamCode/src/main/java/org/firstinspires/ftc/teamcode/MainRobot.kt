package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.Robot
import com.amarcolini.joos.command.RobotOpMode
import com.amarcolini.joos.hardware.Servo


class MainRobot(opMode: RobotOpMode) : Robot(opMode) {

    private val bucket = Bucket(Servo(hMap, "2", 2.0, 2.0))

    override fun init() {
        TODO("Not yet implemented")
    }

    override fun start() {
        if (isInTeleOp) {
            drive()
        }
    }

    private fun drive() {

        if (isInAutonomous)
            schedule(
                Command.of {
                    val stick = gamepad.p1.getLeftStick()

                }.runUntil(false)
            )

        map({ gamepad.p1.b.justActivated }, bucket.setPos(gamepad.p1.getLeftStick().x).requires(bucket))


    }
}
