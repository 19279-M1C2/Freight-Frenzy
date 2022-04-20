package org.firstinspires.ftc.teamcode.components.arm

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.hardware.Motor

class Spool(private val motor: Motor) : AbstractComponent() {
    fun setPower(power: Double) {
        motor.power = power
    }


}