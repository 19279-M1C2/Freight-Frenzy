package org.firstinspires.ftc.teamcode.components.arm

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.hardware.Servo

class Folds(private val firstJoint: Servo, private val secondJoint: Servo) : AbstractComponent() {
    init {
        subcomponents.add(firstJoint)
        subcomponents.add(secondJoint)
    }

    // ideally we move this out into two files so that we can manage the requirements separately.

    val currentAngle
        get() = Pair(firstAngle, secondAngle)

    val firstAngle
        get() = firstJoint.angle

    fun setFirstAngle(angle: Double): Command =
        Command.of { firstJoint.angle = angle.coerceIn(0.0..180.0) }.requires(this)

    val secondAngle
        get() = secondJoint.angle

    fun setSecondAngle(angle: Double): Command =
        Command.of { secondJoint.angle = angle.coerceIn(0.0..180.0) }.requires(this)

//    fun setPosition(pos: Pair<Double, Double>): Command = setFirstAngle(pos.first) and setSecondAngle(pos.second)

    // how do we decide what angle we want to point the arm? which joint do we move?
}