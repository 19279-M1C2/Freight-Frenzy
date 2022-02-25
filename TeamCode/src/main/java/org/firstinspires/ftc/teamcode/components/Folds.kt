package org.firstinspires.ftc.teamcode.components

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.hardware.Servo

class Folds(private val firstJoint: Servo, private val secondJoint: Servo) : AbstractComponent() {
    init {
        subcomponents.add(firstJoint)
        subcomponents.add(secondJoint)
    }

    fun getCurrentAngle(): Pair<Double, Double> = Pair(firstAngle, secondAngle)

    val firstAngle
        get() = firstJoint.angle

    fun setFirstAngle(angle: Double): Command = Command.of { firstJoint.angle = angle }

    val secondAngle
        get() = secondJoint.angle

    fun setSecondAngle(angle: Double): Command = Command.of { secondJoint.angle = angle.coerceIn(0.0..180.0) }

    fun setPosition(pos: Pair<Double, Double>): Command = setFirstAngle(pos.first) and setSecondAngle(pos.second)

    // how do we decide what angle we want to point the arm? which joint do we move?
}