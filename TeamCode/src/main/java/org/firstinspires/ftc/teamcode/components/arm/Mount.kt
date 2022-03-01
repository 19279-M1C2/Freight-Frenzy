package org.firstinspires.ftc.teamcode.components.arm

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.command.Command

class Mount(val spin: MountSpin, val height: MountHeight) : AbstractComponent() {

    init {
        subcomponents.add(spin)
        subcomponents.add(height)
    }

    /**
     * Sets the position of the arm mount (spin, height)
     * @param angle a pair of the degrees
     */
    fun setAngle(angle: Pair<Double, Double>): Command =
        (spin.setAngle(angle.first) and height.setAngle(angle.second))

    /**
     * Current position (spin, height)
     */
    val angle
        get() = Pair(spin.angle, height.angle)

    // ? does this need to be made into a command?
    fun reset() {
        spin.reset();
        height.reset();
    }
}
