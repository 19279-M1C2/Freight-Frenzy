package org.firstinspires.ftc.teamcode.components.arm

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.hardware.Motor
import org.firstinspires.ftc.teamcode.Constants

class MountHeight(private val height: Motor) : AbstractComponent() {
    init {
        subcomponents.add(height)
    }


    fun reset() = height.resetEncoder()

    val angle
        get() = ((height.currentPosition / height.TPR) * 360 % 360) + 90

    // we should give some padding here to make sure we don't really mess up the module
    private fun calculateAngle(target: Double): Double =
        (target.coerceIn(0.0..180.0) - 90 / 360) * height.TPR * Constants.MOUNT_HEIGHT_RATIO


    fun setAngle(target: Double): Command =
        (Command.of {
            height.setSpeed(
                if (target > angle) {
                    0.2
                } else if (target == angle) {
                    0.0
                } else {
                    -0.2
                }
            )
        } then height.goToPosition(this.calculateAngle(target).toInt())).requires(this)
}