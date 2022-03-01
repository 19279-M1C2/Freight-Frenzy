package org.firstinspires.ftc.teamcode.components.arm

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.hardware.Motor
import org.firstinspires.ftc.teamcode.Constants
import kotlin.math.abs

class MountSpin(private val spin: Motor) : AbstractComponent() {
    init {
        subcomponents.add(spin)
    }


    fun reset() = spin.resetEncoder()

    val angle
        get() = (spin.currentPosition / spin.TPR) * 360 % 360

    private fun calculateAngle(target: Double): Double {
        val diff = abs(target - angle)
        return if (diff > 180) {
            spin.currentPosition - ((diff / 360) * spin.TPR * Constants.MOUNT_SPIN_RATIO)
        } else {
            spin.currentPosition + ((diff / 360) * spin.TPR * Constants.MOUNT_SPIN_RATIO)
        }
    }

    fun setAngle(target: Double): Command =
        (Command.of {
            spin.setSpeed(
                if (this.calculateAngle(target) > spin.currentPosition) {
                    0.2
                } else if (this.calculateAngle(target).toInt() == spin.currentPosition) {
                    0.0
                } else {
                    -0.2
                }
            )
        } then spin.goToPosition(this.calculateAngle(target).toInt())).requires(this)
}