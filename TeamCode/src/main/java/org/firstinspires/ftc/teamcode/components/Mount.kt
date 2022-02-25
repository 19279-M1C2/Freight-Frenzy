package org.firstinspires.ftc.teamcode.components

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.hardware.Motor
import org.firstinspires.ftc.teamcode.Constants
import kotlin.math.abs

class Mount(private val spin: Motor, private val height: Motor) : AbstractComponent() {

    init {
        subcomponents.add(spin)
        subcomponents.add(height)
    }

    /**
     * Groups all the spin controls
     */
    inner class Spin {
        fun reset() = spin.resetEncoder()

        fun getAngle(): Double = (spin.currentPosition / spin.TPR) * 360 % 360

        private fun calculateAngle(target: Double): Double {
            val diff = abs(target - getAngle())
            return if (diff > 180) {
                spin.currentPosition - ((diff / 360) * spin.TPR * Constants.MOUNT_SPIN_RATIO)
            } else {
                spin.currentPosition + ((diff / 360) * spin.TPR * Constants.MOUNT_SPIN_RATIO)
            }
        }

        fun setAngle(target: Double): Command =
            spin.goToPosition(this@Spin.calculateAngle(target).toInt())
    }

    /**
     * Groups all the height controls
     */
    inner class Height {
        fun reset() = height.resetEncoder()

        fun getAngle(): Double = ((height.currentPosition / height.TPR) * 360 % 360) + 90

        // we should give some padding here to make sure we don't really mess up the module
        private fun calculateAngle(target: Double): Double =
            (target.coerceIn(0.0..180.0) - 90 / 360) * spin.TPR * Constants.MOUNT_HEIGHT_RATIO


        fun setAngle(target: Double): Command =
            height.goToPosition(this@Height.calculateAngle(target).toInt())
    }

    /**
     * Sets the position of the arm mount
     * @param angle a pair of the degrees
     */
    fun setAngle(angle: Pair<Double, Double>): Command =
        Spin().setAngle(angle.first) and Height().setAngle(angle.second)

    /**
     * Gets the current position in degrees
     *
     * @return a pair of the degree positions (spin, height)
     */
    fun getCurrentAngle(): Pair<Double, Double> =
        Pair(Spin().getAngle(), Height().getAngle())

    fun reset() {
        Spin().reset();
        Height().reset();
    }
}
