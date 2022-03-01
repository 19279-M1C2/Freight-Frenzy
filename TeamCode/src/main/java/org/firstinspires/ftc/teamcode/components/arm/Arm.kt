package org.firstinspires.ftc.teamcode.components.arm

import com.amarcolini.joos.command.AbstractComponent
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.Util.toDegree
import kotlin.math.atan
import kotlin.math.tan

class Arm(private val mount: Mount, private val folds: Folds) : AbstractComponent() {

    val height
        get() = (tan(180 - folds.secondAngle) * Constants.FIRST_FOLD_LENGTH) - Constants.ARM_HEIGHT


    init {
        subcomponents.add(mount)
        subcomponents.add(folds)
    }

    // set height in M
//    fun setHeight()

    fun setHeightAngle(angle: Double) = mount.height.setAngle(angle) and folds.setSecondAngle(angle)

    fun setHeight(height: Double) =
        setHeightAngle((180 - atan(height + Constants.ARM_HEIGHT / Constants.FIRST_FOLD_LENGTH).toDegree()))

    fun setSpin(angle: Double) = mount.spin.setAngle(angle)
}