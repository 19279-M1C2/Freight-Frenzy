package org.firstinspires.ftc.teamcode

// all distances in M
object Constants {
    const val ARM_HEIGHT: Double = 0.02

    // to make sure the arm doesn't crush itself, we have an angle ofset.
    const val ARM_CUSHION: Double = 5.0

    // length of the first fold
    const val FIRST_FOLD_LENGTH: Double = 18.0 * 0.0254

    const val FIRST_FOLD_NAME = "first-fold-servo"
    const val SECOND_FOLD_NAME = "second-fold-servo"

    const val MOUNT_SPIN_NAME = "mount-spin-motor"
    const val MOUNT_HEIGHT_NAME = "mount-height-name"
    const val MOUNT_SPIN_RATIO = 4.0
    const val MOUNT_HEIGHT_RATIO = 4.0

    const val ULTRAPLANATRY_TICKS = 28.0
}