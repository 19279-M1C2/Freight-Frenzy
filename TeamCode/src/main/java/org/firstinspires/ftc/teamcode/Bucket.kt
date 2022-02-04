package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.hardware.Servo

/**
 * This bucket class controls a mock bucket. It extends AbstractComponents and adds some methods such as close and open
 * These methods could be more complex
 *
 * @param servo the bucket servo
 */
class Bucket(private val servo: Servo) : AbstractComponent() {
    private var isOpen = false

    // when the servo is created it auto aligns the bucket
    init {
        servo.angle = 0.1
    }

    private fun open() {
        servo.position = 1.0
        isOpen = true
    }

    /**
     * This is an example method that returns a command.
     * The difference between this method and the other methods that are runables is that you have can pass in params
     * @param pos set the pos of the bucket
     */
    fun setPosition(pos: Double): Command = Command.of {
        // this part of the command runs every tick
        update()
    }.onInit {
        // this runs when the command starts
        servo.position = pos
    }.runUntil(servo.position >= pos).requires(this).isInterruptable(true)

    private fun close() {
        servo.position = 1.0
        isOpen = false
    }

    fun toggle() {
        if (isOpen) close() else open()
    }

    fun isOpen(): Boolean {
        return isOpen
    }

    override fun update() {
        servo.update()
    }
}