package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.hardware.Servo

class Bucket(private val servo: Servo) : AbstractComponent() {
    private var isOpen = false

    init {
        servo.angle = 0.1
    }

    private fun open() {
        servo.position = 1.0
        isOpen = true
    }

    fun setPos(pos: Double): Command = Command.of {
        servo.position = pos
        servo.update()
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