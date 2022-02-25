package org.firstinspires.ftc.teamcode

typealias Radian = Double
typealias Degree = Double
typealias Inch = Double
typealias Meter = Double

object Util {

    fun Degree.toRadian(): Radian = this / 180 * Math.PI
    fun Radian.toDegree(): Degree = this * 180 / Math.PI

    fun Inch.toMeter(): Meter = 0.0254 * this

    // if you use this, I will cry.
    fun Meter.toInch(): Inch = 39.37008 * this
}