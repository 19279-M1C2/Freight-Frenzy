package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.teamcode.components.Vision
import org.firstinspires.ftc.teamcode.components.arm.Arm

fun <T, B, V> tripleZip(first: List<T>, second: List<B>, third: List<V>) =
    first.zip(second).zip(third).map { (a, b) -> Triple(a.first, a.second, b) }

fun weightedAverage(values: List<Double>, weights: List<Double>) =
    values.zip(weights).sumOf { it.first * it.second } / weights.sum()

// Let's just make it very clear what units we're using to avoid confusion
// Anything to do with RoadRunner is in Inches
typealias Radian = Double
typealias Degree = Double
typealias Inch = Double
typealias Meter = Double

fun Degree.toRadian(): Radian = this / 180 * Math.PI
fun Radian.toDegree(): Degree = this * 180 / Math.PI

fun Inch.toMeter(): Meter = 0.0254 * this

// if you use this, I will cry.
fun Meter.toInch(): Inch = 39.37008 * this

fun Vision.ObjectPosition.toLevel(): Arm.Position =
    when (this) {
        Vision.ObjectPosition.LEFT -> Arm.Position.LOW
        Vision.ObjectPosition.RIGHT -> Arm.Position.MEDIUM
        Vision.ObjectPosition.MIDDLE -> Arm.Position.HIGH
    }
