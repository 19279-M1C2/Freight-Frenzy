package org.firstinspires.ftc.teamcode.util

fun weightedAverage(values: List<Double>, weights: List<Double>) =
    values.zip(weights).sumOf { it.first * it.second } / weights.sum()