package org.firstinspires.ftc.teamcode.util

fun <T, B, V> tripleZip(first: List<T>, second: List<B>, third: List<V>) =
    first.zip(second).zip(third).map { (a, b) -> Triple(a.first, a.second, b) }