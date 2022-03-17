package org.firstinspires.ftc.teamcode.util

import kotlin.math.abs
import kotlin.math.pow

/**
 * A simple implementation of a Kalman filter.
 * @param processVariance How fast your measurement moves.
 * @param initialVariance Estimation Uncertainty - Can be initialized with the same value as measurementVariance since the kalman filter will adjust its value.
 */
class SimpleKalmanFilter(
    private val processVariance: Double,
    private val initialVariance: Double
) {
    var estimationVariance = initialVariance
    private var lastMeasurement: Double? = null

    fun getEstimate(measurement: Double): Double {
        val last = lastMeasurement ?: measurement

        val estimate =
            last + estimationVariance * (measurement - last)

        estimationVariance =
            (1 - estimationVariance) * estimationVariance + abs(last - estimate) * processVariance

        lastMeasurement = estimate

        return estimate
    }

    fun clear() {
        estimationVariance = initialVariance
        lastMeasurement = null
    }
}

class FusedKalmanFilter(
    processVariances: List<Double>,
    initialVariances: List<Double>
) {
    private val filters = processVariances.zip(initialVariances).map { (a, b) ->
        SimpleKalmanFilter(a, b)
    }

    fun getEstimate(measurements: List<Double>) =
        filters.zip(measurements)
            .sumOf { (filter, estimate) -> filter.estimationVariance.pow(-1) * filter.getEstimate(estimate) } / (filters.sumOf { it.estimationVariance })
}