package org.firstinspires.ftc.teamcode.util

import kotlin.math.abs

/**
 * [RibbieFilter] an awesome filter developed by Robby Marshall
 * It combines multiple sensor inputs into one "estimated" reading
 * @param adjust How much to adjust the values by. This will change over time.
 * Should be between 0 and 1. Start with 0.5 and tune.
 * @param upperBound How far away from the average should the sensor before trust is decreased.
 * Should be between 0 and 1. Start with 0.8 and tune.
 * @param lowerBound How far close from the average should the sensor before trust is increased.
 * Should be between 0 and 1. Start with 0.2 and tune.
 * @param initConfidence A list of the confidences at the start, it will change over time.
 * This will determine what order the sensors should be in future inputs. ALl confidences should be between 0 and 1.
 * Start with 0.8 for all sensors and adjust based on how trustworthy you think each sensor is.
 * You can also take the values after running the loop for a while by calling [RibbieFilter.getConfidences]
 * @property adjust How much the filter is currently adjusting the values
 * @author omagarwal25, Math by Robby Marshall
 * @constructor Inital values for the function
 */
class RibbieFilter(
    adjust: Double,
    private val upperBound: Double,
    private val lowerBound: Double,
    initConfidence: List<Double>
) {

    private val confidence = initConfidence.toMutableList()

    var adjust = adjust
        private set

    /**
     * Conducts one iteration of the Ribbie Filter
     * @param sensor a list of current sensor values
     * @return an estimated "true" value for the position
     */
    fun update(sensor: List<Double>): Double {
        val averageSensor = weightedAverage(sensor, confidence)

        for (i in sensor.indices) {
            val offset = abs(averageSensor - sensor[i]) / averageSensor

            // If offset is higher than the acceptable bound then we decrease it
            // if it is very close to the average we increase its confidence
            // else we just have no change
            val percentChange = when {
                offset > upperBound -> (adjust * offset)
                offset < lowerBound -> 1 + (adjust * (1 - offset))
                else -> 1.0
            }

            confidence[i] = (confidence[i] * percentChange).coerceIn(0.0..1.0)
        }

        // val averageConfidence = confidence.average()

        return weightedAverage(sensor, confidence)
    }

    /**
     * Expose public access to confidences,
     * but make sure it's not mutable just in case someone tries to do something weird
     * @return a list of confidences, same order as the sensors
     */
    fun getConfidences() = confidence.toList()
}