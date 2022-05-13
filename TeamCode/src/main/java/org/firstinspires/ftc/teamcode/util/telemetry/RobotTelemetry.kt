package org.firstinspires.ftc.teamcode.util.telemetry

import com.amarcolini.joos.command.Component
import com.amarcolini.joos.command.SuperTelemetry

object RobotTelemetry : Component {
    private val teleValues = mutableListOf<TelemetryUpdater>()
    var telemetry: SuperTelemetry? = null

    fun addTelemetry(caption: String, updater: () -> Any) {
        telemetry?.let {
            teleValues.add(TelemetryUpdater(caption, it, updater))
        }
    }

    fun addTelemetry(vararg values: Pair<String, () -> Any>) {
        values.forEach {
            addTelemetry(it.first, it.second)
        }
    }

    fun addLine(line: String) {
        telemetry?.addLine(line)
    }

    fun clearAll() {
        telemetry?.clearAll()
    }

    fun addData(caption: String, value: Any) {
        telemetry?.addData(caption, value)
    }


    override fun update() {
        telemetry?.let {
            teleValues.forEach {
                it.update()
            }
        }
    }

}