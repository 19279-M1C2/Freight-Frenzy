package org.firstinspires.ftc.teamcode.util.telemetry

import com.amarcolini.joos.command.SuperTelemetry

class TelemetryUpdater(private val item: SuperTelemetry.Item, private val updater: () -> Any) {

    init {
        item.setRetained(true)
        item.setValue(updater())
    }

    constructor(caption: String, telemetry: SuperTelemetry, updater: () -> Any) : this(
        telemetry.addData(
            caption,
            updater()
        ), updater
    )


    fun update() {
        item.setValue(updater())
    }
}
