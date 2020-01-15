package org.firstinspires.ftc.teamcode.api

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket

class Telemetry : Updateable {

    val sources = mutableListOf<TelemetrySource>()

    private val dash = FtcDashboard.getInstance()

    override val busy: Boolean
        get() = false

    override fun update() {
        val pkt = TelemetryPacket()
        sources.forEach { pkt.(it.populate)() }
        dash.sendTelemetryPacket(pkt)
    }

}

data class TelemetrySource(val populate: (TelemetryPacket.() -> Unit)) {
    private var telemetry: Telemetry? = null

    fun register(telem: Telemetry?) = apply {
        unregister()
        telem?.sources?.add(this)
        telemetry = telem
    }

    fun unregister() {
        telemetry?.sources?.remove(this)
        telemetry = null
    }
}