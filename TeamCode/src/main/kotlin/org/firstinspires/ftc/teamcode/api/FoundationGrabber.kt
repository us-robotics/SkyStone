package org.firstinspires.ftc.teamcode.api

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.api.DeviceNames.FOUNDATION_GRABBER_L
import org.firstinspires.ftc.teamcode.api.DeviceNames.FOUNDATION_GRABBER_R

@Config
class FoundationGrabber(opMode: LinearOpMode, telemetry: Telemetry?) : HardwareComposition(opMode) {
    companion object {
        @JvmField
        var UNGRABBING = 1.0

        @JvmField
        var GRABBING = 0.0
    }

    val servoLeft = opMode.hardwareMap.servo[FOUNDATION_GRABBER_L].toUpdateable().register()
    val servoRight = opMode.hardwareMap.servo[FOUNDATION_GRABBER_R].toUpdateable().register()

    val position: Double get() = servoLeft.position

    inline var targetPosition: Double?
        get() = servoLeft.targetPosition
        set(value) {
            servoLeft.targetPosition = value
            servoRight.targetPosition = value
        }

    init {
        TelemetrySource {
            val posToDisp = { it: Double? ->
                when (it) {
                    GRABBING -> "Grabbing"
                    UNGRABBING -> "Not Grabbing"
                    null -> "None"
                    else -> it.toString()
                }
            }

            put("FG Position", posToDisp(position))
            put("FG Target", posToDisp(targetPosition))
        }.register(telemetry)
    }


    fun grab() {
        targetPosition = GRABBING
    }

    fun ungrab() {
        targetPosition = UNGRABBING
    }
}