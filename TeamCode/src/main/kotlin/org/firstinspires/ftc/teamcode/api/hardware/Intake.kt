package org.firstinspires.ftc.teamcode.api.hardware

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.api.DeviceNames.INTAKE_BOUNCE
import org.firstinspires.ftc.teamcode.api.DeviceNames.INTAKE_L
import org.firstinspires.ftc.teamcode.api.DeviceNames.INTAKE_R
import org.firstinspires.ftc.teamcode.api.framework.Telemetry
import org.firstinspires.ftc.teamcode.api.framework.TelemetrySource
import org.firstinspires.ftc.teamcode.api.framework.init

@Config
class Intake(hardwareMap: HardwareMap, telemetry: Telemetry?) {
    companion object {
        @JvmField
        var INTAKING = 1.0

        @JvmField
        var OUTTAKING = -1.0
    }

    private val leftMotor = hardwareMap.dcMotor[INTAKE_L].init(useEncoder = false, reverse = true)
    private val rightMotor = hardwareMap.dcMotor[INTAKE_R].init(useEncoder = false, reverse = true)
    private val bounceMotor = hardwareMap.dcMotor[INTAKE_BOUNCE].init(useEncoder = false)

    var power = 0.0
        set(value) {
            field = value
            leftMotor.power = value
            rightMotor.power = value
            bounceMotor.power = value
        }

    // Telemetry
    init {
        TelemetrySource {
            put("Intake Power", when (power) {
                INTAKING -> "INTAKING"
                OUTTAKING -> "OUTTAKING"
                else -> power.toString()
            })
        }.register(telemetry)
    }
}