package org.firstinspires.ftc.teamcode.api

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.api.DeviceNames.LIFT_L
import org.firstinspires.ftc.teamcode.api.DeviceNames.LIFT_R
import org.openftc.revextensions2.ExpansionHubMotor

@Config
class Lift(opMode: LinearOpMode, telemetry: Telemetry?): HardwareComposition(opMode) {

    companion object {
        @JvmField
        var BOTTOM = 0

        @JvmField
        var TOP = 255
    }

    private val motorLeft = opMode.hardwareMap.dcMotor[LIFT_L].init().toUpdateable().register()
    private val motorRight = opMode.hardwareMap.dcMotor[LIFT_R].init(true).toUpdateable().register()

    init {
        TelemetrySource {
            put("Lift Pos", "TODO")
        }.register(telemetry)
    }

    fun goto(position: Int) {
        motorLeft.targetPosition = position
        motorRight.targetPosition = position
    }
}