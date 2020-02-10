package org.firstinspires.ftc.teamcode.api.hardware

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.api.DeviceNames.LIFT_L
import org.firstinspires.ftc.teamcode.api.DeviceNames.LIFT_R
import org.firstinspires.ftc.teamcode.api.framework.Telemetry
import org.firstinspires.ftc.teamcode.api.framework.init
import org.firstinspires.ftc.teamcode.api.framework.toUpdatable

class Lift(hardwareMap: HardwareMap, telemetry: Telemetry?) {

    private val motorLeft = hardwareMap.dcMotor[LIFT_L].init(useEncoder = false).toUpdatable()
    private val motorRight = hardwareMap.dcMotor[LIFT_R].init(reverse = true, useEncoder = false)

    // TODO: Encoder & telemetry

    var power: Double
        get() = motorRight.power
        set(value) {
            motorLeft.power = value * (if (value >= 0) 1.0 else 0.1)
            motorRight.power = value
        }
}