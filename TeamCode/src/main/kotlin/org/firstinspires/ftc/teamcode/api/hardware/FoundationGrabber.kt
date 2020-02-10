package org.firstinspires.ftc.teamcode.api.hardware

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.api.DeviceNames.FOUNDATION_GRABBER_L
import org.firstinspires.ftc.teamcode.api.DeviceNames.FOUNDATION_GRABBER_R
import org.firstinspires.ftc.teamcode.api.framework.Telemetry
import org.firstinspires.ftc.teamcode.api.framework.TelemetrySource
import org.firstinspires.ftc.teamcode.api.framework.UpdatableComposition
import org.firstinspires.ftc.teamcode.api.framework.toUpdatable

@Config
class FoundationGrabber(hardwareMap: HardwareMap, telemetry: Telemetry?) : UpdatableComposition() {
    private val servoLeft = hardwareMap.servo[FOUNDATION_GRABBER_L].toUpdatable().register()
    private val servoRight = hardwareMap.servo[FOUNDATION_GRABBER_R].toUpdatable().register()

    var grabbing: Boolean
        get() = (servoLeft.position == 0.95)
        set(value) {
            servoLeft.position = if (value) 0.95 else 0.7
            servoRight.position = if (value) 0.0 else 0.3
        }

    init {
        TelemetrySource {
            put("FG Grabbing", grabbing)
        }.register(telemetry)
    }
}