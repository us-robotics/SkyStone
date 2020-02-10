package org.firstinspires.ftc.teamcode.api.hardware

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.api.hardware.Depositor.Position.*
import org.firstinspires.ftc.teamcode.api.DeviceNames.FOURBAR_LEFT
import org.firstinspires.ftc.teamcode.api.DeviceNames.FOURBAR_RIGHT
import org.firstinspires.ftc.teamcode.api.DeviceNames.GRABBER
import org.firstinspires.ftc.teamcode.api.framework.UpdatableComposition
import org.firstinspires.ftc.teamcode.api.framework.toUpdatable

class Depositor(hardwareMap: HardwareMap) : UpdatableComposition() {
    private val fourBarLeft = hardwareMap.servo[FOURBAR_LEFT].toUpdatable().register()
    private val fourBarRight = hardwareMap.servo[FOURBAR_RIGHT].toUpdatable().register()
    private val grabber = hardwareMap.servo[GRABBER].toUpdatable().register()

    enum class Position {
        IDLE,
        DOWN,
        OUT
    }

    var grabbing: Boolean
        get() = (grabber.position == 1.0)
        set(value) {
            grabber.position = if (value) 1.0 else 0.5
        }

    var position: Position
        get() = when (fourBarLeft.position) {
            1.0 -> OUT
            0.05 -> DOWN
            else -> IDLE
        }

        set(value) {
            fourBarLeft.position = when (value) {
                OUT -> 1.0
                DOWN -> 0.05
                IDLE -> 0.5
            }
            fourBarRight.position = when (value) {
                OUT -> 0.0
                DOWN -> 0.95
                IDLE -> 0.5
            }
        }
}