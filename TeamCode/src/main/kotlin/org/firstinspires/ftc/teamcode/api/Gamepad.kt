package org.firstinspires.ftc.teamcode.api

import com.qualcomm.robotcore.hardware.Gamepad
import kotlin.math.atan2
import kotlin.math.sqrt

val Gamepad.leftOffset: Double
    get() =
        sqrt((this.left_stick_x * this.left_stick_x) + (this.left_stick_y * this.left_stick_y)).toDouble()

val Gamepad.leftAngle: Double
    get() = atan2(-this.left_stick_y, this.left_stick_x).toDouble()

val Gamepad.rightOffset: Double
    get() =
        sqrt((this.right_stick_x * this.right_stick_x) + (this.right_stick_y * this.right_stick_y)).toDouble()

val Gamepad.rightAngle: Double
    get() = atan2(-this.right_stick_y, this.right_stick_x).toDouble()