package org.firstinspires.ftc.teamcode.api

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple

data class Point(var x: Double = 0.0, var y: Double = 0.0) {
    operator fun plus(other: Point) = Point(this.x + other.x, this.y + other.y)
}

typealias P = Point // So you can say `position = P(y = 20.0)`

@DslMarker
annotation class RobotAPI

fun DcMotor.init(reverse: Boolean) = (this as DcMotorEx).init(reverse)

fun DcMotorEx.init(reverse: Boolean) = apply {
    direction = if (reverse) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
    zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    power = 0.0
    //setPositionPIDFCoefficients(p)
    //setVelocityPIDFCoefficients(p,i,d,f)
}