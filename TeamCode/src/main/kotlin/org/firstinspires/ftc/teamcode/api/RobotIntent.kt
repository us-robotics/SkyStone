package org.firstinspires.ftc.teamcode.api

@RobotAPI
data class RobotIntent(
        var position: Point = P(x=0.0, y=0.0),
        var angle: Double = 0.0,
        var liftHeight: Double = 0.0,
        var grabFoundation: Boolean = false,
        var depositorAngle: Double = 0.0,
        var depositorGrabbing: Boolean = false
)