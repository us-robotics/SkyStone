package org.firstinspires.ftc.teamcode.api

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.qualcomm.hardware.motors.RevRobotics20HdHexMotor
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import kotlin.math.PI

object Constants {
    val MOTOR_CONFIG = MotorConfigurationType.getMotorType(RevRobotics20HdHexMotor::class.java)

    val GEAR_RATIO = 1.0 // Ratio of wheel:motor speed
    val WHEEL_RADIUS = 1.5
    val TRACK_WIDTH = 13.5
    val WHEEL_BASE = TRACK_WIDTH

    val TRANSLATIONAL_PID = PIDCoefficients(0.0, 0.0, 0.0)
    val HEADING_PID = PIDCoefficients(0.0, 0.0, 0.0)

    val kV = 1.0 / rpmToVelocity(MOTOR_CONFIG.maxRPM)
    val kA = 0.0
    val kStatic = 0.0

    val BASE_CONSTRAINTS = DriveConstraints(
            maxVel = 30.0,
            maxAccel = 30.0,
            maxJerk = 0.0,
            maxAngVel = Math.toRadians(180.0),
            maxAngAccel = Math.toRadians(180.0),
            maxAngJerk = 0.0
    )

    fun encoderTicksToInches(ticks: Int) =
            WHEEL_RADIUS * 2 * PI * GEAR_RATIO * ticks / MOTOR_CONFIG.ticksPerRev

    fun rpmToVelocity(rpm: Double) = rpm * GEAR_RATIO * 2 * PI * WHEEL_RADIUS / 60.0
}