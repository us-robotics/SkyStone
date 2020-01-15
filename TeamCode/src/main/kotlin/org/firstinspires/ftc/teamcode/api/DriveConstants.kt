package org.firstinspires.ftc.teamcode.api

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.qualcomm.hardware.motors.GoBILDA5202Series
import com.qualcomm.hardware.motors.RevRobotics20HdHexMotor
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.opencv.core.Range
import org.opencv.core.Scalar
import kotlin.math.PI

object DeviceNames {
    const val MOTOR_FL = "motorFrontLeft"
    const val MOTOR_BL = "motorBackLeft"
    const val MOTOR_FR = "motorFrontRight"
    const val MOTOR_BR = "motorBackRight"

    const val LIFT_L = "motorLiftLeft"
    const val LIFT_R = "motorLiftRight"

    const val INTAKE_L = "motorIntakeLeft"
    const val INTAKE_R = "motorIntakeRight"

    const val FOUNDATION_GRABBER_L = "servoFGLeft"
    const val FOUNDATION_GRABBER_R = "servoFGRight"

    const val WEBCAM = "Webcam 1"
    const val IMU = "imu"
    const val HUB = "hub"
}

@Config
object DriveConstants {
    private val MOTOR_CONFIG = MotorConfigurationType.getMotorType(GoBILDA5202Series::class.java)!!

    @JvmField var GEAR_RATIO = 4.2 / 2.5 // Ratio of wheel:motor speed (???)
    @JvmField var WHEEL_RADIUS = 2.0 // 4-inch mecanum wheels
    @JvmField var TRACK_WIDTH = 13.0 // Side v side
    @JvmField var WHEEL_BASE = 14.0 // Back v Front

    // TODO
    @JvmField var ODOM_1_POS = Pose2d()
    @JvmField var ODOM_2_POS = Pose2d()
    @JvmField var ODOM_3_POS = Pose2d()
    // TODO

    @JvmField var TRANSLATIONAL_PID = PIDCoefficients(0.0, 0.0, 0.0)
    @JvmField var HEADING_PID = PIDCoefficients(0.0, 0.0, 0.0)
            //PIDCoefficients(-0.9    , 0.0, 0.01)
    @JvmField var MOTOR_VELO_PID: PIDCoefficients? = null
            //PIDCoefficients(20.0, 3.5, 6.5)

    @JvmField var kV = 1.0 / rpmToVelocity(MOTOR_CONFIG.maxRPM * MOTOR_CONFIG.achieveableMaxRPMFraction)
    @JvmField var kA = 0.0
    @JvmField var kStatic = 0.0

    @JvmField var BASE_CONSTRAINTS = DriveConstraints(
            maxVel = 40.0,
            maxAccel = 35.0,
            maxJerk = 0.0,
            maxAngVel = Math.toRadians(180.0),
            maxAngAccel = Math.toRadians(180.0),
            maxAngJerk = 0.0
    )

    fun encoderTicksToInches(ticks: Int) =
            WHEEL_RADIUS * 2.0 * PI * GEAR_RATIO * ticks / MOTOR_CONFIG.ticksPerRev

    fun rpmToVelocity(rpm: Double) = rpm * GEAR_RATIO * 2.0 * PI * WHEEL_RADIUS / 60.0
}

@Config
object VisionConstants {
    enum class Stage {
        NOOP, // Passes the input image through
        THRESHOLD, // Convert the image to YUV and threshold it
        ANNOTATED // Show boundaries and points
    }

    @JvmField var OUTPUT_STAGE = Stage.ANNOTATED

    @JvmField var BLUR_RADIUS = 0.0
    @JvmField var EROSION_ITERATIONS = 3

    data class VisionScalar(@JvmField var a: Double, @JvmField var b: Double, @JvmField var c: Double) {
        fun toScalar() = Scalar(a, b, c)
    }

    @JvmField var THRESHOLD_MIN = VisionScalar(100.0, 0.0, 100.0)
    @JvmField var THRESHOLD_MAX = VisionScalar(255.0, 100.0, 255.0)

    @JvmField var LEFT_BOUNARY = 0.8
    @JvmField var RIGHT_BOUNDARY = 0.45
}