package org.firstinspires.ftc.teamcode.api

import android.annotation.SuppressLint
import android.content.Context
import android.util.Log
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.robotcore.external.navigation.Rotation
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.robotcore.internal.system.PreferencesHelper
import org.firstinspires.ftc.teamcode.api.framework.rad
import org.opencv.core.Scalar
import kotlin.math.PI

@Config
object ProgramConstants {
    private val prefs = AppUtil.getDefContext().getSharedPreferences("ProgramConstants", Context.MODE_PRIVATE)

    @JvmField
    var DEBUG = false

    init { // A small monitoring thread that restarts the app if DEBUG changes
        DEBUG = prefs.getBoolean("DEBUG", false)
        Thread {
            try {
                val oldDebug = DEBUG
                while (true) {
                    if (oldDebug != DEBUG) {
                        prefs.edit().putBoolean("DEBUG", DEBUG).apply()
                        Log.i("ProgramConstants", "Restarting App")
                        AppUtil.getInstance().restartApp(0)
                    }
                    Thread.sleep(1000)
                }
            } catch (e: InterruptedException) {
                Log.e("DebugMonitor", "Interrupted and exiting")
            }
        }.start()
    }
}

object DeviceNames {
    const val MOTOR_FL = "motorFrontLeft"
    const val MOTOR_BL = "motorBackLeft"
    const val MOTOR_BR = "motorBackRight"
    const val MOTOR_FR = "motorFrontRight"

    const val LIFT_L = "motorLiftLeft"
    const val LIFT_R = "motorLiftRight"

    const val FOURBAR_LEFT = "fourbarLeft"
    const val FOURBAR_RIGHT = "fourbarRight"
    const val GRABBER = "grabber"

    const val INTAKE_L = "motorIntakeLeft"
    const val INTAKE_R = "motorIntakeRight"

    const val FOUNDATION_GRABBER_L = "servoFGLeft"
    const val FOUNDATION_GRABBER_R = "servoFGRight"

    const val WEBCAM = "Webcam 1"
    const val IMU = "imu"
    const val HUB = "hub"
    const val HUB2 = "hub2"

    const val ODOM_L = INTAKE_L
    const val ODOM_R = INTAKE_R
    const val ODOM_H = LIFT_R
}

@MotorType(ticksPerRev = 537.6, gearing = 19.2, maxRPM = 312.0, orientation = Rotation.CCW)
@DeviceProperties(xmlTag = "GoBILDA-5202-0002-0019", name = "GoBILDA-5202-0002-0019")
interface GoBUILDA5202_0002_0019

@MotorType(ticksPerRev = 383.6, gearing = 13.7, maxRPM = 435.0, orientation = Rotation.CCW, achieveableMaxRPMFraction = 0.78)
@DeviceProperties(xmlTag = "GoBILDA-5202-0002-0014", name = "GoBILDA-5202-0002-0014")
interface GoBUILDA5202_0002_0014

@Config
object DriveConstants {
    private val MOTOR_CONFIG = MotorConfigurationType.getMotorType(GoBUILDA5202_0002_0014::class.java)!!
    val PID_DISABLED = PIDCoefficients()

    @JvmField
    var GEAR_RATIO = 2.5 / 4.2 // Ratio of wheel:motor speed (???)

    @JvmField
    var WHEEL_RADIUS = 2.0 // 4-inch mecanum wheels

    @JvmField
    var TRACK_WIDTH = 12.5 // Side to Side

    @JvmField
    var WHEEL_BASE = 13.0 // Back v Front

    @JvmField
    var SERVO_MAX_SWEEP_TIME = (49.0 / 75.0) //seconds

    @JvmField
    var TRANSLATIONAL_PID = PIDCoefficients(0.0, 0.0, 0.0)

    @JvmField
    var HEADING_PID = PIDCoefficients(0.0, 0.0, 0.0)
    //PIDCoefficients(-0.9    , 0.0, 0.01)

    @JvmField
    var MOTOR_VELO_PID: PIDCoefficients = PID_DISABLED.copy()
    //PIDCoefficients(20.0, 3.5, 6.5)

    @JvmField
    var MAX_RPM_FRACTION = 0.78

    @JvmField
    var kV = 1.0 / rpmToVelocity(MOTOR_CONFIG.maxRPM * MAX_RPM_FRACTION)

    @JvmField
    var kA = 0.0

    @JvmField
    var kStatic = 0.0

    @JvmField
    var BASE_CONSTRAINTS = DriveConstraints(
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

    val TICKS_PER_SEC: Double
        get() = MOTOR_CONFIG.maxRPM * MOTOR_CONFIG.ticksPerRev / 60.0

    // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
    val MOTOR_VELOCITY_F: Double
        get() = 32767 / TICKS_PER_SEC
}

@Config
object OdometryConstants {
    @JvmField
    var ODOM_L_POS = Pose2d(x = 0.0, y = 7.35, heading = 0.0)

    @JvmField
    var ODOM_R_POS = Pose2d(x = 0.0, y = -7.35, heading = 0.0)

    @JvmField
    var ODOM_H_POS = Pose2d(x = -4.0, y = 0.0, heading = 90.0.rad)

    @JvmField
    var ODOM_WHEEL_RADIUS = 1.0 //in

    @JvmField
    var ODOM_TICKS_PER_REV = 8192.0

    fun odomTicksToInches(ticks: Int) =
            ODOM_WHEEL_RADIUS * 2.0 * PI * ticks / ODOM_TICKS_PER_REV
}

@Config
object VisionConstants {
    enum class Stage {
        NOOP, // Passes the input image through
        THRESHOLD, // Convert the image to YUV and threshold it
        ANNOTATED // Show boundaries and points
    }

    @JvmField
    var OUTPUT_STAGE = Stage.ANNOTATED

    @JvmField
    var BLUR_RADIUS = 0.0

    @JvmField
    var EROSION_ITERATIONS = 3

    data class VisionScalar(@JvmField var a: Double, @JvmField var b: Double, @JvmField var c: Double) {
        fun toScalar() = Scalar(a, b, c)
    }

    @JvmField
    var THRESHOLD_MIN = VisionScalar(100.0, 0.0, 100.0)

    @JvmField
    var THRESHOLD_MAX = VisionScalar(255.0, 100.0, 255.0)

    @JvmField
    var LEFT_BOUNARY = 0.8

    @JvmField
    var RIGHT_BOUNDARY = 0.45
}