package org.firstinspires.ftc.teamcode.api

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator
import com.acmerobotics.roadrunner.path.heading.TangentInterpolator
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.TrajectoryConfig
import com.fasterxml.jackson.core.util.VersionUtil
import com.fasterxml.jackson.databind.ObjectMapper
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory
import com.fasterxml.jackson.module.kotlin.KotlinModule
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.Version
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.openftc.revextensions2.ExpansionHubMotor
import java.lang.NumberFormatException
import java.lang.RuntimeException
import kotlin.experimental.and
import kotlin.reflect.jvm.internal.impl.load.kotlin.JvmType


object TrajectoryLoader {
    val MAPPER = ObjectMapper(YAMLFactory())

    init {
        MAPPER.registerModule(KotlinModule())
    }

    fun loadConfig(name: String): TrajectoryConfig {
        val input = AppUtil.getDefContext().assets.open("trajectory/" + name + ".yaml")
        return MAPPER.readValue(input, TrajectoryConfig::class.java)
    }

    /**
     * Load trajectory from config file by name
     */
    fun load(name: String) = loadConfig(name).toTrajectory()
}


object LynxModuleUtil {
    class LynxFirmwareVersionException(msg: String) : RuntimeException(msg)

    val MIN_VERSION = LynxFwVersion(1, 8, 2)

    data class LynxFwVersion(val major: Int, val minor: Int, val patch: Int) : Comparable<LynxFwVersion> {

        override fun toString() = "%d.%d.%d".format(major, minor, patch)

        override operator fun compareTo(other: LynxFwVersion): Int {
            var cmp = major.compareTo(other.major)
            if (cmp != 0) return cmp
            cmp = minor.compareTo(other.minor)
            if (cmp != 0) return cmp
            return patch.compareTo(other.patch)
        }
    }

    private fun getFwVersion(module: LynxModule): LynxFwVersion? {
        val verStr = module.nullableFirmwareVersionString ?: return null
        val parts = verStr.split(Regex("[ :,]+"))
        return try {
            LynxFwVersion(parts[3].toInt(), parts[5].toInt(), parts[7].toInt())
        } catch (e: NumberFormatException) {
            null
        }
    }

    fun ensureMinFWVersion(hardwareMap: HardwareMap) {
        var outdated = mutableMapOf<String, LynxFwVersion?>()
        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            val version = getFwVersion(module)
            if (version == null || version < MIN_VERSION)
                for (name in hardwareMap.getNamesOf(module))
                    outdated.put(name, version)
        }
        if (outdated.isNotEmpty()) {
            val builder = StringBuilder()
            builder.appendln("One or more REV Hubs has outdated firmware")
            builder.appendln("Required version: %s".format(MIN_VERSION.toString()))
            for ((name, version) in outdated)
                builder.appendln("%s: %s".format(name, version?.toString() ?: "Unknown"))
            throw LynxFirmwareVersionException(builder.toString())
        }
    }
}

fun <T : DcMotor> T.init(reverse: Boolean = false) = this.apply {
    direction = if (reverse) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
    mode = DcMotor.RunMode.RUN_USING_ENCODER
    zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
}

enum class AxesSigns(val bval: Int) {
    PPP(0b000),
    PPN(0b001),
    PNP(0b010),
    PNN(0b011),
    NPP(0b100),
    NPN(0b101),
    NNP(0b110),
    NNN(0b111)
}

fun BNO055IMU.remapAxes(order: AxesOrder, signs: AxesSigns) {
    try {
        val indices = order.indices()
        var axisMapConfig = 0
        axisMapConfig = axisMapConfig or (indices[0] shl 4)
        axisMapConfig = axisMapConfig or (indices[1] shl 2)
        axisMapConfig = axisMapConfig or indices[2]

        val axisMapSign = signs.bval xor (0b100 shr indices[0])

        // Set the IMU into write mode
        write8(BNO055IMU.Register.OPR_MODE, (BNO055IMU.SensorMode.CONFIG.bVal and 0x0F).toInt())
        Thread.sleep(100)

        // Write the configuration
        write8(BNO055IMU.Register.AXIS_MAP_CONFIG, axisMapConfig and 0x3F)
        write8(BNO055IMU.Register.AXIS_MAP_SIGN, axisMapSign and 0x07)

        // Set the IMU back into its previous mode
        write8(BNO055IMU.Register.OPR_MODE, (parameters.mode.bVal and 0x0F).toInt())
        Thread.sleep(100)
    } catch (e: InterruptedException) {
        Thread.currentThread().interrupt()
    }
}

val Double.rad: Double
    inline get() = Math.toRadians(this)
val Int.rad: Double
    inline get() = this.toDouble().rad

class ConcurrentOperationException : RuntimeException("Cannot run these tasks concurrently")

fun LinearOpMode.waitForStart(robot: Robot): Boolean {
    try {
        while (!isStarted) robot.telemetry.update()
    } catch (e: InterruptedException) {
        Thread.currentThread().interrupt()
        return false
    }
    return !isStopRequested
}

/**
 * @see MecanumDrive.splineTo
 */
fun TrajectoryBuilder.splineTo(x: Double? = null, y: Double? = null, heading: Double? = null,
             forceTan: Boolean = false) {
    val poseEstimate = build().end()

    val tgtPos = Pose2d(x ?: poseEstimate.x,
            y ?: poseEstimate.y,
            heading ?: poseEstimate.heading)
    val interp = when {
        forceTan -> TangentInterpolator()
        heading != null && heading != poseEstimate.heading ->
            LinearInterpolator(poseEstimate.heading, heading - poseEstimate.heading)
        heading == poseEstimate.heading ->
            ConstantInterpolator(poseEstimate.heading)
        else -> TangentInterpolator()
    }

    splineTo(tgtPos, interp)
}

fun Servo.toUpdateable() = UpdateableServo(this)

fun DcMotor.toUpdateable() = (this as DcMotorEx).toUpdateable()

fun DcMotorEx.toUpdateable() = UpdateableMotor(this)