package org.firstinspires.ftc.teamcode.api.framework

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import kotlin.experimental.and

object LynxModuleUtil {
    class LynxFirmwareVersionException(msg: String) : RuntimeException(msg)

    private val MIN_VERSION = LynxFwVersion(1, 8, 2)

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
        val outdated = mutableMapOf<String, LynxFwVersion?>()
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