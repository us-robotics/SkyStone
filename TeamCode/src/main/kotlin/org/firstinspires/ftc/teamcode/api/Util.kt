package org.firstinspires.ftc.teamcode.api

import com.acmerobotics.roadrunner.trajectory.TrajectoryConfig
import com.fasterxml.jackson.core.util.VersionUtil
import com.fasterxml.jackson.databind.ObjectMapper
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory
import com.fasterxml.jackson.module.kotlin.KotlinModule
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Version
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.openftc.revextensions2.ExpansionHubMotor
import java.lang.NumberFormatException
import java.lang.RuntimeException


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

    fun getFwVersion(module: LynxModule): LynxFwVersion? {
        val verStr = module.nullableFirmwareVersionString ?: return null
        val parts = verStr.split("[ :,]+")
        try {
            return LynxFwVersion(parts[3].toInt(), parts[5].toInt(), parts[7].toInt())
        } catch (e: NumberFormatException) {
            return null
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

fun ExpansionHubMotor.init(reverse: Boolean = false) = this.apply {
    direction = if (reverse) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
    mode = DcMotor.RunMode.RUN_USING_ENCODER
    zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
}