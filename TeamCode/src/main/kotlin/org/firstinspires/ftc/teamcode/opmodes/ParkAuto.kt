package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.api.*
    import org.firstinspires.ftc.teamcode.api.hardware.Depositor.Position.DOWN
import org.firstinspires.ftc.teamcode.api.framework.Configurator
import org.firstinspires.ftc.teamcode.api.framework.MirrorableAuto
import org.firstinspires.ftc.teamcode.api.framework.P
import org.firstinspires.ftc.teamcode.api.framework.rad
import org.firstinspires.ftc.teamcode.opmodes.ParkAutoBase.Zone.BUILDING
import org.firstinspires.ftc.teamcode.opmodes.ParkAutoBase.Zone.LOADING

open class ParkAutoBase(mirror: Boolean, val center: Boolean, val zone: Zone) : MirrorableAuto(mirror) {
    enum class Zone { BUILDING, LOADING }

    private val startPose = when (zone) {
        BUILDING -> P(x = 32.0, y = 62.m, heading = (if (mirror) 90 else 270).rad)
        LOADING -> P(x = -32.0, y = 62.m, heading = (if (mirror) 90 else 270).rad)
    }

    override fun init(robot: Robot) {
        robot.dt.poseEstimate = startPose
        robot.fg.grabbing = true
        robot.depositor.position = DOWN
    }

    override fun run(robot: Robot) {
        val y = (if (center) 36 else 62).m
        when (zone) {
            BUILDING -> robot.driveToSync(x = 0.0, y = y, approach = 180.rad, tan = true)
            LOADING -> robot.driveToSync(x = 0.0, y = y, approach = 0.0, tan = true)
        }
    }
}

@Autonomous(name = "Park Auto", group = "Config")
class ParkAuto : Configurator() {
    private val zone = toggle("Foundation Side")
    private val center = toggle("Park Towards Center")

    override fun build() = ParkAutoBase(
            mirror = mirror.value,
            center = center.value,
            zone = if (zone.value) BUILDING else LOADING
    )
}