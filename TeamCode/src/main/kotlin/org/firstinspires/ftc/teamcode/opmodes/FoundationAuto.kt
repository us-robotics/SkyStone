package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.api.*
import org.firstinspires.ftc.teamcode.api.framework.*

open class FoundationAutoBase(mirror: Boolean) : MirrorableAuto(mirror) {
    override fun init(robot: Robot) {
        robot.dt.poseEstimate = P(x = 32.0, y = 62.m, heading = (if (mirror) 90 else 270).rad)
    }

    override fun run(robot: Robot) {
        robot.driveSync {
            driveTo(x = 50.0, y = 40.m, heading = 90.rad, approach = 270.rad)
            addMarker { robot.fg.grabbing = false }
            driveTo(y = 35.m)
        }
        robot.fg.grabbing = true
        robot.driveToSync(x = 40.0, y = 55.m, heading = 180.rad, approach = 90.rad)
    }
}

@Autonomous(name = "Foundation Auto", group = "Config")
class FoundationAuto : Configurator() {
    override fun build() = FoundationAutoBase(mirror.value)
}