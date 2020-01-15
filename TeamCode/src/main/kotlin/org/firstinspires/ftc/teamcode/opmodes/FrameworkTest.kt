package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.api.*

@Autonomous
class FrameworkTest : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(this)
        waitForStart(robot) || return

        robot.turnSync(20.rad)
        robot.execute {
            dt.splineTo(x = 10.0, y = 14.0, heading = -15.rad)
            fg.grab()
        }
        sleep(1000)
        robot.execute {
            dt.splineTo(x = 0.0, y = 0.0, heading = -15.rad, reverse = true)
            fg.ungrab()
        }
        robot.stop()
    }
}