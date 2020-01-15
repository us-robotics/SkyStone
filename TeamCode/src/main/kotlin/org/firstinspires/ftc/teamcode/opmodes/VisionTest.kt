package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.api.MecanumDrive
import org.firstinspires.ftc.teamcode.api.Robot
import org.firstinspires.ftc.teamcode.api.Vision
import org.firstinspires.ftc.teamcode.api.waitForStart

@Autonomous(group = "Tuning")
class VisionTest : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(this)
        waitForStart(robot) || return
        while (opModeIsActive()) robot.update()
    }
}