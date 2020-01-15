package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.api.*
import org.firstinspires.ftc.teamcode.api.VisionPipeline.Position.LEFT
import org.firstinspires.ftc.teamcode.api.VisionPipeline.Position.RIGHT
import org.firstinspires.ftc.teamcode.opmodes.TeleOp.Companion.START_POS_HEADING
import org.firstinspires.ftc.teamcode.opmodes.TeleOp.Companion.START_POS_X
import org.firstinspires.ftc.teamcode.opmodes.TeleOp.Companion.START_POS_Y

@Autonomous
class Auto : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(this)

        robot.dt.poseEstimate = Pose2d(START_POS_X, START_POS_Y, START_POS_HEADING)
        robot.telemetry.update()
        waitForStart(robot) || return

        val visionOne = when (robot.vision.position) {
            RIGHT -> Pose2d(x = -25.0, y = -36.0, heading = 90.rad)
            LEFT -> Pose2d(x = -40.8, y = -36.0, heading = 90.rad)
            else -> Pose2d(x = -33.2, y = -36.0, heading = 90.rad)
        }
        val visionTwo = when (robot.vision.position) {
            RIGHT -> Pose2d(x = -49.8, y = -36.0, heading = 90.rad)
            LEFT -> Pose2d(x = -59.6, y = -36.0, heading = 90.rad)
            else -> Pose2d(x = -59.6, y = -36.0, heading = 90.rad)
        }

        robot.driveSync {
            splineTo(visionOne, ConstantInterpolator(90.rad))
            forward(10.0)
        }
        robot.relTurnSync(-135.rad)
        robot.driveSync { splineTo(Pose2d(x = 0.0, y = -45.0, heading = 0.0)) }
        robot.driveSync { forward(20.0) }
        sleep(50)
        robot.driveSync { back(20.0) }
        robot.turnSync(90.rad)
        robot.driveSync { back(50.0) }
        robot.dt.poseEstimate = Pose2d(0.0, -75.0, 90.rad)
        robot.driveSync { forward(10.0) }

        robot.driveSync {
            splineTo(visionTwo, ConstantInterpolator(90.rad))
            forward(10.0)
        }
        robot.turnSync(-135.rad)
        robot.driveSync {
            splineTo(Pose2d(x = 0.0, y = -45.0, heading = 0.0))
            forward(20.0)
            back(20.0)
        }

    }
}