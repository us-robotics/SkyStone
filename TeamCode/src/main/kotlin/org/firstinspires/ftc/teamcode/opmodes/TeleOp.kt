package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.api.DriveConstants.BASE_CONSTRAINTS
import org.firstinspires.ftc.teamcode.api.Robot
import org.firstinspires.ftc.teamcode.api.rad
import org.firstinspires.ftc.teamcode.api.waitForStart

@TeleOp
@Config
class TeleOp : LinearOpMode() {

    companion object {
        @JvmField var START_POS_X = -32.7
        @JvmField var START_POS_Y = -62.0
        @JvmField var START_POS_HEADING = 90.rad
    }

    override fun runOpMode() {
        val robot = Robot(this)
        robot.dt.poseEstimate = Pose2d(START_POS_X, START_POS_Y, START_POS_HEADING)
        waitForStart(robot) || return

        while (opModeIsActive()) {
            robot.dt.setDriveSignal(DriveSignal(Pose2d(
                    x = -(gamepad1.left_stick_y * BASE_CONSTRAINTS.maxVel),
                    y = -(gamepad1.left_stick_x * BASE_CONSTRAINTS.maxVel),
                    heading = -(gamepad1.right_stick_x * BASE_CONSTRAINTS.maxAngVel)
            )))
            robot.update()
        }
    }
}