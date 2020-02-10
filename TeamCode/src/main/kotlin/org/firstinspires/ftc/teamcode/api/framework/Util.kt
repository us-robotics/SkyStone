package org.firstinspires.ftc.teamcode.api.framework

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.api.Robot

typealias P = Pose2d

// Utility properties to convert numbers to radian
val Double.rad: Double
    inline get() = Math.toRadians(this)
val Int.rad: Double
    inline get() = this.toDouble().rad

// Noop property and function, both of which do nothing
inline fun noop() {}
inline val noop: Unit
    get() = Unit

fun <T : DcMotor> T.init(reverse: Boolean = false, useEncoder: Boolean = true) = this.apply {
    direction = if (reverse) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
    mode = if (useEncoder)
        DcMotor.RunMode.RUN_USING_ENCODER
    else
        DcMotor.RunMode.RUN_WITHOUT_ENCODER
    zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
}

fun LinearOpMode.runBeforeStart(cmd: () -> Unit) : Boolean {
    try {
        while (!isStarted) cmd()
    } catch (e: InterruptedException) {
        Thread.currentThread().interrupt()
        return false
    }
    return !isStopRequested
}

fun LinearOpMode.waitForStart(robot: Robot): Boolean = runBeforeStart {
    robot.telemetry.update()
}

/**
 * @see Drivetrain.driveTo
 */
fun TrajectoryBuilder.driveTo(x: Double? = null, y: Double? = null, heading: Double? = null,
                              approach: Double? = null, tan: Boolean = false, linear: Boolean = false) {
    val poseEstimate = this.currentPose!!
    val tgt = P(x = x
            ?: poseEstimate.x, y = y ?: poseEstimate.y,
            heading = approach ?: poseEstimate.heading)
    val tgtHeading = heading ?: this.currentHeading!!
    when {
        tan -> splineTo(tgt)
        heading != null && linear -> splineToLinearHeading(tgt, tgtHeading)
        heading != null -> splineToSplineHeading(tgt, tgtHeading)
        else -> splineToConstantHeading(tgt)
    }
}