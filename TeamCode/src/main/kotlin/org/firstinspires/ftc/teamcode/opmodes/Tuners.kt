package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.util.Angle
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.MovingStatistics
import org.firstinspires.ftc.teamcode.api.Constants
import org.firstinspires.ftc.teamcode.api.MecanumDrive
import kotlin.math.sqrt

const val NUM_TRIALS = 5.0
const val ANGLE = 180.0

@Autonomous(group = "Tuning")
class TrackWidthTuner : LinearOpMode() {

    override fun runOpMode() {
        val drive = MecanumDrive(hardwareMap)

        telemetry.log().add("Press play to run the track width tuner")
        telemetry.log().add("Make sure there is enough clearance for the robot to turn")
        telemetry.update()

        waitForStart()
        if (isStopRequested) return

        telemetry.log().clear()
        telemetry.log().add("Running...")
        telemetry.update()

        val trackWidthStats = MovingStatistics(5 /* Num trials */)
        for (i in 0..5) {
            drive.poseEstimate = Pose2d()

            var headingAccumulator = 0.0
            var lastHeading = 0.0

            drive.turn(Math.toRadians(ANGLE))

            while (!isStopRequested && drive.busy) {
                val heading = drive.poseEstimate.heading
                headingAccumulator += Angle.norm(heading - lastHeading)
                lastHeading = heading
                drive.update()
            }

            val trackWidth = Constants.TRACK_WIDTH * ANGLE / headingAccumulator
            trackWidthStats.add(trackWidth)

            sleep(1000)
        }

        telemetry.log().clear()
        telemetry.log().add("Complete!")
        telemetry.log().add("Track width = %.2f (SE = %.3f)".format(trackWidthStats.mean,
                trackWidthStats.standardDeviation / sqrt(NUM_TRIALS)))

        while (!isStopRequested) idle()
    }
}