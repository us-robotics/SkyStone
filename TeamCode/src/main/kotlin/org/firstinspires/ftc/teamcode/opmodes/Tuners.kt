package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.config.ValueProvider
import com.acmerobotics.dashboard.config.variable.BasicVariable
import com.acmerobotics.dashboard.config.variable.CustomVariable
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.util.Angle
import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.MovingStatistics
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.teamcode.api.*
import org.firstinspires.ftc.teamcode.api.DriveConstants.BASE_CONSTRAINTS
import org.firstinspires.ftc.teamcode.api.DriveConstants.encoderTicksToInches
import org.firstinspires.ftc.teamcode.api.DriveConstants.kV
import kotlin.math.sqrt

// To tune the drivetrain:
// DriveVelocityPIDTuner - Get the error as reasonably low as possible
// StraightTest - Make sure you're within a few inches of distance
// TrackWidthTuner - This computer the actual track width
// TurnTest - Manually tune track width until you're close enough
// SplineTest - Make sure it's close enough
// FollowerPID - This gets you the rest of the way

@Config
//@Disabled
@Autonomous(group = "Tuning")
class TrackWidthTuner : LinearOpMode() {
    companion object {
        @JvmField
        var ANGLE = 180.0
        @JvmField
        var NUM_TRIALS = 5
    }

    override fun runOpMode() {
        val robot = Robot(this)

        telemetry.log().add("Press play to run the track width tuner")
        telemetry.log().add("Make sure there is enough clearance for the robot to turn")
        telemetry.update()

        waitForStart(robot) || return

        telemetry.log().clear()
        telemetry.log().add("Running...")
        telemetry.update()

        val trackWidthStats = MovingStatistics(5 /* Num trials */)
        for (i in 0..NUM_TRIALS) {
            robot.dt.poseEstimate = Pose2d()

            var headingAccumulator = 0.0
            var lastHeading = 0.0

            robot.dt.relTurn(Math.toRadians(ANGLE))

            while (!isStopRequested && robot.dt.busy) {
                val heading = robot.dt.poseEstimate.heading
                headingAccumulator += Angle.norm(heading - lastHeading)
                lastHeading = heading
                robot.dt.update()
            }

            val trackWidth = DriveConstants.TRACK_WIDTH * ANGLE / headingAccumulator
            trackWidthStats.add(trackWidth)

            sleep(1000)
        }

        telemetry.log().clear()
        telemetry.log().add("Complete!")
        telemetry.log().add("Track width = %.2f (SE = %.3f)".format(encoderTicksToInches(trackWidthStats.mean.toInt()),
                trackWidthStats.standardDeviation / sqrt(NUM_TRIALS.toDouble())))

        while (!isStopRequested) idle()
    }
}

@Config
//@Disabled
@Autonomous(group = "Tuning")
class DriveVelocityPIDTuner : LinearOpMode() {

    companion object {
        @JvmField
        var DISTANCE = 72.0

        const val PID_VAR_NAME = "VELO_PID"
    }

    val dash = FtcDashboard.getInstance()
    var catVar: CustomVariable? = null
    lateinit var robot: Robot

    private fun generateProfile(movingForward: Boolean): MotionProfile {
        val start = MotionState(if (movingForward) 0.0 else DISTANCE, 0.0, 0.0, 0.0)
        val goal = MotionState(if (!movingForward) 0.0 else DISTANCE, 0.0, 0.0, 0.0)
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal,
                BASE_CONSTRAINTS.maxVel, BASE_CONSTRAINTS.maxAccel, BASE_CONSTRAINTS.maxJerk)
    }

    private fun addPidVariable() {
        val catName = javaClass.simpleName
        catVar = dash.configRoot.getVariable(catName) as CustomVariable
        if (catVar == null) {
            catVar = CustomVariable()
            dash.configRoot.putVariable(catName, catVar)
            RobotLog.w("Unable to find top-level category", catVar)
        }

        val pidVar = CustomVariable()
        pidVar.putVariable("kP", BasicVariable(object : ValueProvider<Double> {
            override fun get(): Double {
                return robot.dt.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).kP
            }

            override fun set(value: Double) {
                val coeffs = robot.dt.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER)
                robot.dt.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDCoefficients(value, coeffs.kI, coeffs.kD))
            }
        }))
        pidVar.putVariable("kI", BasicVariable(object : ValueProvider<Double> {
            override fun get(): Double {
                return robot.dt.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).kI
            }

            override fun set(value: Double) {
                val coeffs = robot.dt.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER)
                robot.dt.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDCoefficients(coeffs.kP, value, coeffs.kD))
            }
        }))
        pidVar.putVariable("kD", BasicVariable(object : ValueProvider<Double> {
            override fun get(): Double {
                return robot.dt.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).kD
            }

            override fun set(value: Double) {
                val coeffs = robot.dt.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER)
                robot.dt.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDCoefficients(coeffs.kP, coeffs.kI, value))
            }
        }))
        catVar?.putVariable(PID_VAR_NAME, pidVar)
        dash.updateConfig()
    }

    private fun removePidVariable() {
        catVar?.removeVariable(PID_VAR_NAME)
        dash.updateConfig()
    }

    override fun runOpMode() {
        val clock = NanoClock.system()
        robot = Robot(this)

        addPidVariable()

        telemetry.log().add("Ready!")
        telemetry.update()
        telemetry.clearAll()
        waitForStart()
        if (isStopRequested) return

        var movingForwards = true
        var activeProfile = generateProfile(true)
        var profileStart = clock.seconds()

        while (!isStopRequested) {
            val profileTime = clock.seconds() - profileStart

            if (profileTime > activeProfile.duration()) {
                movingForwards = !movingForwards
                activeProfile = generateProfile(movingForwards)
                profileStart = clock.seconds()
            }

            val state = activeProfile[profileTime]
            val targetPower = kV * state.v
            robot.dt.setDrivePower(Pose2d(targetPower, 0.0, 0.0))

            val velocities = robot.dt.getWheelVelocities()
            TelemetrySource {
                put("targetVelocity", state.v)
                velocities.indices.forEach {
                    put("velocity" + it, velocities[it])
                    put("error" + it, state.v - velocities[it])
                }
            }.register(robot.telemetry)
        }

        removePidVariable()
    }

}

@Config
//@Disabled
@Autonomous(group = "Tuning")
class FollowerPIDTuner : LinearOpMode() {
    companion object {
        @JvmField
        var DIST = 48.0
    }

    override fun runOpMode() {
        val robot = Robot(this)
        robot.dt.poseEstimate = Pose2d(-DIST / 2, -DIST / 2, 0.0)
        waitForStart(robot) || return

        while (!isStopRequested) {
            robot.driveSync { forward(DIST) }
            robot.relTurnSync(90.rad)
        }
    }

}

@Config
//@Disabled
@Autonomous(group = "Tuning")
class TurnTest : LinearOpMode() {
    companion object {
        @JvmField
        var ANGLE = 90.0
    }

    override fun runOpMode() {
        val robot = Robot(this)
        waitForStart(robot) || return

        while (!isStopRequested) {
            robot.dt.poseEstimate = Pose2d(0.0, 0.0, 0.0)
            robot.relTurnSync(ANGLE.rad)
            sleep(1000)
        }
    }
}

@Config
//@Disabled
@Autonomous(group = "Tuning")
class StraightTest : LinearOpMode() {
    companion object {
        @JvmField
        var DIST = 60.0
    }

    override fun runOpMode() {
        val robot = Robot(this)
        val traj = robot.dt.trajectoryBuilder().forward(DIST).build()
        waitForStart(robot) || return

        robot.dt.followTrajectory(traj)
        robot.waitForIdle()
    }

}

@Config
//@Disabled
@Autonomous(group = "Tuning")
class SplineTest : LinearOpMode() {
    override fun runOpMode() {
        val robot = Robot(this)
        waitForStart(robot) || return

        robot.execute {
            robot.dt.splineTo(x = 30.0, y = 30.0)
        }
        sleep(1000)
        robot.execute {
            dt.splineTo(0.0, 0.0, reverse = true)
        }
    }
}