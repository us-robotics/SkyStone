package org.firstinspires.ftc.teamcode.api

import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

class Robot(opMode: LinearOpMode) : HardwareComposition(opMode) {
    val telemetry = Telemetry().register()
    val vision = Vision(opMode, telemetry)
    val dt = MecanumDrive(opMode, telemetry).register()
    val fg = FoundationGrabber(opMode, telemetry).register()
    //val lift = Lift(opMode, telemetry).register()

    inline fun execute(func: Robot.() -> Unit) {
        func()
        waitForIdle()
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Convenience functions
    ////////////////////////////////////////////////////////////////////////////////////////////////

    fun splineToSync(x: Double? = null, y: Double? = null, heading: Double? = null,
                     reverse: Boolean = false, forceTan: Boolean = false) = execute {
        dt.splineTo(x, y, heading, reverse, forceTan)
    }

    inline fun driveSync(setup: TrajectoryBuilder.() -> Unit) = execute { dt.drive(setup) }

    fun turnSync(newHeading: Double) = execute { dt.turn(newHeading) }

    fun relTurnSync(angle: Double) = execute { dt.relTurn(angle) }

    // Shakes the robot to get the intake to drop
    fun dropIntake() = execute {
        dt.drive {
            forward(2.0)
            back(2.0)
        }
    }

    fun stop() = dt.stop()
}