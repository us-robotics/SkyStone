package org.firstinspires.ftc.teamcode.api

import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.api.hardware.*
import org.firstinspires.ftc.teamcode.api.framework.HardwareComposition
import org.firstinspires.ftc.teamcode.api.framework.Telemetry

class Robot(opMode: OpMode) : HardwareComposition(opMode) {
    val telemetry = Telemetry().register()
    //val vision = Vision(opMode, telemetry)
    val dt = Drivetrain(opMode.hardwareMap, telemetry).register()
    val fg = FoundationGrabber(opMode.hardwareMap, telemetry).register()
    val intake = Intake(opMode.hardwareMap, telemetry)
    //val lift = Lift(opMode.hardwareMap, telemetry)
    val depositor = Depositor(opMode.hardwareMap).register()

    inline fun execute(func: Robot.() -> Unit) {
        func()
        waitForIdle()
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Convenience functions
    ////////////////////////////////////////////////////////////////////////////////////////////////

    fun driveToSync(x: Double? = null, y: Double? = null, heading: Double? = null,
                    approach: Double? = null, tan: Boolean = false, linear: Boolean = false) = execute {
        dt.driveTo(x, y, heading, approach, tan, linear)
    }

    inline fun driveSync(setup: TrajectoryBuilder.() -> Unit) = execute { dt.drive(setup) }

    fun turnSync(newHeading: Double) = execute { dt.turn(newHeading) }

    fun relTurnSync(angle: Double) = execute { dt.relTurn(angle) }

    // Shakes the robot to get the intake to drop
    fun dropIntake() = dt.drive {
        forward(4.0)
        back(4.0)
    }

    fun stop() {
        dt.stop()
        //vision.destroy()
    }
}
