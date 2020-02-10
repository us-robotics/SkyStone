package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.api.hardware.Depositor.Position.*
import org.firstinspires.ftc.teamcode.api.DriveConstants
import org.firstinspires.ftc.teamcode.api.DriveConstants.BASE_CONSTRAINTS
import org.firstinspires.ftc.teamcode.api.Robot
import org.firstinspires.ftc.teamcode.api.ToggleableGamepad
import org.firstinspires.ftc.teamcode.api.ToggleableGamepad.Mode.TOGGLE
import kotlin.math.sign

@TeleOp
class TeleOp : OpMode() {
    // Convert joystick inputs into an exponential scale
    private val Double.exp: Double
        get() = sign(this) * this * this

    private lateinit var robot: Robot
    private lateinit var gamepadDrive: ToggleableGamepad
    private lateinit var gamepadAux: ToggleableGamepad

    // Set up the robot
    override fun init() {
        robot = Robot(this)
        robot.run {
            dt.initTeleop()
            depositor.position = DOWN
            fg.grabbing = true
        }

        // Gamepads with toggleable buttons
        gamepadDrive = ToggleableGamepad(gamepad1, rbMode = TOGGLE)
        gamepadAux = ToggleableGamepad(gamepad2, aMode = TOGGLE)
    }

    override fun loop() = robot.run {
        // Driving
        val velMultiplier = if (gamepad1.left_bumper) 0.3 else 1.0
        val maxVel = BASE_CONSTRAINTS.maxVel * velMultiplier
        val maxAngVel = BASE_CONSTRAINTS.maxAngVel * velMultiplier
        dt.setDriveSignal(DriveSignal(Pose2d(
                x = -(gamepadDrive.leftStickX.exp * maxVel),
                y = (gamepadDrive.leftStickY.exp * maxVel),
                heading = -(gamepadDrive.rightStickX.exp * maxAngVel)
        )))

        // Aux. systems
        intake.power = gamepadAux.rightStickY
        //lift.power = -gamepadAux.leftStickY // TODO: Make this not bad
        fg.grabbing = !gamepadDrive.rightBumper.value
        depositor.grabbing = gamepadAux.a.value

        // TODO: Make this not bad
        depositor.position = when {
            gamepad2.right_bumper -> OUT
            gamepad2.left_bumper -> IDLE
            else -> DOWN
        }

        // Update the hardware and telemetry
        update()
        gamepadDrive.update()
        gamepadAux.update()
    }
}