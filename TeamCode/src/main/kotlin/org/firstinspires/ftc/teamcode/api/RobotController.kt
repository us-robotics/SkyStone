package org.firstinspires.ftc.teamcode.api

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlin.math.*

@RobotAPI
class RobotController(private val opMode: LinearOpMode) {
    val hardware =  RobotHardware(opMode)

    fun auto(run: RobotAutoController.() -> Unit) {
        val auto = RobotAutoController(opMode, this)
        auto.run()
    }

    fun hardware(run: RobotHardware.() -> Unit) {
        hardware.run()
    }

    // Driving
    var drivePower: Double = 0.0
        set(value) {
            field = value
            updateDrive()
        }

    var driveAngle = 0.0
        set(value) {
            field = value
            updateDrive()
        }

    var driveTurn = 0.0
        set(value) {
            field = value
            updateDrive()
        }

    /**
     * Updates the drivetrain's power based on drivePower and driveAngle
     */
    private fun updateDrive() = hardware.apply {
        val ctrl = this@RobotController
        val x = ctrl.drivePower * cos(ctrl.driveAngle)
        val y = ctrl.drivePower * sin(ctrl.driveAngle)
        val turn = ctrl.driveTurn

        //driveL.power = y + turn
        //driveR.power = y - turn
        //strafeOne.power = x - turn
        //strafeTwo.power = x + turn
    }
}