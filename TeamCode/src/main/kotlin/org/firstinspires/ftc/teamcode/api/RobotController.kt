package org.firstinspires.ftc.teamcode.api

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@RobotAPI
class RobotController(val opMode: LinearOpMode) {
    val hardware =  RobotHardware(opMode)

    fun auto(run: RobotAutoController.() -> Unit) {
        val auto = RobotAutoController(opMode, this)
        auto.run()
    }

    fun hardware(run: RobotHardware.() -> Unit) {
        hardware.run()
    }
}