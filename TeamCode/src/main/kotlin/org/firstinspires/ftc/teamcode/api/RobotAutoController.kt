package org.firstinspires.ftc.teamcode.api

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime

@RobotAPI
class RobotAutoController(val opMode: LinearOpMode, val controller: RobotController) {
    private val intent = RobotIntent()

    private val elapsedTime = ElapsedTime().apply {
        reset() // Start the timer
    }
    val time: Double
        get() = elapsedTime.seconds()

    fun execute(modify: RobotIntent.() -> Unit) {
        intent.modify()
        //TODO
    }

    fun direct(execute: RobotController.() -> Unit) {
        controller.execute()
    }

    fun hardware(execute: RobotHardware.() -> Unit) {
        controller.hardware(execute)
    }

    fun moveTo(x: Double = intent.position.x, y: Double = intent.position.y,
               angle: Double = intent.angle) = execute {
        this.position = P(x, y)
        this.angle = angle
    }

    fun moveBy(x: Double = 0.0, y: Double = 0.0, angle: Double = 0.0) = execute {
        this.position += P(x, y)
        this.angle += angle
    }

    fun liftTo(height: Double = 0.0) = execute {
        this.liftHeight = height
    }
}