package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.api.*

@TeleOp(name="Test", group="Foo")
class Test : LinearOpMode() {
    override fun runOpMode() {
        val controller = RobotController(this)
        waitForStart()

        controller.auto {
            execute {
                position = P(0.0, 0.0)
                angle = 25.0
                liftHeight = 2.0
                grabFoundation = true
            }
            moveBy(angle = 25.0)



        }
    }
}