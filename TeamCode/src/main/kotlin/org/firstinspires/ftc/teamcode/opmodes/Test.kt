package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.api.*
import kotlin.math.*

@TeleOp(name="Test", group="Foo")
class Test : LinearOpMode() {
    override fun runOpMode() {
        val controller = RobotController(this)
        waitForStart()

        /*
        controller.auto {
            execute {
                position = P(0.0, 0.0)
                angle = 25.0
                liftHeight = 2.0
                grabFoundation = true
            }
            moveBy(angle = 25.0)
        }
        */

        /*controller.hardware {
            /*if (time < 5) {
                driveL.velocity = 150.0
                driveR.velocity = 150.0
            } else {
                driveL.velocity = 0.0
                driveR.velocity = 0.0
            }*/
            val r = hypot(gamepad1.left_stick_x, gamepad1.left_stick_y)
            val rAngle = atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - (PI / 4)
            val rightX = gamepad1.right_stick_x

            driveLF.power = r * cos(rAngle) + rightX
            driveRF.power = r * cos(rAngle) - rightX
            driveLB.power = r * cos(rAngle) + rightX
            driveRB.power = r * cos(rAngle) - rightX
        }*/

        controller.hardware {
            while (!gamepad1.a) {
                driveLF.power = (gamepad1.left_trigger * (if (gamepad1.left_bumper) -1 else 1)).toDouble()
                driveRF.power = (gamepad1.right_trigger * (if (gamepad1.right_bumper) -1 else 1)).toDouble()
                driveLB.power = -gamepad1.left_stick_y.toDouble()
                driveRB.power = -gamepad1.right_stick_y.toDouble()

                telemetry.addData("LF", driveLF.power)
                telemetry.addData("RF", driveRF.power)
                telemetry.addData("LB", driveLB.power)
                telemetry.addData("RB", driveRB.power)
                telemetry.update()
            }
        }

        controller.hardware {
            while (this@Test.opModeIsActive()) {
                val r = hypot(gamepad1.left_stick_x, gamepad1.left_stick_y)
                val robotAngle = atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - (PI / 4)
                val rightX = gamepad1.right_stick_x

                driveLF.power = r * cos(robotAngle) + rightX
                driveRF.power = r * sin(robotAngle) - rightX
                driveLB.power = r * sin(robotAngle) + rightX
                driveRB.power = r * cos(robotAngle) - rightX

                telemetry.addData("r", r)
                telemetry.addData("angle", robotAngle)
                telemetry.addData("rightX", rightX)
                telemetry.addData("LF", driveLF.power)
                telemetry.addData("RF", driveRF.power)
                telemetry.addData("LB", driveLB.power)
                telemetry.addData("RB", driveRB.power)
                telemetry.update()
            }
        }
    }
}