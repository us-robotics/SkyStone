package org.firstinspires.ftc.teamcode.api

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.api.*

@RobotAPI
class RobotHardware(val opMode: LinearOpMode) {
    val driveLF = opMode.hardwareMap.dcMotor["driveLF"].init(reverse=false)
    val driveRF = opMode.hardwareMap.dcMotor["driveRF"].init(reverse=true)
    val driveLB = opMode.hardwareMap.dcMotor["driveLB"].init(reverse=false)
    val driveRB = opMode.hardwareMap.dcMotor["driveRB"].init(reverse=true)

}