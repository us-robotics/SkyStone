package org.firstinspires.ftc.teamcode.api

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.api.*

@RobotAPI
class RobotHardware(val opMode: LinearOpMode) {
    val driveLF = opMode.hardwareMap.dcMotor["motorFrontLeft"].init(reverse=false)
    val driveRF = opMode.hardwareMap.dcMotor["motorFrontRight"].init(reverse=true)
    val driveLB = opMode.hardwareMap.dcMotor["motorBackLeft"].init(reverse=false)
    val driveRB = opMode.hardwareMap.dcMotor["motorBackRight"].init(reverse=true)

    // Drive Motors
    //val driveL = opMode.hardwareMap.dcMotor["motorLeft"].init(reverse=true)
    //val driveR = opMode.hardwareMap.dcMotor["motorRight"].init(reverse=false    )
    //val strafeOne = opMode.hardwareMap.dcMotor["strafeOne"].init(reverse=true)
    //val strafeTwo = opMode.hardwareMap.dcMotor["strafeTwo"].init(reverse=false)
}